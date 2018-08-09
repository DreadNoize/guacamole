/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2018 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *            joachim.billert@uni-weimar.de                                   *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/
/* This is a simplified and adjusted version of the WarpGridGenerator         *
 * which was written by Simon Schneegans                                      */

#include <gua/renderer/SurfaceDetector.hpp>

// #include <gua/renderer/opengl_debugging.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/SurfaceDetectionPass.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
SurfaceDetector::SurfaceDetector()
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("shaders/common/fullscreen_quad.vert");
  std::string f_shader = factory.read_shader_file("shaders/surface_detection_shader.frag");
#else
  std::string v_shader = Resources::lookup_shader("shaders/common/fullscreen_quad.vert");
  std::string f_shader = Resources::lookup_shader("shaders/surface_detection_shader.frag");
#endif

  surface_detection_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  surface_detection_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

////////////////////////////////////////////////////////////////////////////////
SurfaceDetector::~SurfaceDetector() {
  if (res_ && res_->surface_detection_buffer.first && res_->surface_detection_buffer.second) {
    pipe_->get_context().render_context->make_non_resident(res_->surface_detection_buffer.first);
    pipe_->get_context().render_context->make_non_resident(res_->surface_detection_buffer.second);
  }
}

////////////////////////////////////////////////////////////////////////////////
void SurfaceDetector::render(Pipeline& pipe, PipelinePassDescription const& desc) {
  // std::cout << "Surface Detection ..." << std::endl;

  
  if(res_) {
    if(res_->grid_generated) {res_->grid_generated = false;}
  }
  auto& ctx(pipe.get_context());
  pipe.begin_gpu_query(ctx, "Surface Detection");

  pipe_ = &pipe;

  auto description(dynamic_cast<SurfaceDetectionPassDescription const*>(&desc));

  math::vec2ui resolution(pipe.current_viewstate().camera.config.get_resolution());
  size_t pixel_count(resolution.x * resolution.y / 4);

  if (!surface_detection_program_) {
    surface_detection_program_ = std::make_shared<ShaderProgram>();
    surface_detection_program_->set_shaders(surface_detection_program_stages_, std::list<std::string>(), false, global_substitution_map_);
  }


  int current_level(std::log2(description->cell_size()));

  int mip_map_levels(current_level);
  // get warping resources -----------------------------------------------------
  if(!res_) {
    res_ = description->warp_resources();

    /*if (res_->surface_detection_buffer.second) {
      ctx.render_context->make_non_resident(res_->surface_detection_buffer.first);
      ctx.render_context->make_non_resident(res_->surface_detection_buffer.second);
    }*/

    math::vec2 size(resolution/2);
    scm::gl::sampler_state_desc state_desc(scm::gl::FILTER_MIN_MAG_NEAREST,
      scm::gl::WRAP_CLAMP_TO_EDGE,
      scm::gl::WRAP_CLAMP_TO_EDGE);
    scm::gl::sampler_state_ptr state = ctx.render_device->create_sampler_state(state_desc);

    /*res_->surface_detection_buffer.first = ctx.render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
    ctx.render_context->make_resident(res_->surface_detection_buffer.first, state);
    res_->surface_detection_buffer.second = ctx.render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
    ctx.render_context->make_resident(res_->surface_detection_buffer.second, state);*/

    /*res_->surface_detection_buffer.first = std::make_shared<Texture2D>(size.x, size.y,
        scm::gl::FORMAT_R_16UI, mip_map_levels, state);
    res_->surface_detection_buffer.second = std::make_shared<Texture2D>(size.x, size.y,
        scm::gl::FORMAT_R_16UI, mip_map_levels, state);*/

    surface_detection_buffer_fbos_.clear();
    for (int i(0); i<mip_map_levels; ++i) {
      surface_detection_buffer_fbos_.push_back(ctx.render_device->create_frame_buffer());
    }
  }
  for (int i(0); i<mip_map_levels; ++i) {
    surface_detection_buffer_fbos_[i]->clear_attachments();
    surface_detection_buffer_fbos_[i]->attach_color_buffer(0, res_->surface_detection_buffer.second, i,0);
  }
  // ---------------------------------------------------------------------------
  // ------------------- Surface Information Map -------------------------------
  // ---------------------------------------------------------------------------
  auto gbuffer = dynamic_cast<GBuffer*>(pipe.current_viewstate().target);

  surface_detection_program_->use(ctx);
  uint64_t h = gbuffer->get_depth_buffer()->native_handle();
  math::vec2ui handle(h & 0x00000000ffffffff, h & 0xffffffff00000000);
  surface_detection_program_->set_uniform(ctx, handle, "depth_buffer");
  h = res_->surface_detection_buffer.second->native_handle();
  handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);
  surface_detection_program_->set_uniform(ctx, handle, "surface_detection_buffer");

  for (int i = 0; i < surface_detection_buffer_fbos_.size(); ++i) {
    math::vec2ui level_size(scm::gl::util::mip_level_dimensions(resolution / 2, i));
    ctx.render_context->set_frame_buffer(surface_detection_buffer_fbos_[i]);
    ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(level_size)));
    surface_detection_program_->set_uniform(ctx, i, "current_level");
    pipe.draw_quad();
  }
  pipe.end_gpu_query(ctx, "Surface Detection");

  res_->grid_generated = true;


}

////////////////////////////////////////////////////////////////////////////////
}  // namespace gua