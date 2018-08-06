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

#include <gua/renderer/WarpGridGenerator.hpp>

// #include <gua/renderer/opengl_debugging.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/WarpGridGeneratorPass.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
WarpGridGenerator::WarpGridGenerator()
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("shaders/warp_grid_generate_shader.vert");
  std::string g_shader = factory.read_shader_file("shaders/warp_grid_generate_shader.geom");
#else
  std::string v_shader = Resources::lookup_shader("shaders/warp_grid_generate_shader.vert");
  std::string g_shader = Resources::lookup_shader("shaders/warp_grid_generate_shader.geom");
#endif

  grid_generation_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  grid_generation_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));


// #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
//   v_shader = factory.read_shader_file("shaders/common/fullscreen_quad.vert");
//   g_shader = factory.read_shader_file("shaders/surface_detection_shader.frag");
// #else
//   v_shader = Resources::lookup_shader("shaders/common/fullscreen_quad.vert");
//   g_shader = Resources::lookup_shader("shaders/surface_detection_shader.frag");
// #endif

//   surface_detection_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
//   surface_detection_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, g_shader));
}

////////////////////////////////////////////////////////////////////////////////
WarpGridGenerator::~WarpGridGenerator() {
  if (res_ && res_->surface_detection_buffer.first && res_->surface_detection_buffer.first) {
    pipe_->get_context().render_context->make_non_resident(res_->surface_detection_buffer.first);
    pipe_->get_context().render_context->make_non_resident(res_->surface_detection_buffer.first);
  }
}

////////////////////////////////////////////////////////////////////////////////
void WarpGridGenerator::render(Pipeline& pipe, PipelinePassDescription const& desc) {
  // std::cout << "Generating Grid ..." << std::endl;
  // if(res_) {
  //   if(res_->grid_generated) {res_->grid_generated = false;}
  // }
  auto& ctx(pipe.get_context());

  pipe_ = &pipe;

  auto description(dynamic_cast<WarpGridGeneratorPassDescription const*>(&desc));

  math::vec2ui resolution(pipe.current_viewstate().camera.config.get_resolution());
  size_t pixel_count(resolution.x * resolution.y / 4);

  if (!grid_generation_program_) {
    std::list<std::string> list_temp;
    list_temp.push_back("xfb_output");
    grid_generation_program_ = std::make_shared<ShaderProgram>();
    grid_generation_program_->set_shaders(grid_generation_program_stages_, list_temp, true, global_substitution_map_);
  }

  // if (!surface_detection_program_) {
  //   surface_detection_program_ = std::make_shared<ShaderProgram>();
  //   surface_detection_program_->set_shaders(surface_detection_program_stages_, std::list<std::string>(), false, global_substitution_map_);
  // }


  int current_level(std::log2(description->cell_size()));

  // get warping resources -----------------------------------------------------
  if (!res_) {
    res_ = description->warp_resources();

    /*if (res_->surface_detection_buffer.first) {
      ctx.render_context->make_non_resident(res_->surface_detection_buffer.first);
      ctx.render_context->make_non_resident(res_->surface_detection_buffer.first);
    }*/
    if (!res_->grid_vbo.first.size() == 0 || res_->cell_count < pixel_count) {
      res_->init_grid_resources(ctx, pixel_count);
    }

    math::vec2 size(resolution/2);
    int mip_map_levels(current_level);
    scm::gl::sampler_state_desc state_desc(scm::gl::FILTER_MIN_MAG_NEAREST,
      scm::gl::WRAP_CLAMP_TO_EDGE,
      scm::gl::WRAP_CLAMP_TO_EDGE);
    scm::gl::sampler_state_ptr state = ctx.render_device->create_sampler_state(state_desc);

    /*res_->surface_detection_buffer.first = ctx.render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
    ctx.render_context->make_resident(res_->surface_detection_buffer.first, state);
    res_->surface_detection_buffer.first = ctx.render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
    ctx.render_context->make_resident(res_->surface_detection_buffer.first, state);*/

    /*res_->surface_detection_buffer.first = std::make_shared<Texture2D>(size.x, size.y,
        scm::gl::FORMAT_R_16UI, mip_map_levels, state);
    res_->surface_detection_buffer.first = std::make_shared<Texture2D>(size.x, size.y,
        scm::gl::FORMAT_R_16UI, mip_map_levels, state);*/

    // surface_detection_buffer_fbos_.clear();

    // for (int i(0); i<mip_map_levels; ++i) {
    //   surface_detection_buffer_fbos_.push_back(ctx.render_device->create_frame_buffer());
    //   surface_detection_buffer_fbos_.back()->attach_color_buffer(0, res_->surface_detection_buffer.first, i,0);
    // }
  }

  auto gbuffer = dynamic_cast<GBuffer*>(pipe.current_viewstate().target);

  // ---------------------------------------------------------------------------
  // ------------------- Surface Information Map -------------------------------
  // ---------------------------------------------------------------------------

  // surface_detection_program_->use(ctx);
  // uint64_t h = gbuffer->get_depth_buffer()->native_handle();
  // math::vec2ui handle(h & 0x00000000ffffffff, h & 0xffffffff00000000);
  // surface_detection_program_->set_uniform(ctx, handle, "depth_buffer");
  // h = res_->surface_detection_buffer.first->native_handle();
  // handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);
  // surface_detection_program_->set_uniform(ctx, handle, "surface_detection_buffer");

  // for (int i = 0; i < surface_detection_buffer_fbos_.size(); ++i) {
  //   math::vec2ui level_size(scm::gl::util::mip_level_dimensions(resolution / 2, i));
  //   ctx.render_context->set_frame_buffer(surface_detection_buffer_fbos_[i]);
  //   ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(level_size)));
  //   surface_detection_program_->set_uniform(ctx, i, "current_level");
  //   pipe.draw_quad();
  // }

  // ---------------------------------------------------------------------------
  // --------------------- Generate Warp Grid ----------------------------------
  // ---------------------------------------------------------------------------
  unsigned initial_grid_x(std::ceil((float)resolution.x / description->cell_size()));
  unsigned initial_grid_y(std::ceil((float)resolution.y / description->cell_size()));

  {
    // upload initial data
    auto data = static_cast<math::vec3ui*>(ctx.render_context->map_buffer(
        res_->grid_vbo.first[res_->current_vbo()], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

	  for (unsigned x(0); x < initial_grid_x; ++x) {
		  for (unsigned y(0); y < initial_grid_y; ++y) {
        data[y*initial_grid_x + x] = math::vec3ui(x * description->cell_size(),
                                                  y * description->cell_size(),
                                                  current_level << 12 /* write the current mipmap level at bit position 12 */);
      }
    }

    ctx.render_context->unmap_buffer(res_->grid_vbo.first[res_->current_vbo()]);
  }

  uint64_t h = res_->surface_detection_buffer.first->native_handle();
  math::vec2ui handle(h & 0x00000000ffffffff, h & 0xffffffff00000000);
  // surface_detection_program_->set_uniform(ctx, handle, "surface_detection_buffer");

  grid_generation_program_->use(ctx);
  grid_generation_program_->set_uniform(ctx, handle, "surface_detection_buffer");
  grid_generation_program_->set_uniform(ctx, current_level, "current_level");

  // first subdivision
  ctx.render_context->begin_transform_feedback(res_->grid_tfb.first[res_->current_tfb()],
    scm::gl::PRIMITIVE_POINTS);
  {
    ctx.render_context->bind_vertex_array(res_->grid_vao.first[res_->current_vbo()]);
    ctx.render_context->apply();
    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, initial_grid_x*initial_grid_y);
  }

  ctx.render_context->end_transform_feedback();
  res_->ping = !res_->ping;

  while (--current_level > 1) {
    // further subdivisions
    grid_generation_program_->set_uniform(ctx, current_level, "current_level");
    ctx.render_context->begin_transform_feedback(res_->grid_tfb.first[res_->current_tfb()],
      scm::gl::PRIMITIVE_POINTS);
    {
      ctx.render_context->bind_vertex_array(res_->grid_vao.first[res_->current_vbo()]);
      ctx.render_context->apply();
      ctx.render_context->draw_transform_feedback(scm::gl::PRIMITIVE_POINT_LIST, res_->grid_tfb.first[res_->current_vbo()]);
    }

    ctx.render_context->end_transform_feedback();
    res_->ping = !res_->ping;
  }

  // auto buffer = res_->grid_tfb.first[res_->current_vbo()]->stream_out_buffer(0);
  // auto buffer_2 = ctx.render_device->create_buffer(buffer->descriptor()._bindings, buffer->descriptor()._usage, buffer->descriptor()._size);
  // ctx.render_context->copy_buffer_data(res_->grid_vbo_warp.first[res_->current_vbo()],buffer,0,0,buffer->descriptor()._size);

  // res_->grid_generated = true;
  ctx.render_context->reset_state_objects();

}

////////////////////////////////////////////////////////////////////////////////
}  // namespace gua