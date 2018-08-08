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
/* This is a simplified and adjusted version of the WarpRenderer              *
 * which was written by Simon Schneegans                                      */

#include <gua/renderer/WarpRenderer.hpp>

#include <gua/renderer/WarpPass.hpp>
#include <gua/config.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/WarpGridGenerator.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
//#include <gua/renderer/opengl_debugging.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/WindowDatabase.hpp>

#include <scm/core/math/math.h>


namespace gua {
  
////////////////////////////////////////////////////////////////////////////////
WarpRenderer::WarpRenderer()
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("shaders/warp_gbuffer.vert");
  std::string f_shader = factory.read_shader_file("shaders/warp_gbuffer.frag");
  std::string g_shader = factory.read_shader_file("shaders/warp_gbuffer.geom");
#else
  std::string v_shader = Resources::lookup_shader("shaders/warp_gbuffer.vert");
  std::string f_shader = Resources::lookup_shader("shaders/warp_gbuffer.frag");
  std::string g_shader = Resources::lookup_shader("shaders/warp_gbuffer.geom");
#endif

  warp_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  warp_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));
  warp_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  v_shader = factory.read_shader_file("shaders/warp_abuffer.vert");
  f_shader = factory.read_shader_file("shaders/warp_abuffer.frag");
#else
  v_shader = Resources::lookup_shader("shaders/warp_abuffer.vert");
  f_shader = Resources::lookup_shader("shaders/warp_abuffer.frag");
#endif

  warp_abuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  warp_abuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  v_shader = factory.read_shader_file("shaders/common/fullscreen_quad.vert");
  f_shader = factory.read_shader_file("shaders/hole_filling_texture.frag");
#else
  v_shader = Resources::lookup_shader("shaders/common/fullscreen_quad.vert");
  f_shader = Resources::lookup_shader("shaders/hole_filling_texture.frag");
#endif

  hole_filling_texture_program_ = std::make_shared<ShaderProgram>();
  hole_filling_texture_program_->create_from_sources(v_shader, f_shader);

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  v_shader = factory.read_shader_file("shaders/warp_grid_shader.vert");
  f_shader = factory.read_shader_file("shaders/warp_grid_shader.frag");
  g_shader = factory.read_shader_file("shaders/warp_grid_shader.geom");
#else
  v_shader = Resources::lookup_shader("shaders/warp_grid_shader.vert");
  f_shader = Resources::lookup_shader("shaders/warp_grid_shader.frag");
  g_shader = Resources::lookup_shader("shaders/warp_grid_shader.geom");
#endif

  render_grid_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  render_grid_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));
  render_grid_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

////////////////////////////////////////////////////////////////////////////////
WarpRenderer::~WarpRenderer()
{
  if(res_) {
    if (color_buffer_ && color_buffer_) {
      pipe_->get_context().render_context->make_non_resident(color_buffer_);
    }
    if (depth_buffer_ && depth_buffer_) {
      pipe_->get_context().render_context->make_non_resident(depth_buffer_);
    }
    if (hole_filling_texture_ && hole_filling_texture_) {
      hole_filling_texture_->make_non_resident(pipe_->get_context());
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void WarpRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc) {
  // std::cout << "Warping GBuffer ..." << std::endl;
  auto ctx(pipe.get_context());
  pipe_ = &pipe;

  pipe.begin_gpu_query(ctx, "Warping");

  auto description(dynamic_cast<WarpPassDescription const*>(&desc));
  math::vec2ui resolution(pipe.current_viewstate().camera.config.get_resolution());

  auto gbuffer = dynamic_cast<GBuffer*>(pipe.current_viewstate().target);
  // get and create resources --------------------------------------------------
  if (!res_) {
    res_ = description->warp_resources();
    for (int i = 0; i < 2; ++i) {
      res_->warp_vao[i] = ctx.render_device->create_vertex_array(
            scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3UI, sizeof(math::vec3ui)), {res_->grid_vbo.first[i]});
    }
  }
  if (!warp_gbuffer_program_) {
    warp_gbuffer_program_ = std::make_shared<gua::ShaderProgram>();
    warp_gbuffer_program_->set_shaders(warp_gbuffer_program_stages_, std::list<std::string>(), false, global_substitution_map_);

    warp_abuffer_program_ = std::make_shared<ShaderProgram>();
    warp_abuffer_program_->set_shaders(warp_abuffer_program_stages_, std::list<std::string>(), false, global_substitution_map_);

    scm::gl::rasterizer_state_desc p_desc;
    p_desc._point_state = scm::gl::point_raster_state(true);
    // p_desc._fill_mode = scm::gl::fill_mode::FILL_WIREFRAME;
    points_ = ctx.render_device->create_rasterizer_state(p_desc);

    depth_stencil_state_yes_ = ctx.render_device->create_depth_stencil_state(
        true, true, scm::gl::COMPARISON_LESS, true, 1, 0,
        scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    );

    depth_stencil_state_no_ = ctx.render_device->create_depth_stencil_state(
        false, false, scm::gl::COMPARISON_LESS, true, 1, 0,
        scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    );

    auto empty_vbo = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, sizeof(math::vec2), 0);
    empty_vao_ = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC2F, sizeof(math::vec2)), {empty_vbo});

    scm::gl::sampler_state_desc state_desc(scm::gl::FILTER_MIN_MAG_NEAREST,
      scm::gl::WRAP_CLAMP_TO_EDGE,
      scm::gl::WRAP_CLAMP_TO_EDGE);
    scm::gl::sampler_state_ptr state = ctx.render_device->create_sampler_state(state_desc);

    // std::cout << "[WARP] color buffer.first adress: " << res_->color_buffer.first->native_handle() << std::endl;
    // color_buffer_ = res_->color_buffer.first;
    // depth_buffer_ = res_->depth_buffer.first;

    color_buffer_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
    ctx.render_context->make_resident(color_buffer_, state);
    depth_buffer_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
    ctx.render_context->make_resident(depth_buffer_, state);
    // color_buffer_ = gbuffer->get_color_buffer();
    // depth_buffer_ = gbuffer->get_depth_buffer();

    fbo_ = ctx.render_device->create_frame_buffer();
    fbo_->attach_color_buffer(0, color_buffer_,0,0);
    fbo_->attach_depth_stencil_buffer(depth_buffer_,0,0);

    if (description->hole_filling_mode() == WarpPassDescription::HOLE_FILLING_BLUR) {
      int holefilling_levels = 6;
      scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_MIP_LINEAR,
        scm::gl::WRAP_CLAMP_TO_EDGE,
        scm::gl::WRAP_CLAMP_TO_EDGE);
      hole_filling_texture_ = std::make_shared<Texture2D>(resolution.x/2, resolution.y/2,
        scm::gl::FORMAT_RGBA_32F, holefilling_levels, state);

      for (int i = 0; i < holefilling_levels; ++i) {
        hole_filling_fbos_.push_back(ctx.render_device->create_frame_buffer());
        hole_filling_fbos_.back()->attach_color_buffer(0, hole_filling_texture_->get_buffer(ctx),i,0);
      }
    }
  }

  // res_->swap_shared_resources();

  if (description->depth_test()) {
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_yes_, 1);
  } else {
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_, 1);
  }

  // ---------------------------------------------------------------------------
  // ---------------------------- WarpMatrix -----------------------------------
  // ---------------------------------------------------------------------------
  auto stereo_type(pipe.current_viewstate().camera.config.stereo_type());
  //auto stereo_type(StereoType::SPATIAL_WARP);

  bool first_eye(
       (stereo_type == StereoType::SPATIAL_WARP  && ctx.mode != CameraMode::RIGHT)
    || (stereo_type == StereoType::TEMPORAL_WARP && (ctx.framecount % 2 == 0) == (ctx.mode == CameraMode::RIGHT))
    || (stereo_type == StereoType::SINGLE_TEMPORAL_WARP && (ctx.framecount % 2 == 0) == (ctx.mode == CameraMode::RIGHT))
  );

  bool first_warp(
       (stereo_type == StereoType::SPATIAL_WARP && first_eye)
    || (stereo_type == StereoType::TEMPORAL_WARP && !first_eye)
    || (stereo_type == StereoType::SINGLE_TEMPORAL_WARP && !first_eye)
  );

  bool perform_warp(
    !(stereo_type == StereoType::SINGLE_TEMPORAL_WARP && first_warp)
  );

  if (first_eye) {
    //cached_warp_state_ = description->get_warp_state()();
    cached_warp_state_ = res_->warp_state;
  }
 
  gua::math::mat4d proj;
  gua::math::mat4d view;
  gua::math::mat4d warp;

  if (first_eye && stereo_type == StereoType::TEMPORAL_WARP) {
    proj = last_frustum_.get_projection();
    view = last_frustum_.get_view();
    //warp = cached_warp_state_.get(ctx.mode);
    cached_warp_state_ = res_->warp_state;
  } else if (first_eye && stereo_type == StereoType::SINGLE_TEMPORAL_WARP) {
    proj = last_frustum_.get_projection();
    view = last_frustum_.get_view();

    auto cur(Frustum::perspective(
      pipe.current_viewstate().frustum.get_camera_transform(),
      pipe.current_viewstate().frustum.get_screen_transform(),
      pipe.current_viewstate().frustum.get_clip_near(),
      pipe.current_viewstate().frustum.get_clip_far()*1.5
    ));

    warp = cur.get_projection() * cur.get_view();

  } else {
    proj = pipe.current_viewstate().frustum.get_projection();
    view = pipe.current_viewstate().frustum.get_view();
    warp = cached_warp_state_.get(ctx.mode);
    //cached_warp_state_ = res_->warp_state;
  }

  if (first_warp) {
    gbuffer->bind(ctx, false);
    gbuffer->toggle_ping_pong();
    last_frustum_ = pipe.current_viewstate().frustum;
  }
  /* print_matrix(proj, "Projection");
  print_matrix(view, "View");
  print_matrix(proj * view, "ProjectionView");
  print_matrix(warp, "Cached ProjectionView"); */

  math::mat4f warp_matrix(warp * scm::math::inverse(proj * view));
  math::mat4f inv_warp_matrix(scm::math::inverse(warp * scm::math::inverse(proj * view)));

  print_matrix(warp_matrix, "Resulting WarpMatrix");

  // ---------------------------------------------------------------------------
  // --------------------------------- Warp Gbuffer ----------------------------
  // ---------------------------------------------------------------------------
  // if (first_warp) {
  //   gbuffer->get_abuffer().update_min_max_buffer();
  // }

  if (perform_warp) {
    ctx.render_context->set_frame_buffer(fbo_);
    gbuffer->set_viewport(ctx);
    ctx.render_context->clear_color_buffers(
        fbo_, scm::math::vec4f(0,0,0,0));
    ctx.render_context->clear_depth_stencil_buffer(fbo_);

    {
      warp_gbuffer_program_->use(ctx);
      warp_gbuffer_program_->apply_uniform(ctx, "warp_matrix", warp_matrix);

      pipe.bind_gbuffer_input(warp_gbuffer_program_);

      uint64_t h = res_->color_buffer.first->native_handle();
      // std::cout << "[WARP] color buffer.first adress: " << h << std::endl;

      // uint64_t h = gbuffer->get_color_buffer()->native_handle();
      math::vec2ui handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);

      warp_gbuffer_program_->set_uniform(ctx,
                          handle,
                          "gua_gbuffer_color");

      h = gbuffer->get_pbr_buffer()->native_handle();
      handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);

      warp_gbuffer_program_->set_uniform(ctx,
                          handle,
                          "gua_gbuffer_pbr");

      h = res_->depth_buffer.first->native_handle();
      // h = gbuffer->get_depth_buffer()->native_handle();
      handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);

      warp_gbuffer_program_->set_uniform(ctx,
                          handle,
                          "gua_gbuffer_depth");

      if (description->gbuffer_warp_mode() == WarpPassDescription::GBUFFER_GRID_SURFACE_ESTIMATION ||
          description->gbuffer_warp_mode() == WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION ||
          description->gbuffer_warp_mode() == WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION) {
        if (res_) {
          h = res_->surface_detection_buffer.first->native_handle();
          sdb_handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);

          warp_gbuffer_program_->set_uniform(ctx, sdb_handle,  "gua_warp_grid_tex");
          ctx.render_context->bind_vertex_array(res_->warp_vao[res_->current_vbo()]);
          ctx.render_context->apply();
          // std::cout << "\n[WARP_RENDERER] grid_tfb id (current vbo): " << res_->grid_tfb.first[res_->current_vbo()]->object_id() << std::endl;
          // std::cout << "\n[WARP_RENDERER] grid_tfb id (current tfb): " << res_->grid_tfb.first[res_->current_tfb()]->object_id() << std::endl;
          ctx.render_context->draw_transform_feedback(scm::gl::PRIMITIVE_POINT_LIST, res_->grid_tfb.first[res_->current_vbo()]);
        }
      } else {
        ctx.render_context->set_rasterizer_state(points_);
        ctx.render_context->bind_vertex_array(empty_vao_);
        ctx.render_context->apply();
        ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, resolution.x * resolution.y);
      }
    }
  }

  // ---------------------------------------------------------------------------
  // --------------------- warp abuffer & hole filling -------------------------
  // ---------------------------------------------------------------------------

  // if (perform_warp && description->hole_filling_mode() == WarpPassDescription::HOLE_FILLING_BLUR) {
  //   ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_, 1);

  //   hole_filling_texture_program_->use(ctx);
  //   uint64_t h = color_buffer_->native_handle();
  //   cb_handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);
  //   hole_filling_texture_program_->set_uniform(ctx, cb_handle, "color_buffer");

  //   h = depth_buffer_->native_handle();
  //   db_handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);
  //   hole_filling_texture_program_->set_uniform(ctx, db_handle, "depth_buffer");
  //   hole_filling_texture_program_->set_uniform(ctx, hole_filling_texture_->get_handle(ctx), "hole_filling_texture");

  //   for (int i = 0; i < hole_filling_fbos_.size(); ++i) {
  //     math::vec2ui level_size(scm::gl::util::mip_level_dimensions(math::vec2ui(hole_filling_texture_->width(), hole_filling_texture_->height()), i));
  //     ctx.render_context->set_frame_buffer(hole_filling_fbos_[i]);
  //     ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(level_size)));
  //     hole_filling_texture_program_->set_uniform(ctx, i, "current_level");
  //     pipe_->draw_quad();
  //   }
  // }
  bool debug_grid = false;
  
  if (debug_grid) {
    render_grid(pipe, desc, warp_matrix);
  } else {  
    gbuffer->set_viewport(ctx);
    
    warp_abuffer_program_->use(ctx);
    warp_abuffer_program_->apply_uniform(ctx, "warp_matrix", warp_matrix);
    warp_abuffer_program_->apply_uniform(ctx, "inv_warp_matrix", inv_warp_matrix);
    warp_abuffer_program_->apply_uniform(ctx, "perform_warp", perform_warp);

    //gbuffer->get_abuffer().bind_min_max_buffer(warp_abuffer_program_);

    bool write_all_layers = false;
    bool do_clear = false;
    bool do_swap = false;
    gbuffer->bind(ctx, write_all_layers);

    if (description->hole_filling_mode() == WarpPassDescription::HOLE_FILLING_BLUR) {
      warp_abuffer_program_->set_uniform(ctx, hole_filling_texture_->get_handle(ctx), "hole_filling_texture");
    }

    if (perform_warp) {
      uint64_t h = color_buffer_->native_handle();
      // uint64_t h = res_->color_buffer.first->native_handle();
      math::vec2ui handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);
      warp_abuffer_program_->set_uniform(ctx, handle, "warped_color_buffer");

      h = depth_buffer_->native_handle();
      // h = res_->depth_buffer.first->native_handle();
      handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);
      warp_abuffer_program_->set_uniform(ctx, handle, "warped_depth_buffer");
    } else {
      uint64_t h = gbuffer->get_pbr_buffer()->native_handle();
      math::vec2ui handle(h & 0x00000000ffffffff, h & 0xffffffff00000000);
      warp_abuffer_program_->set_uniform(ctx, handle, "orig_pbr_buffer");

      h = res_->color_buffer.first->native_handle();
      handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);

      // h = gbuffer->get_color_buffer()->native_handle();
      // handle = math::vec2ui (h & 0x00000000ffffffff, h & 0xffffffff00000000);
      warp_abuffer_program_->set_uniform(ctx, handle, "warped_color_buffer");

      h = res_->depth_buffer.first->native_handle();
      handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);

      // h = gbuffer->get_depth_buffer()->native_handle();
      // handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);
      warp_abuffer_program_->set_uniform(ctx, handle, "warped_depth_buffer");
    }

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_, 1);
    ctx.render_context->apply();
    pipe.draw_quad();
  }

  // gbuffer->toggle_ping_pong();
  gbuffer->unbind(ctx);

  ctx.render_context->reset_state_objects();

  res_->updated = true;
  pipe.end_gpu_query(ctx, "Warping");

}
////////////////////////////////////////////////////////////////////////////////

void WarpRenderer::render_grid(Pipeline& pipe, PipelinePassDescription const& desc, math::mat4f warp_matrix)
{
  auto description(dynamic_cast<WarpPassDescription const*>(&desc));

  // print_matrix(warp_matrix, "Warp Matrix");

  if (!render_grid_program_) {
    render_grid_program_ = std::make_shared<ShaderProgram>();
    render_grid_program_->set_shaders(render_grid_program_stages_, std::list<std::string>(), false);
  }

  auto& ctx(pipe.get_context());

  if (!depth_stencil_state_grid_) {
    depth_stencil_state_grid_ = ctx.render_device->create_depth_stencil_state(
      false, false, scm::gl::COMPARISON_LESS, true, 1, 0,
      scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    );
    rasterizer_state_ = ctx.render_device->create_rasterizer_state(
      scm::gl::FILL_SOLID,
      scm::gl::CULL_NONE,
      scm::gl::ORIENT_CCW,
      false,
      false,
      0.0f,
      false,
      true,
      scm::gl::point_raster_state(true));
    blend_state_ = ctx.render_device->create_blend_state(scm::gl::blend_ops(true,
      scm::gl::FUNC_ONE,
      scm::gl::FUNC_ONE,
      scm::gl::FUNC_ONE,
      scm::gl::FUNC_ONE), false);
  }

  if(!res_) {
    res_ = description->warp_resources();
  }
    

  if (res_) {
    auto& target = *pipe.current_viewstate().target;

    bool write_all_layers = false;
    bool do_clear = false;
    bool do_swap = false;
    target.bind(ctx, write_all_layers);
    target.set_viewport(ctx);
    // std::cout << "[WARP] doin some warpin ..." << std::endl;
    render_grid_program_->use(ctx);
    uint64_t h = res_->depth_buffer.first->native_handle();
    // h = gbuffer->get_depth_buffer()->native_handle();
    math::vec2ui handle = math::vec2ui(h & 0x00000000ffffffff, h & 0xffffffff00000000);

    render_grid_program_->set_uniform(ctx, handle, "gua_gbuffer_depth");
    render_grid_program_->apply_uniform(ctx, "warp_matrix", warp_matrix);

    ctx.render_context->set_blend_state(blend_state_);
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_grid_, 1);
    ctx.render_context->bind_vertex_array(res_->grid_vao.first[res_->current_vbo()]);
    ctx.render_context->apply();
    ctx.render_context->draw_transform_feedback(scm::gl::PRIMITIVE_POINT_LIST, res_->grid_tfb.first[res_->current_vbo()]);

    target.unbind(ctx);

    ctx.render_context->reset_state_objects();
  }
}




}