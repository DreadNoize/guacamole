/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
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

#ifndef GUA_RENDERER_HPP
#define GUA_RENDERER_HPP

// external headers
#include <vector>
#include <string>
#include <memory>
#include <map>

#include <gua/platform.hpp>
#include <gua/math.hpp>
#include <gua/utils/FpsCounter.hpp>
#include <gua/concurrent/Doublebuffer.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Texture2D.hpp>

#include <scm/gl_core/texture_objects/texture_objects_fwd.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/constants.h>
#include <scm/gl_core/render_device/opengl/util/error_helper.h>
//#include <scm/gl_core/render_device/opengl/util/constants_helper.h>
#include <scm/gl_core/render_device/opengl/util/data_format_helper.h>
#include <scm/gl_core/render_device/opengl/GL/glcorearb.h>
#include <scm/gl_core.h>
#include <scm/gl_core/sync_objects.h>

//#define GL_PIXEL_PACK_BUFFER              0x88EB

struct GLFWwindow;

namespace gua {

class SceneGraph;

namespace node {
  struct SerializedCameraNode;
  class CameraNode;
}

/**
 * Manages the rendering on multiple contexts.
 *
 * This class is used to provide a renderer frontend interface to the user.
 */
class GUA_DLL Renderer {
 public:
  // Container struct for resources (like textures) needed for the asynchronous warping technique
  // using DBTexture = std::shared_ptr<gua::concurrent::Doublebuffer<scm::gl::texture_2d_ptr>>;
  struct WarpingResources {
    
    struct WarpState {
      math::mat4f projection_view_center;
      math::mat4f projection_view_right;
      math::mat4f projection_view_left;

      math::mat4f const& get(CameraMode mode) {
        if (mode == CameraMode::LEFT)  return projection_view_left;
        if (mode == CameraMode::RIGHT) return projection_view_right;
        return projection_view_center;
      } 
    } warp_state;

    WarpingResources() = default;

    WarpingResources& operator=(WarpingResources const& rhs) {
      if (this != &rhs) { 
        std::lock(copy_mutex, rhs.copy_mutex);
        std::lock_guard<std::mutex> m_lhs(copy_mutex, std::adopt_lock);
        std::lock_guard<std::mutex> m_rhs(rhs.copy_mutex, std::adopt_lock);
        color_buffer_left = rhs.color_buffer_left;
        depth_buffer_left = rhs.depth_buffer_left;
        sampler_state_desc = rhs.sampler_state_desc;
        sampler_state = rhs.sampler_state;
        is_left = rhs.is_left;
        renderer_ready = rhs.renderer_ready;
        updated = rhs.updated;
        initialized = rhs.initialized;
        initialized_fbo = rhs.initialized_fbo;
        shared_initialized = rhs.shared_initialized;
      }
      return *this;
    }

    void init(gua::RenderContext* ctx, gua::math::vec2ui const& resolution) {
      // synch = "unsynched";
      // is_left = std::make_pair<bool,bool>(false,false);

      sampler_state_desc = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);
      // std::cout << "Initializing Warping Sampler State ..." << std::endl; 
      sampler_state = ctx->render_device->create_sampler_state(sampler_state_desc);
      
      // std::cout << "Initializing Warping Texture Color ..." << std::endl; 
      color_buffer_left = std::make_tuple(ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGBA_32F, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGBA_32F, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGBA_32F, 1));
      ctx->render_context->make_resident(std::get<0>(color_buffer_left), sampler_state);
      ctx->render_context->make_resident(std::get<1>(color_buffer_left), sampler_state);
      ctx->render_context->make_resident(std::get<2>(color_buffer_left), sampler_state);

      color_buffer_right = std::make_tuple(ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGBA_32F, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGBA_32F, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGBA_32F, 1));
      ctx->render_context->make_resident(std::get<0>(color_buffer_right), sampler_state);
      ctx->render_context->make_resident(std::get<1>(color_buffer_right), sampler_state);
      ctx->render_context->make_resident(std::get<2>(color_buffer_right), sampler_state);


      // std::cout << "Initializing Warping Texture Depth ..." << std::endl; 

      depth_buffer_left = std::make_tuple(ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1));
      ctx->render_context->make_resident(std::get<0>(depth_buffer_left), sampler_state);
      ctx->render_context->make_resident(std::get<1>(depth_buffer_left), sampler_state);
      ctx->render_context->make_resident(std::get<2>(depth_buffer_left), sampler_state);

      depth_buffer_right = std::make_tuple(ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1),
                                     ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1));
      ctx->render_context->make_resident(std::get<0>(depth_buffer_right), sampler_state);
      ctx->render_context->make_resident(std::get<1>(depth_buffer_right), sampler_state);
      ctx->render_context->make_resident(std::get<2>(depth_buffer_right), sampler_state);


      math::vec2 size(resolution / 2);

      int current_level(std::log2(32));
      int mip_map_levels(current_level);

      scm::gl::sampler_state_desc state_desc(scm::gl::FILTER_MIN_MAG_NEAREST,
                                             scm::gl::WRAP_MIRRORED_REPEAT,
                                             scm::gl::WRAP_MIRRORED_REPEAT);
      scm::gl::sampler_state_ptr state = ctx->render_device->create_sampler_state(state_desc);

      // std::get<0>(surface_detection_buffer_left) = ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
      // ctx->render_context->make_resident(std::get<0>(surface_detection_buffer_left), state);
      // std::get<2>(surface_detection_buffer_left) = ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
      // ctx->render_context->make_resident(std::get<2>(surface_detection_buffer_left), state);
      surface_detection_buffer_left = std::make_tuple(ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels),
                                                 ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels),
                                                 ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels));
      ctx->render_context->make_resident(std::get<0>(surface_detection_buffer_left), state);
      ctx->render_context->make_resident(std::get<1>(surface_detection_buffer_left), state);
      ctx->render_context->make_resident(std::get<2>(surface_detection_buffer_left), state);

      surface_detection_buffer_right = std::make_tuple(ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels),
                                                 ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels),
                                                 ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels));
      ctx->render_context->make_resident(std::get<0>(surface_detection_buffer_right), state);
      ctx->render_context->make_resident(std::get<1>(surface_detection_buffer_right), state);
      ctx->render_context->make_resident(std::get<2>(surface_detection_buffer_right), state);

      initialized = true;
    }

    void init_grid_resources(RenderContext ctx, /* math::vec2ui const& resolution */float pixel_count) {
      // auto pixel_count(resolution.x * resolution.y / 4);

      grid_vbo = std::vector<scm::gl::buffer_ptr>(2);

      grid_vao = std::vector<scm::gl::vertex_array_ptr>(2);

      grid_tfb = std::vector<scm::gl::transform_feedback_ptr>(2);
      for (int i = 0; i < 2; ++i) {
        grid_vbo[i] = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_DRAW,
                                           pixel_count * sizeof(math::vec3ui));

        grid_vao[i] = ctx.render_device->create_vertex_array(
            scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3UI, sizeof(math::vec3ui)), {grid_vbo[i]});

        grid_tfb[i] = ctx.render_device->create_transform_feedback(
            scm::gl::stream_output_setup(grid_vbo[i]));
      }
      cell_count = pixel_count;

      grid_initialized = true;
    }

    void init_fbo(gua::RenderContext* ctx) {
      framebuffer_resolved = ctx->render_device->create_frame_buffer();
      // framebuffer_resolved->attach_color_buffer_left(0, std::get<2>(color_buffer_left));
      // framebuffer_resolved->attach_depth_stencil_buffer(std::get<2>(depth_buffer_left));
      initialized_fbo = true;
    }

    void postprocess_frame(RenderContext* ctx) {
      // ctx->render_context->resolve_multi_sample_buffer(framebuffer, framebuffer_resolved);
      // ctx->render_context->generate_mipmaps(std::get<2>(color_buffer_left));
      framebuffer_resolved->attach_color_buffer(0, std::get<2>(color_buffer_left));
      framebuffer_resolved->attach_depth_stencil_buffer(std::get<2>(depth_buffer_left));
      //std::cout << "[POST PROCESS] color buffer second adress: " << std::get<2>(color_buffer_left)->native_handle() << std::endl;
      ctx->render_context->copy_color_buffer(framebuffer, framebuffer_resolved, 0);
      ctx->render_context->copy_depth_stencil_buffer(framebuffer, framebuffer_resolved);
      ctx->render_context->reset();
      framebuffer_resolved->clear_attachments();
      updated = true;
    }

    void swap_buffers_fast() {
      if(updated){
        std::lock_guard<std::mutex> lock(copy_mutex);
        std::swap(std::get<0>(color_buffer_left), std::get<1>(color_buffer_left));
        std::swap(std::get<0>(depth_buffer_left), std::get<1>(depth_buffer_left));
        std::swap(std::get<0>(is_left), std::get<1>(is_left));
        updated = false;
      }
    }
    void swap_buffers_slow() {
      std::lock_guard<std::mutex> lock(copy_mutex);
      std::swap(std::get<1>(color_buffer_left), std::get<2>(color_buffer_left));
      std::swap(std::get<1>(depth_buffer_left), std::get<2>(depth_buffer_left));
      std::swap(std::get<1>(is_left), std::get<2>(is_left));
      updated = true;
    }

    void swap_surface_buffer_fast() {
      if (grid_generated) {
        // std::cout << "[FAST] swapping shared resources" << std::endl;
        std::lock_guard<std::mutex> lock(copy_mutex);
        std::swap(std::get<0>(surface_detection_buffer_left), std::get<1>(surface_detection_buffer_left));
        grid_generated = false;
      }
    }
    void swap_surface_buffer_slow() {
      std::lock_guard<std::mutex> lock(copy_mutex);
      std::swap(std::get<1>(surface_detection_buffer_left), std::get<2>(surface_detection_buffer_left));
      grid_generated = true;
    }

    inline int current_tfb() {
      return ping ? 1 : 0;
    }

    inline int current_vbo() {
      return ping ? 0 : 1;
    }

    void is_there_time_left() {
      if (time_left >= 1.0) {
        return;
      } else {
        while (time_left < 1.0) {
          Sleep(0.1);
        }
      }
    }

    std::tuple<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> surface_detection_buffer_left;
    std::tuple<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> surface_detection_buffer_right;

    std::vector<scm::gl::buffer_ptr> grid_vbo;
    std::vector<scm::gl::transform_feedback_ptr> grid_tfb;
    std::vector<scm::gl::vertex_array_ptr> grid_vao;

    size_t cell_count = 0;
    bool ping = false;

    bool grid_initialized = false;
    bool grid_generated = false;
    CameraMode camera_mode;

    float time_budget;
    float time_left;
    
    std::shared_ptr<node::SerializedCameraNode> serialized_warp_cam;

    std::tuple<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> color_buffer_left;
    std::tuple<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> depth_buffer_left;
    std::tuple<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> color_buffer_right;
    std::tuple<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> depth_buffer_right;

    scm::gl::frame_buffer_ptr framebuffer;
    scm::gl::frame_buffer_ptr framebuffer_resolved;

    scm::gl::sampler_state_desc sampler_state_desc;

    scm::gl::sampler_state_ptr sampler_state;

    std::tuple<bool,bool,bool> is_left = std::make_tuple<bool,bool,bool>(false, false, false);

    bool renderer_ready = false;
    bool updated = false;
    bool initialized = false;
    bool initialized_fbo = false;
    bool shared_initialized = false;
    GLFWwindow* shared;

    mutable std::mutex copy_mutex;
  }; // struct WarpingResources


  using SceneGraphs = std::vector<std::unique_ptr<const SceneGraph> >;

  /**
   * Constructor.
   *
   * This constructs a new Renderer.
   *
   * \param pipelines        A vector of Pipelines to process. For each
   *                         pipeline a RenderClient is created.
   */
  Renderer();
  Renderer(Renderer const&) = delete;
  Renderer& operator=(Renderer const&) = delete;

  /**
  *
  */
  ~Renderer();

  /**
   * Request a redraw of all RenderClients.
   *
   * Takes a Scenegraph and asks all clients to draw it.
   *
   * \param scene_graphs      The SceneGraphs to be processed.
   */
  void queue_draw(std::vector<SceneGraph const*> const& scene_graphs, bool enable_warping = false, int const desired_framerate = 100);

  void draw_single_threaded(std::vector<SceneGraph const*> const& scene_graphs);

  void stop();

  inline float get_application_fps() {
    return application_fps_.fps;
  }


 private:
  void send_renderclient(std::string const& window,
                         std::shared_ptr<const Renderer::SceneGraphs> sgs,
                         node::CameraNode* cam,
                         bool enable_warping,
                         int const desired_framerate);

  struct Item {
    Item() = default;
    Item( std::shared_ptr<node::SerializedCameraNode> const& sc,
          std::shared_ptr<const SceneGraphs> const& sgs,
          bool warp = false)
          : serialized_cam(sc), scene_graphs(sgs), enable_warping(warp)
    {}

    std::shared_ptr<node::SerializedCameraNode> serialized_cam;
    std::shared_ptr<const SceneGraphs>          scene_graphs;
    bool                                        enable_warping;
  };


  using Mailbox = std::shared_ptr<gua::concurrent::Doublebuffer<Item> >;
  using Renderclient = std::pair<Mailbox, std::thread>;
  using Warpclient = std::pair<std::string, std::thread>;

  std::map<std::string, std::shared_ptr<WarpingResources>> warp_resources;

  // standard renderclient
  static void renderclient(Mailbox in, std::string name);
  // renderclients used for warped rendering
  static void renderclient_slow(Mailbox in, std::string name, std::map<std::string, std::shared_ptr<WarpingResources>>&);
  static void renderclient_fast(Mailbox in, std::string name, std::map<std::string, std::shared_ptr<WarpingResources>>&);


  std::map<std::string, Renderclient> render_clients_;
  std::map<std::string, std::pair<Renderclient, Warpclient>> warp_clients_;

  FpsCounter application_fps_;
};

}

#endif  // GUA_RENDERER_HPP
