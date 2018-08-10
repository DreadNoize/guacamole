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
        color_buffer = rhs.color_buffer;
        depth_buffer = rhs.depth_buffer;
        sampler_state_desc = rhs.sampler_state_desc;
        sampler_state = rhs.sampler_state;
        is_left = rhs.is_left;
        renderer_ready = rhs.renderer_ready;
        updated = rhs.updated;
        initialized = rhs.initialized;
        initialized_fbo = rhs.initialized_fbo;
        shared_initialized = rhs.shared_initialized;
        rctx = rhs.rctx;
      }
      return *this;
    }

    void init(gua::RenderContext* ctx, gua::math::vec2ui const& resolution) {
      // synch = "unsynched";
      // is_left = std::make_pair<bool,bool>(false,false);

      sampler_state_desc = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE);
      // std::cout << "Initializing Warping Sampler State ..." << std::endl; 
      sampler_state = ctx->render_device->create_sampler_state(sampler_state_desc);
      
      // std::cout << "Initializing Warping Texture Color ..." << std::endl; 

      color_buffer.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer.first, sampler_state);
      color_buffer.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer.second, sampler_state);

      // std::cout << "Initializing Warping Texture Depth ..." << std::endl; 

      depth_buffer.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer.first, sampler_state);
      depth_buffer.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer.second, sampler_state);

      // auto pixel_count(resolution.x * resolution.y / 4);

      // grid_vbo_warp.first = std::vector<scm::gl::buffer_ptr>(2);
      // grid_vbo_warp.second = std::vector<scm::gl::buffer_ptr>(2);

      // grid_tfb_warp.first = std::vector<scm::gl::transform_feedback_ptr>(2);
      // grid_tfb_warp.second = std::vector<scm::gl::transform_feedback_ptr>(2);

      // copy_buffer.first = ctx->render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
      //                                      scm::gl::USAGE_DYNAMIC_DRAW,
      //                                      pixel_count * sizeof(math::vec3ui));
      // copy_buffer.second = ctx->render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
      //                                      scm::gl::USAGE_DYNAMIC_DRAW,
      //                                      pixel_count * sizeof(math::vec3ui));

      // for(int i = 0; i < 2; ++i) {
      //   grid_vbo_warp.first[i] = ctx->render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
      //                                      scm::gl::USAGE_DYNAMIC_DRAW,
      //                                      pixel_count * sizeof(math::vec3ui));
      //   grid_vbo_warp.second[i] = ctx->render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
      //                                      scm::gl::USAGE_DYNAMIC_DRAW,
      //                                      pixel_count * sizeof(math::vec3ui));

      //   grid_tfb_warp.first[i] = ctx->render_device->create_transform_feedback(
      //       scm::gl::stream_output_setup(grid_vbo_warp.first[i]));
      //   grid_tfb_warp.second[i] = ctx->render_device->create_transform_feedback(
      //       scm::gl::stream_output_setup(grid_vbo_warp.second[i]));
      // }

      math::vec2 size(resolution / 2);

      int current_level(std::log2(32));
      int mip_map_levels(current_level);

      scm::gl::sampler_state_desc state_desc(scm::gl::FILTER_MIN_MAG_NEAREST,
        scm::gl::WRAP_CLAMP_TO_EDGE,
        scm::gl::WRAP_CLAMP_TO_EDGE);
      scm::gl::sampler_state_ptr state = ctx->render_device->create_sampler_state(state_desc);

      surface_detection_buffer.first = ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
      ctx->render_context->make_resident(surface_detection_buffer.first, state);
      surface_detection_buffer.second = ctx->render_device->create_texture_2d(math::vec2ui(size.x, size.y), scm::gl::FORMAT_R_16UI, mip_map_levels);
      ctx->render_context->make_resident(surface_detection_buffer.second, state);

      initialized = true;
    }

    void init_grid_resources(RenderContext ctx, /* math::vec2ui const& resolution */float pixel_count) {
      // auto pixel_count(resolution.x * resolution.y / 4);

      grid_vbo.first = std::vector<scm::gl::buffer_ptr>(2);
      grid_vbo.second = std::vector<scm::gl::buffer_ptr>(2);

      grid_vao.first = std::vector<scm::gl::vertex_array_ptr>(2);
      grid_vao.second = std::vector<scm::gl::vertex_array_ptr>(2);

      grid_tfb.first = std::vector<scm::gl::transform_feedback_ptr>(2);
      grid_tfb.second = std::vector<scm::gl::transform_feedback_ptr>(2);
      for (int i = 0; i < 2; ++i) {
        grid_vbo.first[i] = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_DRAW,
                                           pixel_count * sizeof(math::vec3ui));
        grid_vbo.second[i] = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_DRAW,
                                           pixel_count * sizeof(math::vec3ui));

        grid_vao.first[i] = ctx.render_device->create_vertex_array(
            scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3UI, sizeof(math::vec3ui)), {grid_vbo.first[i]});
        grid_vao.second[i] = ctx.render_device->create_vertex_array(
            scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3UI, sizeof(math::vec3ui)), {grid_vbo.second[i]});

        grid_tfb.first[i] = ctx.render_device->create_transform_feedback(
            scm::gl::stream_output_setup(grid_vbo.first[i]));
        grid_tfb.second[i] = ctx.render_device->create_transform_feedback(
            scm::gl::stream_output_setup(grid_vbo.second[i]));
      }
      cell_count = pixel_count;

      grid_initialized = true;
      
      /*surface_detection_buffer.first = std::make_shared<Texture2D>();
      surface_detection_buffer.second = std::make_shared<Texture2D>();*/
    }

    void init_fbo(gua::RenderContext* ctx) {
      framebuffer_resolved = ctx->render_device->create_frame_buffer();
      // framebuffer_resolved->attach_color_buffer(0, color_buffer.second);
      // framebuffer_resolved->attach_depth_stencil_buffer(depth_buffer.second);
      initialized_fbo = true;
    }

    void postprocess_frame(RenderContext* ctx) {
      // ctx->render_context->resolve_multi_sample_buffer(framebuffer, framebuffer_resolved);
      // ctx->render_context->generate_mipmaps(color_buffer.second);
      framebuffer_resolved->attach_color_buffer(0, color_buffer.second);
      framebuffer_resolved->attach_depth_stencil_buffer(depth_buffer.second);
      //std::cout << "[POST PROCESS] color buffer second adress: " << color_buffer.second->native_handle() << std::endl;
      ctx->render_context->copy_color_buffer(framebuffer, framebuffer_resolved, 0);
      ctx->render_context->copy_depth_stencil_buffer(framebuffer, framebuffer_resolved);
      ctx->render_context->reset();
      framebuffer_resolved->clear_attachments();
      updated = true;
    }

    void swap_buffers() {
      if(updated){
        // std::cout << "swapping buffers ..." << std::endl; 
        // std::cout << "[BEFORE SWAP] color buffer second adress: " << color_buffer.second->native_handle() << std::endl;
        std::lock_guard<std::mutex> lock(copy_mutex);
        std::swap(color_buffer.first, color_buffer.second);
        std::swap(depth_buffer.first, depth_buffer.second);
        std::swap(is_left.first, is_left.second);
        // std::cout << "[AFTER SWAP] color buffer second adress: " << color_buffer.second->native_handle() << std::endl;
        updated = false;
      }
    }

    inline int current_tfb() {
      return ping ? 1 : 0;
    }

    inline int current_vbo() {
      return ping ? 0 : 1;
    }

    void swap_shared_resources() {
      if (grid_generated) {
        // std::cout << "[FAST] swapping shared resources" << std::endl;
        std::lock_guard<std::mutex> lock(copy_mutex);
        std::swap(surface_detection_buffer.first, surface_detection_buffer.second);
        // std::swap(grid_vbo.first, grid_vbo.second);
        // std::swap(grid_tfb.first, grid_tfb.second);
        // std::swap(grid_vao.first, grid_vao.second);
        // std::swap(grid_vbo_warp.first, grid_vbo_warp.second);
        // std::swap(grid_tfb_warp.first, grid_tfb_warp.second);
      }
    }

    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> surface_detection_buffer;

    std::pair<std::vector<scm::gl::buffer_ptr>, std::vector<scm::gl::buffer_ptr>> grid_vbo;
    std::pair<std::vector<scm::gl::transform_feedback_ptr>, std::vector<scm::gl::transform_feedback_ptr>> grid_tfb;
    // std::pair<std::vector<scm::gl::buffer_ptr>, std::vector<scm::gl::buffer_ptr>> grid_vbo_warp;
    // std::pair<std::vector<scm::gl::transform_feedback_ptr>, std::vector<scm::gl::transform_feedback_ptr>> grid_tfb_warp;
    std::pair<std::vector<scm::gl::vertex_array_ptr>, std::vector<scm::gl::vertex_array_ptr>> grid_vao;
    scm::gl::vertex_array_ptr warp_vao[2];

    // std::pair<scm::gl::buffer_ptr, scm::gl::buffer_ptr> copy_buffer;

    size_t cell_count = 0;
    bool ping = false;

    bool grid_initialized = false;
    bool grid_generated = false;
    CameraMode camera_mode;
    bool debug_grid;
    
    std::shared_ptr<node::SerializedCameraNode> serialized_warp_cam;

    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> color_buffer;
    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> depth_buffer;

    scm::gl::frame_buffer_ptr framebuffer;
    scm::gl::frame_buffer_ptr framebuffer_resolved;

    scm::gl::sampler_state_desc sampler_state_desc;

    scm::gl::sampler_state_ptr sampler_state;

    std::pair<bool,bool> is_left = std::make_pair<bool,bool>(false, false);
    bool renderer_ready = false;
    bool updated = false;
    bool initialized = false;
    bool initialized_fbo = false;
    bool shared_initialized = false;

    mutable gua::RenderContext rctx;

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
  void queue_draw(std::vector<SceneGraph const*> const& scene_graphs, bool enable_warping = false);

  void draw_single_threaded(std::vector<SceneGraph const*> const& scene_graphs);

  void stop();

  inline float get_application_fps() {
    return application_fps_.fps;
  }


 private:

  void send_renderclient(std::string const& window,
                         std::shared_ptr<const Renderer::SceneGraphs> sgs,
                         node::CameraNode* cam,
                         bool enable_warping);

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
