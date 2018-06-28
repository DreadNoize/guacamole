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

  // Container struct for resources (like textures) needed for the asynchronous warping technique
  // using DBTexture = std::shared_ptr<gua::concurrent::Doublebuffer<scm::gl::texture_2d_ptr>>;
  struct WarpingResources {
    WarpingResources() = default;
    /* WarpingResources( gua::RenderContext const& ctx, gua::math::vec2ui const& resolution)
        : sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT),
          is_left(false,false),
          renderer_ready(false),
          updated(false),
          synch("unsynched") {
            // scm::gl::sampler_state_desc sampler_state_desc();
            std::cout << "Initializing Warping Sampler State ..." << std::endl; 
            sampler_state = ctx.render_device->create_sampler_state(sampler_state_desc);
            
            std::cout << "Initializing Warping Texture Color ..." << std::endl; 

            color_buffer.first = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
            ctx.render_context->make_resident(color_buffer.first, sampler_state);
            color_buffer.second = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
            ctx.render_context->make_resident(color_buffer.second, sampler_state);

            std::cout << "Initializing Warping Texture Depth ..." << std::endl; 

            depth_buffer.first = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
            ctx.render_context->make_resident(depth_buffer.first, sampler_state);
            depth_buffer.second = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
            ctx.render_context->make_resident(depth_buffer.second, sampler_state);
    } */

    WarpingResources& operator=(WarpingResources const& rhs) {
      if (this != &rhs) { 
        std::lock(copy_mutex, rhs.copy_mutex);
        std::lock_guard<std::mutex> m_lhs(copy_mutex, std::adopt_lock);
        std::lock_guard<std::mutex> m_rhs(rhs.copy_mutex, std::adopt_lock);
        color_buffer = rhs.color_buffer;
        color_buffer_fast = rhs.color_buffer_fast;
        color_buffer_slow = rhs.color_buffer_slow;
        depth_buffer = rhs.depth_buffer_slow;
        depth_buffer_fast = rhs.depth_buffer_fast;
        depth_buffer_slow = rhs.depth_buffer_slow;
        pbo_color = rhs.pbo_color;
        sampler_state_desc = rhs.sampler_state_desc;
        sampler_state_desc_fast = rhs.sampler_state_desc_fast;
        sampler_state_desc_slow = rhs.sampler_state_desc_slow;
        sampler_state = rhs.sampler_state;
        sampler_state_fast = rhs.sampler_state_fast;
        sampler_state_slow = rhs.sampler_state_slow;
        is_left = rhs.is_left;
        renderer_ready = rhs.renderer_ready;
        updated = rhs.updated;
        shared_initialized = rhs.shared_initialized;
        rctx = rhs.rctx;
        pixel_data = rhs.pixel_data;
        synch = rhs.synch;
      }
      return *this;
    }

    void init(gua::RenderContext* ctx, gua::math::vec2ui const& resolution) {
      // synch = "unsynched";
      // is_left = std::make_pair<bool,bool>(false,false);
      renderer_ready = false;
      updated = false;

      sampler_state_desc = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);
      std::cout << "Initializing Warping Sampler State ..." << std::endl; 
      sampler_state = ctx->render_device->create_sampler_state(sampler_state_desc);
      
      std::cout << "Initializing Warping Texture Color ..." << std::endl; 

      color_buffer.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer.first, sampler_state);
      color_buffer.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer.second, sampler_state);

      std::cout << "Initializing Warping Texture Depth ..." << std::endl; 

      depth_buffer.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer.first, sampler_state);
      depth_buffer.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer.second, sampler_state);

      initialized = true;
    }

    void init_fast(gua::RenderContext* ctx, gua::math::vec2ui const& resolution) {
      // synch = "unsynched";
      // is_left = std::make_pair<bool,bool>(false,false);
      renderer_ready = false;
      updated = false;

      sampler_state_desc_fast = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);
      std::cout << "Initializing Warping Sampler State ..." << std::endl; 
      sampler_state_fast = ctx->render_device->create_sampler_state(sampler_state_desc_fast);
      
      std::cout << "Initializing Warping Texture Color ..." << std::endl; 

      color_buffer_fast.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer_fast.first, sampler_state_fast);
      color_buffer_fast.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer_fast.second, sampler_state_fast);

      std::cout << "Initializing Warping Texture Depth ..." << std::endl; 

      depth_buffer_fast.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer_fast.first, sampler_state_fast);
      depth_buffer_fast.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer_fast.second, sampler_state_fast);

      initialized = true;
    }

    void init_slow(gua::RenderContext* ctx, gua::math::vec2ui const& resolution) {
      synch = "unsynched";
      is_left = std::make_pair<bool,bool>(false,false);
      renderer_ready = false;
      updated = false;

      sampler_state_desc_slow = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);
      std::cout << "Initializing Rendering Sampler State ..." << std::endl; 
      sampler_state_slow = ctx->render_device->create_sampler_state(sampler_state_desc_slow);
      
      std::cout << "Initializing Rendering Texture Color ..." << std::endl; 

      color_buffer_slow.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer_slow.first, sampler_state_slow);
      color_buffer_slow.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer_slow.second, sampler_state_slow);

      std::cout << "Initializing Rendering Texture Depth ..." << std::endl; 

      depth_buffer_slow.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer_slow.first, sampler_state_slow);
      depth_buffer_slow.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer_slow.second, sampler_state_slow);

      initialized = true;
    }
    
    void init_pbo(gua::RenderContext* ctx, gua::math::vec2ui const& resolution) {
      std::cout << "Creating PBO ..." << std::endl;
      pbo_color.first = ctx->render_device->create_buffer(scm::gl::BIND_PIXEL_PACK_BUFFER, scm::gl::USAGE_STREAM_DRAW, sizeof(float) * resolution.x * resolution.y * 3);
      pbo_color.second = ctx->render_device->create_buffer(scm::gl::BIND_PIXEL_PACK_BUFFER, scm::gl::USAGE_STREAM_DRAW, sizeof(float) * resolution.x * resolution.y * 3);
    }

    void update_pbo(RenderContext* ctx, scm::gl::texture_2d_ptr in_texture) {
      std::cout << "Updating PBO ..." << std::endl;
      const scm::gl::opengl::gl_core& glapi = ctx->render_context->opengl_api();
      scm::gl::util::gl_error glerror(glapi);
      
      assert(pbo_color.second->object_id() != 0);
      assert(pbo_color.second->state().ok());


      //glapi.glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_color.second->object_id());

      std::cout << "[update_pbo] glerror bind buffer: " << glerror.error_string() << std::endl;

      //glapi.glBindTexture(in_texture->object_target(), in_texture->object_id());

      //std::cout << "[update_pbo] glerror bind texture: " << glerror.error_string() << std::endl;

      // ctx->render_context->retrieve_texture_data(in_texture, in_texture->mip_map_layers(), 0);
      glapi.glGetTexImage(in_texture->object_target(), in_texture->mip_map_layers(), scm::gl::util::gl_base_format(in_texture->format()), scm::gl::util::gl_base_type(in_texture->format()), (GLvoid*) 0);
      
      std::cout << "[update_pbo] glerror get texture image: " << glerror.error_string() << std::endl;

      fence = glapi.glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    }

    void update_texture(RenderContext* ctx, math::vec2ui const& resolution) {
      std::cout << "Updating Texture ..." << std::endl;
      const scm::gl::opengl::gl_core& glapi = ctx->render_context->opengl_api();
      scm::gl::util::gl_error glerror(glapi);
      
      assert(pbo_color.second->object_id() != 0);
      assert(pbo_color.second->state().ok());

      auto pixels = glapi.glMapBuffer(pbo_color.first->descriptor()._bindings, GL_READ_ONLY);

      std::cout << "[update_texture] glerror map pbo: " << glerror.error_string() << std::endl;

      scm::gl::texture_region region(scm::math::vec3ui(0.0, 0.0, 0.0), scm::math::vec3ui(resolution.x, resolution.y, 1.0));

      std::cout << "[update_texture] color buffer adress of second is " << color_buffer_slow.second << std::endl;
      std::cout << "[update_texture] color buffer object id is " << color_buffer_slow.second->object_id() << std::endl;
      std::cout << "[update_texture] color buffer object target is " << color_buffer_slow.second->object_target() << std::endl;
      std::cout << "[update_texture] color buffer object format is " << scm::gl::format_string(color_buffer_slow.second->descriptor()._format) << std::endl;
      ctx->render_context->update_sub_texture(color_buffer_slow.second, region, color_buffer_slow.second->mip_map_layers(),
                                              color_buffer_slow.second->format(),
                                              pixels);
    }

    bool fence_status(RenderContext* ctx) {
      const scm::gl::opengl::gl_core& glapi = ctx->render_context->opengl_api();
      scm::gl::util::gl_error glerror(glapi);

      int status = 0;
      
      glapi.glGetSynciv(fence, GL_SYNC_STATUS, 1, 0, &status);

      switch (status) {
        case GL_SIGNALED:   return true;
        case GL_UNSIGNALED: 
        default:            return false;
      }
    }

    void update(gua::RenderContext* ctx, const void* in_data, math::vec2ui const& resolution) {
      // std::cout << "creating region ..." << std::endl;
      // rctx.render_context->apply();
      scm::gl::texture_region region(scm::math::vec3ui(0.0, 0.0, 0.0),
                                     scm::math::vec3ui(resolution.x, resolution.y, 1.0));

      std::cout << "updating texture ..." << std::endl;
      ctx->render_context->update_sub_texture(color_buffer.second, region, 0,
                                              color_buffer.second->format(),
                                              in_data);
      // std::cout << "updating texture FINISHED" << std::endl;
      updated = true;
    }
    /* void update2(const void *in_data, math::vec2ui const& resolution) {
      // std::cout << "creating region ..." << std::endl;
      rctx.render_context->apply();
      scm::gl::texture_region region(scm::math::vec3ui(0.0, 0.0, 0.0),
                                     scm::math::vec3ui(resolution.x, resolution.y, 1.0));

      std::cout << "updating texture2 ..." << std::endl;
      rctx.render_context->update_sub_texture(color_buffer.first, region, 0,
                                              color_buffer.first->format(),
                                              in_data);
      // std::cout << "updating texture FINISHED" << std::endl;
      updated = true;
    } */

    void swap_buffers() {
      if(updated){
        std::cout << "swapping buffers ..." << std::endl; 
        std::lock_guard<std::mutex> lock(copy_mutex);
        std::swap(color_buffer.first, color_buffer.second);
        std::swap(depth_buffer.first, depth_buffer.second);
        std::swap(is_left.first, is_left.second);
        updated = false;
      }
    }

    void swap_buffers_fast() {
      if(updated){
        std::cout << "swapping buffers ..." << std::endl; 
        std::lock_guard<std::mutex> lock(copy_mutex_fast);
        std::swap(color_buffer_fast.first, color_buffer_fast.second);
        std::swap(depth_buffer_fast.first, depth_buffer_fast.second);
        std::swap(is_left.first, is_left.second);
        updated = false;
      }
    }

    void swap_buffers_slow() {
      if(updated){
        std::cout << "swapping buffers ..." << std::endl; 
        std::lock_guard<std::mutex> lock(copy_mutex_slow);
        std::swap(color_buffer_slow.first, color_buffer_slow.second);
        std::swap(depth_buffer_slow.first, depth_buffer_slow.second);
        std::swap(is_left.first, is_left.second);
        updated = false;
      }
    }

    void swap_pbo() {
      std::lock_guard<std::mutex> lock(copy_mutex_pbo);
      std::swap(pbo_color.first, pbo_color.second);
    }

    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> color_buffer;
    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> color_buffer_fast;
    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> color_buffer_slow;

    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> depth_buffer;
    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> depth_buffer_fast;
    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> depth_buffer_slow;

    std::pair<scm::gl::buffer_ptr, scm::gl::buffer_ptr> pbo_color;

    scm::gl::sampler_state_desc sampler_state_desc;
    scm::gl::sampler_state_desc sampler_state_desc_fast;
    scm::gl::sampler_state_desc sampler_state_desc_slow;

    scm::gl::sampler_state_ptr sampler_state;
    scm::gl::sampler_state_ptr sampler_state_fast;
    scm::gl::sampler_state_ptr sampler_state_slow;

    std::pair<bool,bool> is_left;
    bool renderer_ready;
    bool updated;
    bool initialized = false;
    bool shared_initialized = false;

    mutable gua::RenderContext rctx;

    GLFWwindow* shared;

    std::vector<float> pixel_data;

    std::string synch;
    GLsync fence;

    mutable std::mutex copy_mutex;
    mutable std::mutex copy_mutex_fast;
    mutable std::mutex copy_mutex_slow;
    mutable std::mutex copy_mutex_pbo;
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
