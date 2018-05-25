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
#include <gua/utils/FpsCounter.hpp>
#include <gua/concurrent/Doublebuffer.hpp>

#include <scm/gl_core/texture_objects/texture_objects_fwd.h>


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

  using DBTexture = std::shared_ptr<gua::concurrent::Doublebuffer<scm::gl::texture_2d_ptr>>;
  struct CustomBuffer {
    CustomBuffer() = default;
    CustomBuffer(std::pair<DBTexture, DBTexture> const& color,
                 std::pair<DBTexture, DBTexture> const& depth)
        : color_buffer(color),
          depth_buffer(depth),
          is_left(false),
          renderer_ready(false),
          synch("unsynched") {}

    CustomBuffer& operator=(CustomBuffer const& rhs) {
      if (this != &rhs) { 
        std::lock(tex_mutex, rhs.tex_mutex);
        std::lock_guard<std::mutex> m_lhs(tex_mutex, std::adopt_lock);
        std::lock_guard<std::mutex> m_rhs(rhs.tex_mutex, std::adopt_lock);
        color_buffer = rhs.color_buffer;
        depth_buffer = rhs.depth_buffer;
        is_left = rhs.is_left;
        renderer_ready = rhs.renderer_ready;
        synch = rhs.synch;
      }
      return *this;
    }

    std::pair<DBTexture, DBTexture> color_buffer;
    std::pair<DBTexture, DBTexture> depth_buffer;

    bool is_left;
    bool renderer_ready;

    std::string synch;

    mutable std::mutex tex_mutex;
  };

  using Mailbox = std::shared_ptr<gua::concurrent::Doublebuffer<Item> >;
  using Renderclient = std::pair<Mailbox, std::thread>;
  using Warpclient = std::pair<std::string, std::thread>;

  std::map<std::string, std::shared_ptr<CustomBuffer>> warp_resources;

  static void renderclient(Mailbox in, std::string name);
  static void renderclient_warp(Mailbox in, std::string name, std::map<std::string, std::shared_ptr<CustomBuffer>>&);
  static void warpclient(Mailbox in, std::string name, std::map<std::string, std::shared_ptr<CustomBuffer>>&);


  std::map<std::string, Renderclient> render_clients_;
  std::map<std::string, std::pair<Renderclient, Warpclient>> warp_clients_;

  FpsCounter application_fps_;
};

}

#endif  // GUA_RENDERER_HPP
