/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2018 Bauhaus-Universität Weimar                        *
 * Contact:   yorrick.paolo.sieler-morzuch@uni-weimar.de                      *
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

#include <functional>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Trackball.hpp>

/* scenegraph overview for "main_scenegraph"
  /
  └ screen
    └ camera
*/

int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // initialize scenegraph
  gua::SceneGraph graph("main_scenegraph");

  // initialize trimeshloader for model loading
  gua::TriMeshLoader loader;

  // create a transform node
  // which will be attached to the scenegraph
  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  // the model will be attached to the transform node
  auto sphere_geometry(loader.create_geometry_from_file(
      "sphere_geometry", "../data/objects/sphere.obj"));
  sphere_geometry->scale(0.5);

  graph.add_node("/transform", sphere_geometry);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.28f, 0.72f));  // real world size of screen
  screen->translate(0, 0, 1.0);

  // resolution to be used for camera resolution and windows size
  auto resolution = gua::math::vec2ui(1280, 800);

  // set up camera and connect to screen in scenegraph
  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");

  // set up window
  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_stereo_mode(gua::StereoMode::MONO);
  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(
        gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
  });
  // window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  
  // registers the function given in ticker.on_tick to
  // be called by the given loop in the given interval
  gua::events::Ticker ticker(loop, 1.0 / 500.0);

  // log fps and handle close events
  size_t ctr{};
  ticker.on_tick.connect([&]() {
    // log fps every 150th tick
    if (ctr++ % 150 == 0) {
      gua::Logger::LOG_WARNING
        << "Frame time: " << 1000.f / window->get_rendering_fps()
        << " ms, fps: " << window->get_rendering_fps() << std::endl;
    }
    window->process_events();
    if (window->should_close()) {
      // stop rendering and close the window
      renderer.stop();
      window->close();
      loop.stop();
    } else {
      // draw our scenegrapgh
      // std::cout << "MAIN: starting rendering..." << std::endl;
      renderer.queue_draw({&graph}, true);
    }
  });

  loop.start();

  return 0;
}
