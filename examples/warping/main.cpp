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
#include <Navigator.hpp>

bool manipulation_navigator = true;
bool manipulation_camera = false;
bool warping = true;

/* scenegraph overview for "main_scenegraph"
  /
  └ screen
    └ camera
*/
// forward mouse interaction to trackball
void mouse_button (gua::utils::Trackball& trackball, int mousebutton, int action, int mods) {
  gua::utils::Trackball::button_type button;
  gua::utils::Trackball::state_type state;

  switch (mousebutton) {
    case 0: button = gua::utils::Trackball::left; break;
    case 2: button = gua::utils::Trackball::middle; break;
    case 1: button = gua::utils::Trackball::right; break;
  };

  switch (action) {
    case 0: state = gua::utils::Trackball::released; break;
    case 1: state = gua::utils::Trackball::pressed; break;
  };

  trackball.mouse(button, state, trackball.posx(), trackball.posy());
}

int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // initialize movement and mouse interaction
  gua::utils::Trackball object_trackball(0.01, 0.002, 0.2);
  Navigator nav;
  Navigator warp_nav;
  nav.set_transform(scm::math::make_translation(0.f, 0.f, 3.f));

  // initialize scenegraph
  gua::SceneGraph graph("main_scenegraph");

  // initialize trimeshloader for model loading
  gua::TriMeshLoader loader;

  // create a transform node
  // which will be attached to the scenegraph
  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

  //auto material = ;
  // the model will be attached to the transform node
  auto geometry(loader.create_geometry_from_file(
      "geometry", "../data/objects/old-house-2.obj",  gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
  // geometry->translate(-0.6, 0.0, 0.0);
  //geometry->translate(0.0, -50.0,-50.0);
  //geometry->scale(0.005);
  auto sun_light = graph.add_node<gua::node::LightNode>("/", "sun_light");
  sun_light->data.set_type(gua::node::LightNode::Type::SUN);
  sun_light->data.set_color(gua::utils::Color3f(1.5f, 1.2f, 1.f));
  sun_light->data.set_shadow_cascaded_splits({ 0.1f, 1.5, 5.f, 10.f });
  sun_light->data.set_shadow_near_clipping_in_sun_direction(100.0f);
  sun_light->data.set_shadow_far_clipping_in_sun_direction(100.0f);
  sun_light->data.set_max_shadow_dist(30.0f);
  sun_light->data.set_shadow_offset(0.0004f);
  sun_light->data.set_enable_shadows(true);
  sun_light->data.set_shadow_map_size(512);
  sun_light->rotate(-65, 1, 0, 0);
  sun_light->rotate(-100, 0, 1, 0);
  
  auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");
  auto warp_navigation = graph.add_node<gua::node::TransformNode>("/navigation", "warp");

  graph.add_node("/transform", geometry);

  auto screen = graph.add_node<gua::node::ScreenNode>("/navigation", "screen");
  screen->data.set_size(gua::math::vec2(1.28f, 0.72f));  // real world size of screen
  screen->translate(0, 0, 1.0);

  // resolution to be used for camera resolution and windows size
  auto resolution = gua::math::vec2ui(600, 400);

  // set up camera and connect to screen in scenegraph
  auto camera = graph.add_node<gua::node::CameraNode>("/navigation", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/navigation/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  //camera->config.set_stereo_type(gua::StereoType::SPATIAL_WARP);
  camera->config.set_far_clip(350.f);
  camera->config.set_near_clip(0.1f);

  auto pipe_desc = camera->get_pipeline_description();
  pipe_desc->get_resolve_pass()->
    background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE).
    background_texture("../data/textures/sphericalskymap.jpg").
    environment_lighting_texture("../data/textures/sphericalskymap.jpg").
    background_color(gua::utils::Color3f(0,0,0)).
    environment_lighting(gua::utils::Color3f(0.4, 0.4, 0.5)).
    environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR).
    ssao_enable(true).
    tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::HEJL).
    tone_mapping_exposure(1.5f).
    horizon_fade(0.2f).
    ssao_intensity(1.5f).
    ssao_radius(2.f);

  // set up warp cam and warp screen
  auto warp_screen = graph.add_node<gua::node::ScreenNode>("/navigation/warp", "warp_screen");
  warp_screen->data.set_size(gua::math::vec2(1.28f, 0.72f));  // real world size of screen


  auto warp_cam = graph.add_node<gua::node::CameraNode>("/navigation/warp", "warp_cam");
  // warp_cam->translate(5,0,2);
  warp_cam->config.set_resolution(resolution);
  warp_cam->config.set_screen_path("/navigation/warp/warp_screen");
  warp_cam->config.set_scene_graph_name("main_scenegraph");
  warp_cam->config.set_far_clip(camera->config.get_far_clip() * 1.5);
  warp_cam->config.set_near_clip(camera->config.get_near_clip());

  // update view mode
  auto update_view_mode([&](){
    // set stereo options
    /* if (stereo) {

      normal_cam->config.set_enable_stereo(true);

      #if OCULUS1
        warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(-0.04f, 0.f, -0.05f)));
        warp_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.04f, 0.f, -0.05f)));
      #elif OCULUS2
        warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(-0.03175f, 0.f, -0.08f)));
        warp_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.03175f, 0.f, -0.08f)));
      #else
        window->config.set_stereo_mode(POWER_WALL || USE_SIDE_BY_SIDE ? gua::StereoMode::SIDE_BY_SIDE : gua::StereoMode::ANAGLYPH_RED_CYAN);
      #endif


      if (warping) {

        if (stereotype_spatial) {
          normal_cam->config.set_stereo_type(gua::StereoType::SPATIAL_WARP);

          #if OCULUS1
            normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
            normal_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
          #elif OCULUS2
            normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
            normal_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
          #endif

        } else if (stereotype_temporal) {
          normal_cam->config.set_stereo_type(gua::StereoType::TEMPORAL_WARP);
        } else if (stereotype_single_temporal) {
          normal_cam->config.set_stereo_type(gua::StereoType::SINGLE_TEMPORAL_WARP);
        }

      } else {
        normal_cam->config.set_stereo_type(gua::StereoType::RENDER_TWICE);
      } 

    } else { */

    camera->config.set_enable_stereo(false);

  /*   #if OCULUS1
      normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
      warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
    #elif OCULUS2
      normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
      warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
    #else
      window->config.set_stereo_mode(gua::StereoMode::MONO);
    #endif */

    if (warping) {
      camera->config.set_stereo_type(gua::StereoType::SPATIAL_WARP);
      camera->config.set_eye_offset(0.f);
      warp_cam->config.set_eye_offset(0.f);
    } else {
      camera->config.set_stereo_type(gua::StereoType::RENDER_TWICE);
      camera->config.set_eye_offset(0.f);
    }
    // }

  });

  update_view_mode();

  // set up window
  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_title("FAST CLIENT WINDOW");
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_stereo_mode(gua::StereoMode::MONO);
  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    warp_screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    
  });

  window->on_button_press.connect([&](int key, int action, int mods) {
    nav.set_mouse_button(key, action);
    warp_nav.set_mouse_button(key, action);

  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    if (manipulation_navigator) {
      nav.set_key_press(key, action);
    } else if (manipulation_camera) {
      warp_nav.set_key_press(key, action);
    }
    if (action == 1) {
      switch (key) {
        case 'C':
          std::swap(manipulation_navigator, manipulation_camera);
          break;
        case 'V':
          warping = !warping;
          update_view_mode();
          break;
      }
    }
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    if (manipulation_navigator) {
      nav.set_mouse_position(gua::math::vec2i(pos));
    } else {
      warp_nav.set_mouse_position(gua::math::vec2i(pos));
    }
  });

    window->on_button_press.connect(std::bind(mouse_button, std::ref(object_trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

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
    auto time = gua::Timer::get_now();
    // geometry->rotate(time*0.00000000002, gua::math::vec3(0.0,1.0,0.0));
    // log fps every 150th tick
    if (ctr++ % 150 == 0) {
      gua::Logger::LOG_WARNING
        << "Frame time: " << 1000.f / window->get_rendering_fps()
        << " ms, fps: " << window->get_rendering_fps() << std::endl;
    }

    nav.update();
    warp_nav.update();

    navigation->set_transform(gua::math::mat4(nav.get_transform()));
    warp_navigation->set_transform(gua::math::mat4(warp_nav.get_transform()));

    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(object_trackball.shiftx()),
                                                              gua::math::float_t(object_trackball.shifty()),
                                                              gua::math::float_t(object_trackball.distance())) * gua::math::mat4(object_trackball.rotation());
    transform->set_transform(modelmatrix);

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
