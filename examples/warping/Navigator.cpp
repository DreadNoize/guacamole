#include "Navigator.hpp"

#include <scm/gl_core/math.h>
#include <gua/math/math.hpp>
#include <chrono>
#include <iostream>

Navigator::Navigator()
  : transform_(scm::math::mat4f::identity())
  , current_location_(scm::math::vec4f(0.0))
  , current_rotation_(scm::math::vec2f(0.0))
  , mouse_position_(scm::math::vec2i(0))
  , mouse_movement_(scm::math::vec2i(0))
  , w_pressed_(false)
  , s_pressed_(false)
  , a_pressed_(false)
  , d_pressed_(false)
  , mlb_pressed_(false)
  , frame_time_(-1.0)
{}

void Navigator::update() {
  if (frame_time_ == -1.0) {
    timer_.start();
    frame_time_ = 0.0;
  } else {

    frame_time_ = timer_.get_elapsed();

    const float rotation_speed = 0.6f;
    const float motion_speed = 1.f;

    auto y_rot(scm::math::mat4f::identity());
    auto x_rot(scm::math::mat4f::identity());

    if (mlb_pressed_) {
      current_rotation_ -= mouse_movement_*rotation_speed;
    }

    y_rot = scm::math::make_rotation( current_rotation_.x, 0.f, 1.f, 0.f);
    x_rot = scm::math::make_rotation(-current_rotation_.y, 1.f, 0.f, 0.f);

    auto rotation = y_rot * x_rot;

    if (w_pressed_) {
      current_location_ += (rotation * scm::math::make_translation(0.f, 0.f, (float)(-motion_speed*frame_time_)))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    if (s_pressed_) {
      current_location_ += (rotation * scm::math::make_translation(0.f, 0.f, (float)(motion_speed*frame_time_)))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    if (a_pressed_) {
      current_location_ += (rotation * scm::math::make_translation((float)(-motion_speed*frame_time_), 0.f, 0.f))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    if (d_pressed_) {
      current_location_ += (rotation * scm::math::make_translation((float)(motion_speed*frame_time_), 0.f, 0.f))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    auto target = scm::math::make_translation(current_location_.x, current_location_.y, current_location_.z) * rotation;
    float smoothness = 0.2;
    transform_ = transform_ * (1.f - smoothness) + target * smoothness;
    mouse_movement_ = scm::math::vec2i(0);

    timer_.reset();
  }
}

void Navigator::set_transform(scm::math::mat4f const& transform) {
  transform_ = transform;
  current_location_ = scm::math::vec4f(transform[3], transform[7], transform[11], 1.0f);

  auto quat = gua::math::quat::from_matrix(gua::math::mat4(transform));
  gua::math::quat::value_type angle;
  gua::math::vec3 axis;
  quat.retrieve_axis_angle(angle, axis);

  current_rotation_ = gua::math::vec2(-axis.y*angle, axis.x*angle);
}

void Navigator::reset() {
  transform_ = scm::math::mat4f::identity();
  current_location_ = scm::math::vec4f(0.f, 0.f, 0.f, 1.0f);
  current_rotation_ = scm::math::vec2f(0.0);
}

scm::math::mat4f const& Navigator::get_transform() const {
  return transform_;
}

void Navigator::set_key_press(int key, int action) {
  switch (key) {
    case 87:
      w_pressed_ = action != 0;
      break;

    case 83:
      s_pressed_ = action != 0;
      break;

    case 65:
      a_pressed_ = action != 0;
      break;

    case 68:
      d_pressed_ = action != 0;
      break;
  }
}

void Navigator::set_mouse_button(int button, int state) {
  switch (button) {
    case 0:
      mlb_pressed_ = state == 1;
      break;
  }
}

void Navigator::set_mouse_position(scm::math::vec2i const& new_position) {
  mouse_movement_ = new_position - mouse_position_;
  mouse_position_ = new_position;
}

