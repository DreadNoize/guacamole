#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <gua/utils/Timer.hpp>

#include <scm/core/math.h>

class Navigator {

  public:

    Navigator();

    void update();

    void set_transform(scm::math::mat4f const& transform);
    void reset();
    scm::math::mat4f const& get_transform() const;

    void set_key_press(int key, int action);
    void set_mouse_button(int button, int state);

    void set_mouse_position(scm::math::vec2i const& new_position);

  private:

    scm::math::mat4f transform_;

    scm::math::vec4f current_location_;
    scm::math::vec2f current_rotation_;

    scm::math::vec2f mouse_position_;
    scm::math::vec2f mouse_movement_;

    bool w_pressed_;
    bool s_pressed_;
    bool a_pressed_;
    bool d_pressed_;
    bool mlb_pressed_;

    gua::Timer timer_;
    double frame_time_;
};

#endif  // NAVIGATOR_HPP
