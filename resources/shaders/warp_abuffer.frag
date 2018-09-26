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

@include "common/header.glsl"
@include "common/gua_camera_uniforms.glsl"
@include "abuffer_warp_modes.glsl"
@include "hole_filling_modes.glsl"

layout(pixel_center_integer) in vec4 gl_FragCoord;

// output
layout(location=0) out vec3 gua_out_color;

#define MAX_RAY_STEPS @max_raysteps@

uniform float gua_tone_mapping_exposure = 1.5;

@include "common/gua_abuffer.glsl"

in vec2 gua_quad_coords;

uniform bool perform_warp;
uniform mat4 inv_warp_matrix;
uniform mat4 warp_matrix;
uniform uvec2 warped_depth_buffer;
uniform uvec2 warped_color_buffer;
uniform uvec2 orig_pbr_buffer;
uniform uvec2 hole_filling_texture;


vec2 get_epipolar_direction() {
  vec4 epipol = warp_matrix * vec4(0, 0, -1, 0);
  vec2 epi_dir = vec2(0);

  if (epipol.w == 0) {
    epipol.xy = epipol.xy*0.5 + 0.5;
    epi_dir = epipol.xy - gua_quad_coords;
  } else if (epipol.w < 0) {
    epipol /= epipol.w;
    epipol.xy = epipol.xy*0.5 + 0.5;
    epi_dir = epipol.xy - gua_quad_coords;
  } else {
    epipol /= epipol.w;
    epipol.xy = epipol.xy*0.5 + 0.5;
    epi_dir = gua_quad_coords - epipol.xy;
  }

  return normalize(epi_dir);
}

vec4 hole_filling_epipolar_search() {
  vec2 boundary_pos = vec2(0);

  vec2 epi_dir = get_epipolar_direction();
  float sample_depth = 1.0;

  for (int i=1; i<=75; ++i) {
    boundary_pos = gua_quad_coords + i*epi_dir/gua_resolution;
    sample_depth = texelFetch(sampler2D(warped_depth_buffer), ivec2(boundary_pos*gua_resolution), 0).x;

    if (sample_depth < 1.0) break;
  }

  for (int i=1; i<=75; ++i) {
    vec2 pos = gua_quad_coords - i*epi_dir/gua_resolution;
    float depth = texelFetch(sampler2D(warped_depth_buffer), ivec2(pos*gua_resolution), 0).x;

    if (depth < 1.0) {
      if (depth > sample_depth || sample_depth == 1.0) {
        boundary_pos = pos;
        sample_depth = depth;
      }
      break;
    }
  }

  if (sample_depth == 1.0) {
    return vec4(@hole_filling_color@, 1);
  }

  return texelFetch(sampler2D(warped_color_buffer), ivec2(boundary_pos*gua_resolution),0);
}

vec4 hole_filling_epipolar_mirror() {
  vec2 boundary_pos = vec2(0);

  vec2 epi_dir = get_epipolar_direction();
  float sample_depth = 1.0;

  for (int i=1; i<=75; ++i) {
    boundary_pos = gua_quad_coords + i*epi_dir/gua_resolution;
    sample_depth = texelFetch(sampler2D(warped_depth_buffer), ivec2(boundary_pos*gua_resolution), 0).x;

    if (sample_depth < 1.0) {
      sample_depth = texelFetch(sampler2D(warped_depth_buffer), ivec2((2*boundary_pos - gua_quad_coords)*gua_resolution), 0).x;
      if (sample_depth < 1.0) break;
    }
  }

  if (sample_depth == 1.0) {
    return vec4(@hole_filling_color@, 1);
  }

  return texelFetch(sampler2D(warped_color_buffer), ivec2((2*boundary_pos - gua_quad_coords)*gua_resolution), 0);
}

vec4 hole_filling_blur() {
  const float step_size = 0.5;
  const float max_level = 7;
  const vec2  epi_dir = get_epipolar_direction();
  const vec2  dirs[2] = {
    vec2( epi_dir.x,  epi_dir.y),
    vec2(-epi_dir.x, -epi_dir.y)
  };

  float depth = 0.0;
  float level = max_level;

  for (int i=0; i<dirs.length(); ++i) {
    for (float l=0; l<=max_level; l+=step_size) {
      vec2  p = gua_quad_coords - pow(2,l)*dirs[i]/gua_resolution;
      float d = texelFetch(sampler2D(warped_depth_buffer), ivec2(p*gua_resolution), 0).x;

      if (d < 1.0) {
        if (d > depth+0.0001 || (abs(d-depth)<0.0001 && l<level)) {
          level = l;
          depth = d;
        }
        break;
      }
    }
  }

  if (depth == 0) {
    return vec4(@hole_filling_color@, 1);
  }

  return vec4(textureLod(sampler2D(hole_filling_texture), gua_quad_coords, level+1).rgb, 0);
}

void main() {

  vec4 color = vec4(0);
  vec4 opaque_color_emit = vec4(0);
  float emissivity = 0;

  if (!perform_warp) {
      gua_out_color = texture2D(sampler2D(warped_color_buffer), gua_quad_coords).rgb;
      // gua_out_color = vec3(1.0,0.0,0.0);
    return;
  }

  // hole filling
  float depth = texture2D(sampler2D(warped_depth_buffer), gua_quad_coords).x;
  #if HOLE_FILLING_MODE == HOLE_FILLING_MODE_EPIPOLAR_SEARCH
    // if (depth == 1.0) opaque_color_emit = vec4(0.0,0.1,0.3, 1);
    if (depth == 1.0) opaque_color_emit = hole_filling_epipolar_search();
    else              opaque_color_emit = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
    // opaque_color_emit = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
    // opaque_color_emit = vec4(2.0 * depth - 1.0, 2.0 * depth - 1.0, 2.0 * depth - 1.0, 1);
  #elif HOLE_FILLING_MODE == HOLE_FILLING_MODE_EPIPOLAR_MIRROR
    if (depth == 1.0) opaque_color_emit = hole_filling_epipolar_mirror();
    else              opaque_color_emit = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
  #elif HOLE_FILLING_MODE == HOLE_FILLING_MODE_BLUR
    if (depth == 1.0) opaque_color_emit = hole_filling_blur();
    else              opaque_color_emit = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
  #else
    // if (depth == 1.0) opaque_color_emit = vec4(depth,depth,depth, 1);
    if (depth == 1.0) opaque_color_emit = vec4(1.0,0.1,0.3, 1);
    // if (depth == 1.0) opaque_color_emit = vec4(@hole_filling_color@, 1);
    // else              opaque_color_emit = vec4(0.3,0.0,0.1,1);
    else              opaque_color_emit = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
    // else              opaque_color_emit = vec4(0.3,0.0,0.1,1);
    // else              opaque_color_emit = vec4(2.0 * depth - 1.0, 2.0 * depth - 1.0, 2.0 * depth - 1.0, 1);
  #endif
    // opaque_color_emit = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
    // if (depth == 1.0) opaque_color_emit = vec4(0.5, 0.3, 0.1, 1.0);
    // else opaque_color_emit = vec4(2.0 * depth - 1.0, 2.0 * depth - 1.0, 2.0 * depth - 1.0, 1);
    gua_out_color = opaque_color_emit.rgb;
}
