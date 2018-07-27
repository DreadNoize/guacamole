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

@include "shaders/common/header.glsl"
@include "gbuffer_warp_modes.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/warp_grid_bits.glsl"

layout(points) in;
layout(points, max_vertices = 4) out;

in uvec3 varying_position[];

uniform uvec2 surface_detection_buffer;
uniform int current_level;

out uvec3 xfb_output;

void emit(uvec2 offset, uint data) {
  xfb_output = uvec3(varying_position[0].xy+offset, data);
  if (xfb_output.x < gua_resolution.x && xfb_output.y < gua_resolution.y) {
    EmitVertex(); EndPrimitive();
  }
}

void main() {

  if (current_level == (varying_position[0].z >> BIT_CURRENT_LEVEL)) {
    const uint new_level = current_level - 1;
    const uint size = 1 << new_level;
    uint bit_data = 0;

    uint s0 = 0;
    uint s1 = 0;
    uint s2 = 0;
    uint s3 = 0;


    bit_data = texelFetch(usampler2D(surface_detection_buffer), ivec2(varying_position[0].xy/(1 << current_level)), int(new_level)).x;

    if (current_level == 2) {

      // we are at the last level. Output 2x2 quads and set a flag whether it
      // should be splitted by the warp pass. 

      // s0-s1
      // |   |
      // s2-s3

      s0 = texelFetch(usampler2D(surface_detection_buffer), ivec2(varying_position[0].xy/(1 << new_level)) + ivec2(0, 1), int(new_level-1)).x;
      s1 = texelFetch(usampler2D(surface_detection_buffer), ivec2(varying_position[0].xy/(1 << new_level)) + ivec2(1, 1), int(new_level-1)).x;
      s2 = texelFetch(usampler2D(surface_detection_buffer), ivec2(varying_position[0].xy/(1 << new_level)) + ivec2(0, 0), int(new_level-1)).x;
      s3 = texelFetch(usampler2D(surface_detection_buffer), ivec2(varying_position[0].xy/(1 << new_level)) + ivec2(1, 0), int(new_level-1)).x;
    }

    if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_NONE) {

      // the current cell covers more than one surface --- we need to split it!
      emit(uvec2(0,    size), (new_level << BIT_CURRENT_LEVEL) | s0);
      emit(uvec2(size, size), (new_level << BIT_CURRENT_LEVEL) | s1);
      emit(uvec2(0,    0),    (new_level << BIT_CURRENT_LEVEL) | s2);
      emit(uvec2(size, 0),    (new_level << BIT_CURRENT_LEVEL) | s3);


    } else {
      
      // the current cell covers only one continuous surface --- write it's size, continuity and that's one surface
      emit(uvec2(0), (current_level << BIT_CURRENT_LEVEL) 
                   | (varying_position[0].z & (1<<BIT_EXPAND_X)) 
                   | (varying_position[0].z & (1<<BIT_EXPAND_Y)) 
                   | (ALL_CONTINUITY_BITS & bit_data) | 1);
    }

  } else {
    // the current cell has not been split in the previous pass --- we dont need to touch it
    xfb_output = varying_position[0];
    EmitVertex(); EndPrimitive();
  }
}
