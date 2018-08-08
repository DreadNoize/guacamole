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
@include "shaders/common/gua_camera_uniforms.glsl"

layout(points) in;
layout(line_strip, max_vertices = 20) out;

uniform uvec2 gua_gbuffer_depth;
uniform mat4 warp_matrix;

flat in uvec3 varying_position[];
flat out uint cellsize;

float gua_get_depth(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth(sampler2D depth_texture, vec2 frag_pos) {
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}

float get_depth_raw(vec2 frag_pos) {
  return texelFetch(sampler2D(gua_gbuffer_depth), ivec2(frag_pos), 0).x*2-1;
}

float get_min_depth(vec2 frag_pos) {
  const float depth0 = get_depth_raw(frag_pos + vec2(-0.5, -0.5));
  const float depth1 = get_depth_raw(frag_pos + vec2( 0.5, -0.5));
  const float depth2 = get_depth_raw(frag_pos + vec2(-0.5,  0.5));
  const float depth3 = get_depth_raw(frag_pos + vec2( 0.5,  0.5));

  return min(depth0, min(depth1, min(depth2, depth3)));
}

@include "shaders/warp_grid_bits.glsl"

void emit_quad(uvec2 offset, vec2 size) {
  float depth = get_depth_raw(varying_position[0].xy);

  vec2 vertex_position = vec2(varying_position[0].xy + offset) / gua_resolution * 2 - 1;
  gl_Position = warp_matrix * vec4(vertex_position, depth, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset + vec2(size.x, 0)) / gua_resolution * 2 - 1;
  gl_Position = warp_matrix * vec4(vertex_position, depth, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset + vec2(size.x, size.y)) / gua_resolution * 2 - 1;
  gl_Position = warp_matrix * vec4(vertex_position, depth, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset + vec2(0, size.y)) / gua_resolution * 2 - 1;
  gl_Position = warp_matrix * vec4(vertex_position, depth, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset) / gua_resolution * 2 - 1;
  gl_Position = warp_matrix * vec4(vertex_position, depth, 1);
  EmitVertex();

  EndPrimitive();

}

void main() {

  uint  level = varying_position[0].z >> BIT_CURRENT_LEVEL;
  uvec2 scale = 1 + uvec2((varying_position[0].z >> BIT_EXPAND_X) & 1, (varying_position[0].z >> BIT_EXPAND_Y) & 1);

  cellsize = 1 << level;

  if ((varying_position[0].z & 1) > 0) {
    emit_quad(uvec2(0), max(vec2(1), cellsize * scale - vec2(0)));
  } else {
    const uvec2 offsets[4] = {uvec2(0), uvec2(1, 0),
                              uvec2(1), uvec2(0, 1)};
    for (int v=0; v<4; ++v) {
      emit_quad(offsets[v]+uvec2(1), max(vec2(1), scale - vec2(2)));
    }
  }
}
