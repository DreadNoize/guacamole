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
layout(triangle_strip, max_vertices = 16) out;

uniform uvec2 gua_gbuffer_depth;
uniform mat4 warp_matrix;
// uniform uvec2 resolution;

flat in uvec3 varying_position[];

flat out uint cellsize;
out vec2 cellcoords;
out vec2 texcoords;
out float pass_depth;
flat out vec2 pos;

#define GUA_MAGIC_DEPTH_OFFSET  0.99999 // clamping offset, to prevent depth stencil call

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

float get_depth(vec2 position) {
  return gua_get_depth(position/gua_resolution);
}

void emit_grid_vertex(vec2 position, float depth) {
  pass_depth = depth;
  gl_Position = warp_matrix * vec4(2.0 * (position / gua_resolution) - 1.0, depth, 1.0);
  // gl_Position = vec4(position, depth, 1.0);
  EmitVertex();
}

#define GAP @pixel_size@

@include "shaders/warp_grid_bits.glsl"

void emit_quad(uvec2 offset, uvec2 size) {

  if (size.x > 0 && size.y > 0) {

    float depth1, depth2, depth3, depth4;

    vec2 pos1 = vec2(varying_position[0].xy)                        + vec2(offset);
    vec2 pos2 = vec2(varying_position[0].xy) + vec2(size.x, 0)      + vec2(offset);
    vec2 pos3 = vec2(varying_position[0].xy) + vec2(0,      size.y) + vec2(offset);
    vec2 pos4 = vec2(varying_position[0].xy) + vec2(size.x, size.y) + vec2(offset);

    const int cont_l = int(varying_position[0].z >> BIT_CONTINUOUS_L) & 1;
    const int cont_r = int(varying_position[0].z >> BIT_CONTINUOUS_R) & 1;
    const int cont_t = int(varying_position[0].z >> BIT_CONTINUOUS_T) & 1;
    const int cont_b = int(varying_position[0].z >> BIT_CONTINUOUS_B) & 1;

    const int cont_tl = int(varying_position[0].z >> BIT_CONTINUOUS_TL) & 1;
    const int cont_tr = int(varying_position[0].z >> BIT_CONTINUOUS_TR) & 1;
    const int cont_bl = int(varying_position[0].z >> BIT_CONTINUOUS_BL) & 1;
    const int cont_br = int(varying_position[0].z >> BIT_CONTINUOUS_BR) & 1;

    depth1 = get_depth(vec2(-cont_l, -cont_b)*cont_bl*0.5 + pos1+vec2( 0.5,  0.5));
    depth2 = get_depth(vec2( cont_r, -cont_b)*cont_br*0.5 + pos2+vec2(-0.5,  0.5));
    depth3 = get_depth(vec2(-cont_l,  cont_t)*cont_tl*0.5 + pos3+vec2( 0.5, -0.5));
    depth4 = get_depth(vec2( cont_r,  cont_t)*cont_tr*0.5 + pos4+vec2(-0.5, -0.5));

    depth1 = clamp(depth1, 0.0, GUA_MAGIC_DEPTH_OFFSET);
    depth2 = clamp(depth2, 0.0, GUA_MAGIC_DEPTH_OFFSET);
    depth3 = clamp(depth3, 0.0, GUA_MAGIC_DEPTH_OFFSET);
    depth4 = clamp(depth4, 0.0, GUA_MAGIC_DEPTH_OFFSET);    

    // #endif // ------------------------------------------------------------------

    cellsize = min(size.x, size.y);

    texcoords = (pos1 / (gua_resolution));
    cellcoords = vec2(0, 0);
    pos = pos1;
    emit_grid_vertex(pos1 + vec2(-GAP, -GAP), depth1);

    texcoords = (pos2 / (gua_resolution));
    cellcoords = vec2(1, 0);
    pos = pos2;
    emit_grid_vertex(pos2 + vec2( GAP, -GAP), depth2);

    texcoords = (pos3 / (gua_resolution));
    cellcoords = vec2(0, 1);
    pos = pos3;
    emit_grid_vertex(pos3 + vec2(-GAP,  GAP), depth3);

    texcoords = (pos4 / (gua_resolution));
    cellcoords = vec2(1, 1);
    pos = pos4;
    emit_grid_vertex(pos4 + vec2( GAP,  GAP), depth4);

    EndPrimitive();
  }
}

void emit_pixel(uvec2 offset) {

  cellsize = 1;
  vec2 position = varying_position[0].xy + offset;
  pos = position;
  // remove strange one-pixel line
  if (position.y == gua_resolution.y) return;

  texcoords = (position / (gua_resolution));
  float depth = get_depth_raw(position);

  depth = clamp(depth, 0.0, GUA_MAGIC_DEPTH_OFFSET);

  cellcoords = vec2(0, 0);
  emit_grid_vertex(position + vec2(0, 0) + vec2(-GAP, -GAP), depth);
  cellcoords = vec2(1, 0);
  emit_grid_vertex(position + vec2(1, 0) + vec2( GAP, -GAP), depth);
  cellcoords = vec2(0, 1);
  emit_grid_vertex(position + vec2(0, 1) + vec2(-GAP,  GAP), depth);
  cellcoords = vec2(1, 1);
  emit_grid_vertex(position + vec2(1, 1) + vec2( GAP,  GAP), depth);

  EndPrimitive();
}

void main() {

  uint  level = varying_position[0].z >> BIT_CURRENT_LEVEL;
  uvec2 scale = 1 + uvec2((varying_position[0].z >> BIT_EXPAND_X) & 1, (varying_position[0].z >> BIT_EXPAND_Y) & 1);

  cellsize = 1 << level;
  if ((varying_position[0].z & 1 ) > 0) {
    emit_quad(uvec2(0), uvec2(1 << level));
    // emit_quad(uvec2(0), max(vec2(1), cellsize * scale - vec2(0)));
  } else {
    emit_pixel(uvec2(0, 0));
    emit_pixel(uvec2(1, 0));
    emit_pixel(uvec2(1, 1));
    emit_pixel(uvec2(0, 1));
    // const uvec2 offsets[4] = {uvec2(0), uvec2(1, 0),
    //                           uvec2(1), uvec2(0, 1)};
    // for (int v=0; v<4; ++v) {
    //   emit_quad(offsets[v]+uvec2(1), max(uvec2(1), scale - uvec2(2)));
    // }
  }
}
