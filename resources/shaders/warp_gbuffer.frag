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
@include "common/gua_gbuffer_input.glsl"
@include "gbuffer_warp_modes.glsl"
@include "hole_filling_modes.glsl"
@include "interpolation_modes.glsl"
@include "warp_grid_bits.glsl"

// -----------------------------------------------------------------------------
// #if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD || WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION
// -----------------------------------------------------------------------------

flat in uint cellsize;
in vec2 texcoords;
in vec2 cellcoords;

uniform uvec2 gua_warp_grid_tex;

// output
layout(location=0) out vec4 gua_out_color_emit;

vec3 heat(float v) {
  float value = 1.0-v;
  return (0.5+0.5*smoothstep(0.0, 0.1, value))*vec3(
    smoothstep(0.5, 0.3, value),
    value < 0.3 ? smoothstep(0.0, 0.3, value) : smoothstep(1.0, 0.6, value),
    smoothstep(0.4, 0.6, value)
  );
}

void main() {

  // #if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD || WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION
    // const bool is_surface = (texelFetch(usampler2D(gua_warp_grid_tex), ivec2(ivec2(texcoords*gua_resolution+vec2(0.001))/2), 0).x & 1) == 1;
  // #else
    uint info = texelFetch(usampler2D(gua_warp_grid_tex), ivec2(ivec2(texcoords*gua_resolution+vec2(0.001))/2), 0).x;
    const bool is_surface = (info & ALL_CONTINUITY_BITS) == ALL_CONTINUITY_BITS;
  // #endif

  #if INTERPOLATION_MODE == INTERPOLATION_MODE_NEAREST
    gua_out_color_emit.rgb = texelFetch(sampler2D(gua_gbuffer_color), ivec2(texcoords*gua_resolution+vec2(0.001)), 0).rgb;
    // gua_out_color_emit = vec4(1.0,0.8,0.3, 1.0);
    gua_out_color_emit.a = texelFetch(sampler2D(gua_gbuffer_pbr), ivec2(texcoords*gua_resolution+vec2(0.001)), 0).r;
  #elif INTERPOLATION_MODE == INTERPOLATION_MODE_LINEAR
    gua_out_color_emit.rgb = gua_get_color(texcoords);
    // gua_out_color_emit = vec4(1.0,0.8,0.3, 1.0);
    gua_out_color_emit.a = gua_get_pbr(texcoords).r;
  #else
    if (is_surface) {
      gua_out_color_emit.rgb = gua_get_color(texcoords);
      // gua_out_color_emit = vec4(1.0,0.8,0.3, 1.0);
      gua_out_color_emit.a = gua_get_pbr(texcoords).r;
    } else {
      gua_out_color_emit.rgb = texelFetch(sampler2D(gua_gbuffer_color), ivec2(texcoords*gua_resolution+vec2(0.001)), 0).rgb;
      // gua_out_color_emit = vec4(1.0,0.8,0.3, 1.0);
      gua_out_color_emit.a = texelFetch(sampler2D(gua_gbuffer_pbr), ivec2(texcoords*gua_resolution+vec2(0.001)), 0).r;
    }
  #endif

  /* #if @debug_cell_colors@ == 1
    float intensity = log2(cellsize) / 7.0;
    gua_out_color_emit.rgb = heat(1-intensity);

    if (any(lessThan(cellcoords, vec2(0.6/float(cellsize)))) || any(greaterThan(cellcoords, vec2(1.0-0.6/float(cellsize))))) {
      gua_out_color_emit.rgb = mix(gua_out_color_emit.rgb, vec3(0), 0.7);
    }
  #endif */
}
