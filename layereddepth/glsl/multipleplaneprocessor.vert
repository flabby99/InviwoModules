/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2018 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/
#include "utils/sampler2d.glsl"
layout (location = 0) in vec2 in_position;
uniform mat4 transformMatrix = mat4(1);
uniform int width = 819;

uniform sampler2D tex0Color;
uniform sampler2D tex0Depth;
uniform sampler2D spriteTex;

out vec4 colour;
out float not_valid;

void main(void) {
    colour = texture(tex0Color, in_position);
    // Don't splat transparent points
    if (colour.a < 0.05) {
       not_valid = 1;
       gl_Position = vec4(0, 0, -1, 1);
    }
    else {
        // Multiply the transform Matrix by the incoming vertex and go from there.
        float depth = texture(tex0Depth, in_position).r;
        // depth comes in 0, 1 convert it to -1 1
        depth = 2 * depth - 1;

        // depth = 0.99;
        vec4 screen_pos = vec4(2 * in_position - 1, depth, 1);
        vec4 new_screen_pos = transformMatrix * screen_pos;
        not_valid = float(new_screen_pos.w < 0);
        
        // TODO calculate this based on distances and normals - or at the very least, based on the view position.
        //vec3 displacement = vec3(new_screen_pos.xyz / new_screen_pos.w) - screen_pos.xyz;
        //float point_size = (1 + abs((displacement.x) * 100)) * (1 + abs((displacement.y) * 50));
        //point_size = 2;
        vec4 square_left = vec4(screen_pos.x - (1 / float(width)), screen_pos.yzw);
        vec4 square_right = vec4(screen_pos.x + (1 / float(width)), screen_pos.yzw);
        vec4 screen_vec = square_right - square_left;
        //vec4 new_screen_vec = transformMatrix * screen_vec;
        vec4 new_screen_vec = (transformMatrix * square_right) - (transformMatrix * square_left);
        float point_size = (width) * length(new_screen_vec.xyz);
        point_size = clamp(point_size, 1, 10);
        //colour = vec4(vec3(length(new_screen_vec.x) * (width / 2)), 1);
        //colour = vec4(vec3(length(screen_vec) * width / 3), 1);
        //colour = vec4(vec3(new_screen_vec.x) * width / 4, 1);
        //colour = vec4(vec3(abs(length(new_screen_vec) * (width / 2) - 1) * 100), 1);
        //colour = vec4(vec3(abs(length(new_screen_vec) - length(screen_vec)) * 1000), 1);

        gl_PointSize = 2;

        // Division by w is done in hardware
        gl_Position = new_screen_pos;
    }
}
