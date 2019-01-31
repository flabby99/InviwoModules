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
uniform mat4 transformMatrix;
uniform mat4 inverseMatrix;

uniform sampler2D tex0Color;
uniform sampler2D tex0Depth;

out vec4 colour;
out float not_valid;

// TODO if this is slow this way, could do it in a fragment shader instead - can't control point size though
void main(void) {
    // Multiply the transform Matrix by the incoming vertex and go from there.
    float depth = texture(tex0Depth, in_position).r;

    // depth comes in 0, 1 convert it to -1 1
    depth = 2 * depth - 1;
    vec4 screen_pos = vec4(2 * in_position - 1, 0.8, 1);
    vec4 world_pos = inverseMatrix * screen_pos;
    //vec4 result = world_pos;
    world_pos = world_pos / world_pos.w;
    vec4 result = transformMatrix * world_pos;
    colour = texture(tex0Color, in_position);
    //colour = result;
    //colour.a = 1.0 - pow(1.0 - colour.a, 0.007 * 150);
    //result.z = 0;
    //result.z = 1;
    //result.z = clamp(result.z, 0, 1);
    //result.w = 1;
    // TODO calculate this based on distances and normals
    gl_PointSize = 1.0;

    // Division by w is done in hardware
    gl_Position = result;
    not_valid = float(result.w < 0);
}
