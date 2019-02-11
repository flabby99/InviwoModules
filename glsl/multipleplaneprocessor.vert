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

uniform sampler2D tex0Color;
uniform sampler2D tex0Depth;

out vec4 colour;
out float not_valid;

// TODO if this is slow this way, could do it in a fragment shader instead - can't control point size though
void main(void) {
    colour = texture(tex0Color, in_position);
    // Don't splat transparent points
    if (colour.a == 0) {
        not_valid = 1;
        gl_Position = vec4(0, 0, 1, 1);
    }

    // Multiply the transform Matrix by the incoming vertex and go from there.
    float depth = texture(tex0Depth, in_position).r;
    // depth comes in 0, 1 convert it to -1 1
    depth = 2 * depth - 1;

    vec4 screen_pos = vec4(2 * in_position - 1, depth, 1);
    vec4 world_pos = transformMatrix * screen_pos;
    
    // TODO calculate this based on distances and normals
    gl_PointSize = 1.0;

    // Division by w is done in hardware
    gl_Position = world_pos;
    not_valid = float(world_pos.w < 0);
}
