// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Rainer Gericke, code like pcShaders, but modified
// =============================================================================

#ifndef QUADMAP_SHADERS_H
#define QUADMAP_SHADERS_H

#pragma once

const auto skysphere_vert = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec2 inTexCoord;

layout(location = 0) out vec2 fragTexCoord;

out gl_PerVertex {
    vec4 gl_Position;
};

void main() {
    mat4 modelView = pc.modelview;
    modelView[3] = vec4(0.0, 0.0, 0.0, 1.0);
    gl_Position = (pc.projection * modelView) * vec4(inPosition, 1.0);
    fragTexCoord = inTexCoord;
}
)";

const auto skysphere_frag = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform sampler2D texSampler;

layout(location = 0) in vec2 fragTexCoord;

layout(location = 0) out vec4 outColor;

void main() {
    float sk = 0.9;
    vec4 col = texture(texSampler, fragTexCoord);
    outColor = vec4(sk*col.xyz,col.w);
}
)";

#endif
