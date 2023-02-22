// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Rainer Gericke, code taken from https://github.com/vsg-dev/vsgExamples.git
// =============================================================================

#ifndef CUBEMAP_SHADERS_H
#define CUBEMAP_SHADERS_H

const auto skybox_vert = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

layout(location = 0) in vec3 osg_Vertex;
layout(location = 0) out vec3 outUVW;

out gl_PerVertex{ vec4 gl_Position; };

void main()
{
    outUVW = osg_Vertex;

    // Remove translation
    mat4 modelView = pc.modelView;
    modelView[3] = vec4(0.0, 0.0, 0.0, 1.0);

    gl_Position = pc.projection * modelView * vec4(osg_Vertex, 1.0);
}
)";

const auto skybox_frag = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform samplerCube envMap;
layout(location = 0) in vec3 inUVW;
layout(location = 0) out vec4 outColor;

void main()
{
    outColor = textureLod(envMap, inUVW, 0);
}
)";

#endif
