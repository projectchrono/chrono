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

#ifndef PC_SHADER_VERT_H
#define PC_SHADER_VERT_H

#include <vsg/io/VSG.h>

static auto pcShader_vert = []() {std::istringstream str(
R"(#vsga 0.5.1
Root id=1 vsg::ShaderStage
{
  userObjects 0
  stage 1
  entryPointName "main"
  module id=2 vsg::ShaderModule
  {
    userObjects 0
    hints id=0
    source "#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragTexCoord;

out gl_PerVertex {
    vec4 gl_Position;
};

void main() {
    gl_Position = (pc.projection * pc.modelview) * vec4(inPosition, 1.0);
    fragColor = inColor;
    fragTexCoord = inTexCoord;
}
"
    code 355
     119734787 65536 524298 46 0 131089 1 393227 1 1280527431 1685353262 808793134
     0 196622 0 1 720911 0 4 1852399981 0 10 26 37
     38 42 44 196611 2 450 589828 1096764487 1935622738 1918988389 1600484449 1684105331
     1868526181 1667590754 29556 262149 4 1852399981 0 393221 8 1348430951 1700164197 2019914866
     0 393222 8 0 1348430951 1953067887 7237481 196613 10 0 393221 14
     1752397136 1936617283 1953390964 115 393222 14 0 1785688688 1769235301 28271 393222 14
     1 1701080941 1701410412 119 196613 16 25456 327685 26 1867542121 1769236851 28271
     327685 37 1734439526 1869377347 114 262149 38 1866690153 7499628 393221 42 1734439526
     1131963732 1685221231 0 327685 44 1700032105 1869562744 25714 327752 8 0 11
     0 196679 8 2 262216 14 0 5 327752 14 0 35
     0 327752 14 0 7 16 262216 14 1 5 327752 14
     1 35 64 327752 14 1 7 16 196679 14 2 262215
     26 30 0 262215 37 30 0 262215 38 30 1 262215
     42 30 1 262215 44 30 2 131091 2 196641 3 2
     196630 6 32 262167 7 6 4 196638 8 7 262176 9
     3 8 262203 9 10 3 262165 11 32 1 262187 11
     12 0 262168 13 7 4 262174 14 13 13 262176 15
     9 14 262203 15 16 9 262176 17 9 13 262187 11
     20 1 262167 24 6 3 262176 25 1 24 262203 25
     26 1 262187 6 28 1065353216 262176 34 3 7 262176 36
     3 24 262203 36 37 3 262203 25 38 1 262167 40
     6 2 262176 41 3 40 262203 41 42 3 262176 43
     1 40 262203 43 44 1 327734 2 4 0 3 131320
     5 327745 17 18 16 12 262205 13 19 18 327745 17
     21 16 20 262205 13 22 21 327826 13 23 19 22
     262205 24 27 26 327761 6 29 27 0 327761 6 30
     27 1 327761 6 31 27 2 458832 7 32 29 30
     31 28 327825 7 33 23 32 327745 34 35 10 12
     196670 35 33 262205 24 39 38 196670 37 39 262205 40
     45 44 196670 42 45 65789 65592
  }
  NumSpecializationConstants 0
}
)");
vsg::VSG io;
return io.read_cast<vsg::ShaderStage>(str);
};

#endif


