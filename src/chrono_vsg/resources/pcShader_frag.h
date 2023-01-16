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

#ifndef PC_SHADER_FRAG_H
#define PC_SHADER_FRAG_H

#include <vsg/io/VSG.h>

static auto pcShader_frag = []() {std::istringstream str(
R"(#vsga 0.5.1
Root id=1 vsg::ShaderStage
{
  userObjects 0
  stage 16
  entryPointName "main"
  module id=2 vsg::ShaderModule
  {
    userObjects 0
    hints id=0
    source "#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform sampler2D texSampler;

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;

layout(location = 0) out vec4 outColor;

void main() {
    outColor = texture(texSampler, fragTexCoord);
}
"
    code 173
     119734787 65536 524298 23 0 131089 1 393227 1 1280527431 1685353262 808793134
     0 196622 0 1 524303 4 4 1852399981 0 9 17 22
     196624 4 7 196611 2 450 589828 1096764487 1935622738 1918988389 1600484449 1684105331
     1868526181 1667590754 29556 262149 4 1852399981 0 327685 9 1131705711 1919904879 0
     327685 13 1400399220 1819307361 29285 393221 17 1734439526 1131963732 1685221231 0 327685
     22 1734439526 1869377347 114 262215 9 30 0 262215 13 34 0
     262215 13 33 0 262215 17 30 1 262215 22 30 0
     131091 2 196641 3 2 196630 6 32 262167 7 6 4
     262176 8 3 7 262203 8 9 3 589849 10 6 1
     0 0 0 1 0 196635 11 10 262176 12 0 11
     262203 12 13 0 262167 15 6 2 262176 16 1 15
     262203 16 17 1 262167 20 6 3 262176 21 1 20
     262203 21 22 1 327734 2 4 0 3 131320 5 262205
     11 14 13 262205 15 18 17 327767 7 19 14 18
     196670 9 19 65789 65592
  }
  NumSpecializationConstants 0
}
)");
vsg::VSG io;
return io.read_cast<vsg::ShaderStage>(str);
};

#endif
