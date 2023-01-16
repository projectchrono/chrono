#include <vsg/io/VSG.h>

static auto lineShader_frag = []() {std::istringstream str(
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

layout(location = 0) in vec3 fragColor;

layout(location = 0) out vec4 outColor;

void main() {
    //outColor = texture(texSampler, fragTexCoord);
    outColor = vec4(fragColor,1);
}
"
    code 134
     119734787 65536 524298 19 0 131089 1 393227 1 1280527431 1685353262 808793134
     0 196622 0 1 458767 4 4 1852399981 0 9 12 196624
     4 7 196611 2 450 589828 1096764487 1935622738 1918988389 1600484449 1684105331 1868526181
     1667590754 29556 262149 4 1852399981 0 327685 9 1131705711 1919904879 0 327685
     12 1734439526 1869377347 114 262215 9 30 0 262215 12 30 0
     131091 2 196641 3 2 196630 6 32 262167 7 6 4
     262176 8 3 7 262203 8 9 3 262167 10 6 3
     262176 11 1 10 262203 11 12 1 262187 6 14 1065353216
     327734 2 4 0 3 131320 5 262205 10 13 12 327761
     6 15 13 0 327761 6 16 13 1 327761 6 17
     13 2 458832 7 18 15 16 17 14 196670 9 18
     65789 65592
  }
  NumSpecializationConstants 0
}
)");
vsg::VSG io;
return io.read_cast<vsg::ShaderStage>(str);
};
