#include <vsg/io/VSG.h>
#include <vsg/io/mem_stream.h>
static auto chronoLineShader_frag = []() {
static const char str[] = 
R"(#vsga 1.0.8
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
    code 0
    
  }
  NumSpecializationConstants 0
}
)";
vsg::VSG io;
return io.read_cast<vsg::ShaderStage>(reinterpret_cast<const uint8_t*>(str), sizeof(str));
};
