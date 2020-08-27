# Taken from vulkan scene graph examples
glslc shader_PushConstants.vert -o vert_PushConstants.spv
glslc shader_PushConstants.frag -o frag_PushConstants.spv
# Phong shading based on the chrono_opengl Phong shader
# Translated to glsl version 4.5
# ambient color, diffuse color, specular color and shininess set by application
# actually 10 material parameters are needed as input.
glslc shader_Phong.vert -o vert_Phong.spv
glslc shader_Phong.frag -o frag_Phong.spv

