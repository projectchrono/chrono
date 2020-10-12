rem Taken from vulkan scene graph examples
glslc shader_PushConstants.vert -o vert_PushConstants.spv
glslc shader_PushConstants.frag -o frag_PushConstants.spv
rem Phong shading based on the chrono_opengl Phong shader
rem Translated to glsl version 4.5
rem ambient color, diffuse color, specular color and shininess set by application
rem actually 10 material parameters are needed as input.
glslc shader_Phong.vert -o vert_Phong.spv
glslc shader_Phong.frag -o frag_Phong.spv
rem
glslc shader_SimplePhong.vert -o vert_SimplePhong.spv
glslc shader_SimplePhong.frag -o frag_SimplePhong.spv
rem
glslc shader_BlinnPhong.vert -o vert_BlinnPhong.spv
glslc shader_BlinnPhong.frag -o frag_BlinnPhong.spv
rem
glslc shader_Wireframe.vert -o vert_Wireframe.spv
glslc shader_Wireframe.frag -o frag_Wireframe.spv
rem
glslc shader_PBR.vert -o vert_PBR.spv
glslc shader_PBR.frag -o frag_PBR.spv
