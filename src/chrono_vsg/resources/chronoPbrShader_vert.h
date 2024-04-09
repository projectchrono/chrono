#include <vsg/io/VSG.h>
#include <vsg/io/mem_stream.h>
static auto chronoPbrShader_vert = []() {
    static const char str[] =
        R"(#vsga 1.1.0
Root id=1 vsg::ShaderStage
{
  userObjects 0
  mask 18446744073709551615
  stage 1
  entryPointName "main"
  module id=2 vsg::ShaderModule
  {
    userObjects 0
    hints id=0
    source "#version 450
#extension GL_ARB_separate_shader_objects : enable

#pragma import_defines (VSG_INSTANCE_POSITIONS, VSG_BILLBOARD, VSG_DISPLACEMENT_MAP)

#define VIEW_DESCRIPTOR_SET 0
#define MATERIAL_DESCRIPTOR_SET 1
#define CUSTOM_DESCRIPTOR_SET 2

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

#ifdef VSG_DISPLACEMENT_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 6) uniform sampler2D displacementMap;
#endif

layout(location = 0) in vec3 vsg_Vertex;
layout(location = 1) in vec3 vsg_Normal;
layout(location = 2) in vec2 vsg_TexCoord0;
layout(location = 3) in vec4 vsg_Color;

layout(set = CUSTOM_DESCRIPTOR_SET, binding = 0) uniform TexScale
{
    vec2 fkt;
} texScale;

#ifdef VSG_BILLBOARD
layout(location = 4) in vec4 vsg_position_scaleDistance;
#elif defined(VSG_INSTANCE_POSITIONS)
layout(location = 4) in vec3 vsg_position;
#endif

layout(location = 0) out vec3 eyePos;
layout(location = 1) out vec3 normalDir;
layout(location = 2) out vec4 vertexColor;
layout(location = 3) out vec2 texCoord0;

layout(location = 5) out vec3 viewDir;

out gl_PerVertex{ vec4 gl_Position; };

#ifdef VSG_BILLBOARD
mat4 computeBillboadMatrix(vec4 center_eye, float autoScaleDistance)
{
    float distance = -center_eye.z;

    float scale = (distance < autoScaleDistance) ? distance/autoScaleDistance : 1.0;
    mat4 S = mat4(scale, 0.0, 0.0, 0.0,
                  0.0, scale, 0.0, 0.0,
                  0.0, 0.0, scale, 0.0,
                  0.0, 0.0, 0.0, 1.0);

    mat4 T = mat4(1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  center_eye.x, center_eye.y, center_eye.z, 1.0);
    return T*S;
}
#endif

void main()
{
    vec4 vertex = vec4(vsg_Vertex, 1.0);
    vec4 normal = vec4(vsg_Normal, 0.0);

#ifdef VSG_DISPLACEMENT_MAP
    // TODO need to pass as as uniform or per instance attributes
    vec3 scale = vec3(1.0, 1.0, 1.0);

    vertex.xyz = vertex.xyz + vsg_Normal * (texture(displacementMap, vsg_TexCoord0.st).s * scale.z);

    float s_delta = 0.01;
    float width = 0.0;

    float s_left = max(vsg_TexCoord0.s - s_delta, 0.0);
    float s_right = min(vsg_TexCoord0.s + s_delta, 1.0);
    float t_center = vsg_TexCoord0.t;
    float delta_left_right = (s_right - s_left) * scale.x;
    float dz_left_right = (texture(displacementMap, vec2(s_right, t_center)).s - texture(displacementMap, vec2(s_left, t_center)).s) * scale.z;

    // TODO need to handle different origins of displacementMap vs diffuseMap etc,
    float t_delta = s_delta;
    float t_bottom = max(vsg_TexCoord0.t - t_delta, 0.0);
    float t_top = min(vsg_TexCoord0.t + t_delta, 1.0);
    float s_center = vsg_TexCoord0.s;
    float delta_bottom_top = (t_top - t_bottom) * scale.y;
    float dz_bottom_top = (texture(displacementMap, vec2(s_center, t_top)).s - texture(displacementMap, vec2(s_center, t_bottom)).s) * scale.z;

    vec3 dx = normalize(vec3(delta_left_right, 0.0, dz_left_right));
    vec3 dy = normalize(vec3(0.0, delta_bottom_top, -dz_bottom_top));
    vec3 dz = normalize(cross(dx, dy));

    normal.xyz = normalize(dx * vsg_Normal.x + dy * vsg_Normal.y + dz * vsg_Normal.z);
#endif

#ifdef VSG_INSTANCE_POSITIONS
    vertex.xyz = vertex.xyz + vsg_position;
#endif

#ifdef VSG_BILLBOARD
    mat4 mv = computeBillboadMatrix(pc.modelView * vec4(vsg_position_scaleDistance.xyz, 1.0), vsg_position_scaleDistance.w);
#else
    mat4 mv = pc.modelView;
#endif

    gl_Position = (pc.projection * mv) * vertex;
    eyePos = (mv * vertex).xyz;
    viewDir = - (mv * vertex).xyz;
    normalDir = (mv * normal).xyz;

    vertexColor = vsg_Color;
    //texCoord0 = vsg_TexCoord0;
    texCoord0.s = vsg_TexCoord0.s * texScale.fkt.s;
    texCoord0.t = vsg_TexCoord0.t * texScale.fkt.t;
}
"
    code 690
     119734787 65536 524299 97 0 131089 1 393227 1 1280527431 1685353262 808793134
     0 196622 0 1 983055 0 4 1852399981 0 12 20 40
     51 56 62 67 69 73 75 196611 2 450 589828 1096764487
     1935622738 1918988389 1600484449 1684105331 1868526181 1667590754 29556 262149 4 1852399981 0 262149
     9 1953654134 30821 327685 12 1600615286 1953654102 30821 262149 19 1836216174 27745
     327685 20 1600615286 1836216142 27745 196613 29 30317 393221 30 1752397136 1936617283
     1953390964 115 393222 30 0 1785688688 1769235301 28271 393222 30 1 1701080941
     1701402220 119 196613 32 25456 393221 38 1348430951 1700164197 2019914866 0 393222
     38 0 1348430951 1953067887 7237481 196613 40 0 262149 51 1348827493 29551
     262149 56 2003134838 7498052 327685 62 1836216174 1766091873 114 327685 67 1953654134
     1866692709 7499628 327685 69 1600615286 1869377347 114 327685 73 1131963764 1685221231 48
     393221 75 1600615286 1131963732 1685221231 48 327685 81 1400399188 1701601635 0 262150
     81 0 7629670 327685 83 1400399220 1701601635 0 262215 12 30 0
     262215 20 30 1 262216 30 0 5 327752 30 0 35
     0 327752 30 0 7 16 262216 30 1 5 327752 30
     1 35 64 327752 30 1 7 16 196679 30 2 327752
     38 0 11 0 196679 38 2 262215 51 30 0 262215
     56 30 5 262215 62 30 1 262215 67 30 2 262215
     69 30 3 262215 73 30 3 262215 75 30 2 327752
     81 0 35 0 196679 81 2 262215 83 34 2 262215
     83 33 0 131091 2 196641 3 2 196630 6 32 262167
     7 6 4 262176 8 7 7 262167 10 6 3 262176
     11 1 10 262203 11 12 1 262187 6 14 1065353216 262203
     11 20 1 262187 6 22 0 262168 27 7 4 262176
     28 7 27 262174 30 27 27 262176 31 9 30 262203
     31 32 9 262165 33 32 1 262187 33 34 1 262176
     35 9 27 196638 38 7 262176 39 3 38 262203 39
     40 3 262187 33 41 0 262176 48 3 7 262176 50
     3 10 262203 50 51 3 262203 50 56 3 262203 50
     62 3 262203 48 67 3 262176 68 1 7 262203 68
     69 1 262167 71 6 2 262176 72 3 71 262203 72
     73 3 262176 74 1 71 262203 74 75 1 262165 76
     32 0 262187 76 77 0 262176 78 1 6 196638 81
     71 262176 82 2 81 262203 82 83 2 262176 84 2
     6 262176 88 3 6 262187 76 90 1 327734 2 4
     0 3 131320 5 262203 8 9 7 262203 8 19 7
     262203 28 29 7 262205 10 13 12 327761 6 15 13
     0 327761 6 16 13 1 327761 6 17 13 2 458832
     7 18 15 16 17 14 196670 9 18 262205 10 21
     20 327761 6 23 21 0 327761 6 24 21 1 327761
     6 25 21 2 458832 7 26 23 24 25 22 196670
     19 26 327745 35 36 32 34 262205 27 37 36 196670
     29 37 327745 35 42 32 41 262205 27 43 42 262205
     27 44 29 327826 27 45 43 44 262205 7 46 9
     327825 7 47 45 46 327745 48 49 40 41 196670 49
     47 262205 27 52 29 262205 7 53 9 327825 7 54
     52 53 524367 10 55 54 54 0 1 2 196670 51
     55 262205 27 57 29 262205 7 58 9 327825 7 59
     57 58 524367 10 60 59 59 0 1 2 262271 10
     61 60 196670 56 61 262205 27 63 29 262205 7 64
     19 327825 7 65 63 64 524367 10 66 65 65 0
     1 2 196670 62 66 262205 7 70 69 196670 67 70
     327745 78 79 75 77 262205 6 80 79 393281 84 85
     83 41 77 262205 6 86 85 327813 6 87 80 86
     327745 88 89 73 77 196670 89 87 327745 78 91 75
     90 262205 6 92 91 393281 84 93 83 41 90 262205
     6 94 93 327813 6 95 92 94 327745 88 96 73
     90 196670 96 95 65789 65592
  }
  NumSpecializationConstants 0
}
)";
    vsg::VSG io;
    return io.read_cast<vsg::ShaderStage>(reinterpret_cast<const uint8_t*>(str), sizeof(str));
};
