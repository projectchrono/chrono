#version 450
#extension GL_ARB_separate_shader_objects : enable

#pragma import_defines (VSG_TEXTURECOORD_0, VSG_TEXTURECOORD_1, VSG_TEXTURECOORD_2, VSG_TEXTURECOORD_3, VSG_BILLBOARD, VSG_INSTANCE_TRANSLATION, VSG_INSTANCE_ROTATION, VSG_INSTANCE_SCALE, VSG_DISPLACEMENT_MAP, VSG_SKINNING, VSG_POINT_SPRITE)

#define VIEW_DESCRIPTOR_SET 0
#define MATERIAL_DESCRIPTOR_SET 1

#if defined(VSG_TEXTURECOORD_0)
    layout(location = 2) in vec2 vsg_TexCoord0;
#endif

#if defined(VSG_TEXTURECOORD_1)
    layout(location = 3) in vec2 vsg_TexCoord1;
#endif

#if defined(VSG_TEXTURECOORD_2)
    layout(location = 4) in vec2 vsg_TexCoord2;
#endif

#if defined(VSG_TEXTURECOORD_3)
    layout(location = 5) in vec2 vsg_TexCoord3;
#endif

#if defined(VSG_TEXTURECOORD_3)
    #define VSG_TEXCOORD_COUNT 4
#elif defined(VSG_TEXTURECOORD_2)
    #define VSG_TEXCOORD_COUNT 3
#elif defined(VSG_TEXTURECOORD_1)
    #define VSG_TEXCOORD_COUNT 2
#else
    #define VSG_TEXCOORD_COUNT 1
#endif

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

layout(location = 0) in vec3 vsg_Vertex;
layout(location = 1) in vec3 vsg_Normal;
layout(location = 6) in vec4 vsg_Color;

#ifdef VSG_DISPLACEMENT_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 7) uniform sampler2D displacementMap;
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 8) uniform DisplacementMapScale
{
    vec3 value;
} displacementMapScale;
#endif

#ifdef VSG_BILLBOARD
layout(location = 7) in vec4 vsg_Translation_scaleDistance;
#endif

#if defined(VSG_INSTANCE_TRANSLATION)
layout(location = 7) in vec3 vsg_Translation;
#endif

#if defined(VSG_INSTANCE_ROTATION)
layout(location = 8) in vec4 vsg_Rotation;
#endif

#if defined(VSG_INSTANCE_SCALE)
layout(location = 9) in vec3 vsg_Scale;
#endif

#ifdef VSG_SKINNING
layout(location = 10) in ivec4 vsg_JointIndices;
layout(location = 11) in vec4 vsg_JointWeights;

layout(set = MATERIAL_DESCRIPTOR_SET, binding = 12) readonly buffer JointMatrices
{
	mat4 matrices[];
} joint;
#endif

layout(location = 0) out vec3 eyePos;
layout(location = 1) out vec3 normalDir;
layout(location = 2) out vec4 vertexColor;
layout(location = 3) out vec2 texCoord[VSG_TEXCOORD_COUNT];
layout(location = 6) out vec3 viewDir;

out gl_PerVertex{
    vec4 gl_Position;
#ifdef VSG_POINT_SPRITE
    float gl_PointSize;
#endif
};

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

vec3 rotate(vec4 q, vec3 v)
{
    vec3 uv, uuv;
    vec3 qvec = vec3(q[0], q[1], q[2]);
    uv = cross(qvec, v);
    uuv = cross(qvec, uv);
    uv *= (2.0 * q[3]);
    uuv *= 2.0;
    return v + uv + uuv;
}

void main()
{
    vec4 vertex = vec4(vsg_Vertex, 1.0);
    vec4 normal = vec4(vsg_Normal, 0.0);

#ifdef VSG_DISPLACEMENT_MAP
    vec3 scale = displacementMapScale.value;

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

#ifdef VSG_INSTANCE_SCALE
    vertex.xyz = vertex.xyz * vsg_Scale;
#endif

#ifdef VSG_INSTANCE_ROTATION
    vertex.xyz = rotate(vsg_Rotation, vertex.xyz);
    normal.xyz = rotate(vsg_Rotation, normal.xyz);
#endif

#ifdef VSG_INSTANCE_TRANSLATION
    vertex.xyz = vertex.xyz + vsg_Translation;
#endif

#ifdef VSG_BILLBOARD
    mat4 mv = computeBillboadMatrix(pc.modelView * vec4(vsg_Translation_scaleDistance.xyz, 1.0), vsg_Translation_scaleDistance.w);
#elif defined(VSG_SKINNING)
    // Calculate skinned matrix from weights and joint indices of the current vertex
    mat4 skinMat =
        vsg_JointWeights.x * joint.matrices[vsg_JointIndices.x] +
        vsg_JointWeights.y * joint.matrices[vsg_JointIndices.y] +
        vsg_JointWeights.z * joint.matrices[vsg_JointIndices.z] +
        vsg_JointWeights.w * joint.matrices[vsg_JointIndices.w];

    mat4 mv = pc.modelView * skinMat;
#else
    mat4 mv = pc.modelView;
#endif

    gl_Position = (pc.projection * mv) * vertex;
    eyePos = (mv * vertex).xyz;
    viewDir = - (mv * vertex).xyz;
    normalDir = (mv * normal).xyz;

    vertexColor = vsg_Color;

#ifdef VSG_TEXTURECOORD_0
    texCoord[0] = vsg_TexCoord0;
#endif

#ifdef VSG_TEXTURECOORD_1
    texCoord[1] = vsg_TexCoord1;
#endif

#ifdef VSG_TEXTURECOORD_2
    texCoord[2] = vsg_TexCoord2;
#endif

#ifdef VSG_TEXTURECOORD_3
    texCoord[3] = vsg_TexCoord3;
#endif

#ifdef VSG_POINT_SPRITE
    gl_PointSize = 1.0;
#endif
}
