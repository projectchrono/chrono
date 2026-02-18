// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist, Han Wang, Yan Xiao
// =============================================================================
//
// RT util functions for material shading
//
// =============================================================================

#ifndef SHADER_UTILS_H
#define SHADER_UTILS_H

#include "chrono_sensor/optix/shaders/device_utils.h"

struct __device__ BSDFSample {
    float3 wo;
    float3 wi;
    float3 n;
    float3 f;
    float pdf;
};

__device__ __inline__ PerRayData_occlusion* GetOcclusionPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_occlusion*>(ints_as_pointer(opt0, opt1));
}

// Cosine-weighted hemisphere surface sampling
__device__ __inline__ float3 SampleCosineHemisphereDir(const float& z1, const float& z2, const float3& normal) {
    const float radius = sqrtf(z1);
    const float phi = 2.f * CUDART_PI_F * z2;
    float x = radius * cosf(phi);
    float y = radius * sinf(phi);
    float z = sqrtf(fmaxf(0.f, 1.f - x * x - y * y));

    // Address case of normal = (0, 0, 1)
    float3 binormal = (fabs(normal.x) > fabs(normal.z)) ? make_float3(-normal.y, normal.x, 0) : make_float3(0, -normal.z, normal.y);
    binormal = normalize(binormal);
    float3 tangent = Cross(normal, binormal);
    
    return x * tangent + y * binormal + z * normal;
}

static __device__ __inline__ float3 CrossProduct(const float3& a, const float3& b) {
    return {(a.y * b.z) - (b.y * a.z), (a.z * b.x) - (b.z * a.x), (a.x * b.y) - (b.x * a.y)};
}

static __device__ __inline__ float NormalDist(const float& NdH, const float& roughness) {
    float rough_sqr = roughness * roughness;
    float den_2 = NdH * NdH * (rough_sqr - 1.f) + 1.f;
    float denominator = den_2 * den_2;
    return rough_sqr / denominator;
}

// algorithm reference: https://www.gdcvault.com/play/1024478/PBR-Diffuse-Lighting-for-GGX
static __device__ __inline__ float HammonSmith(const float& NdV, const float& NdL, const float& roughness) {
    float denominator = lerp(2.f * abs(NdV) * abs(NdL), abs(NdL) + abs(NdV), roughness);
    return 0.5f / denominator;
}

// triangle mesh querie information
__device__ __inline__ void GetTriangleData(float3& normal, unsigned int& mat_id, float2& uv, float3& tangent, const unsigned int& mesh_id) {
    
    const int tri_id = optixGetPrimitiveIndex();
    const float2 bary_coord = optixGetTriangleBarycentrics();

    const MeshParameters& mesh_params = params.mesh_pool[mesh_id];
    const uint4& vertex_idx = mesh_params.vertex_index_buffer[tri_id];

    const float3& v1 = make_float3(mesh_params.vertex_buffer[vertex_idx.x]);
    const float3& v2 = make_float3(mesh_params.vertex_buffer[vertex_idx.y]);
    const float3& v3 = make_float3(mesh_params.vertex_buffer[vertex_idx.z]);

    //// Calculate normales either from normal buffer or vertex positions ////
    
    if (mesh_params.normal_index_buffer && mesh_params.normal_buffer) {  // Use vertex normals if normal index buffer exists
        const uint4& normal_idx = mesh_params.normal_index_buffer[tri_id];

        normal = normalize(make_float3(mesh_params.normal_buffer[normal_idx.y]) * bary_coord.x +
                           make_float3(mesh_params.normal_buffer[normal_idx.z]) * bary_coord.y +
                           make_float3(mesh_params.normal_buffer[normal_idx.x]) * (1.0f - bary_coord.x - bary_coord.y));

    }
    else {  // else use face normals calculated from vertices
        normal = normalize(Cross(v2 - v1, v3 - v1));
    }

    // calculate texcoords if they exist
    if (mesh_params.uv_index_buffer && mesh_params.uv_buffer) {  // use vertex normals if normal index buffer exists
        const uint4& uv_idx = mesh_params.uv_index_buffer[tri_id];
        const float2& uv1 = mesh_params.uv_buffer[uv_idx.x];
        const float2& uv2 = mesh_params.uv_buffer[uv_idx.y];
        const float2& uv3 = mesh_params.uv_buffer[uv_idx.z];

        uv = uv2 * bary_coord.x + uv3 * bary_coord.y + uv1 * (1.0f - bary_coord.x - bary_coord.y);
        float3 e1 = v2 - v1;
        float3 e2 = v3 - v1;
        float2 delta_uv1 = uv2 - uv1;
        float2 delta_uv2 = uv3 - uv1;
        float f = 1.f / (delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y);
        tangent.x = f * (delta_uv2.y * e1.x - delta_uv1.y * e2.x);
        tangent.y = f * (delta_uv2.y * e1.y - delta_uv1.y * e2.y);
        tangent.z = f * (delta_uv2.y * e1.z - delta_uv1.y * e2.z);
        tangent = normalize(tangent);
    } else {
        uv = make_float2(0.f);
        tangent = make_float3(0.f);
    }

    // get material index
    if (mesh_params.mat_index_buffer) {                  // use vertex normals if normal index buffer exists
        mat_id += mesh_params.mat_index_buffer[tri_id];  // the material index gives an offset id
    }
}

// Return default per ray data (PRD) of shadow
__device__ __inline__ PerRayData_shadow DefaultShadowPRD() {
    PerRayData_shadow prd = {
        make_float3(1.f, 1.f, 1.f),  // default opacity amount
        3,                           // default depth
        0.f                          // default distance remaining to light
    };                        
    return prd;
};

static __device__ inline void SamplePointLight(const float3& light_pos, const PointLightData& light_data, LightSample* ls) {
    ls->dir = normalize(light_pos - ls->hitpoint);  // How much slow down due to derefing hitpoint twice?
    float dist = Length(light_pos - ls->hitpoint);
    ls->dist = dist;
    ls->pdf = 1.f;
    float point_light_falloff = (light_data.max_range * light_data.max_range / (dist * dist + light_data.max_range * light_data.max_range));
    ls->L = light_data.color * point_light_falloff;
}

// static __device__ inline void SampleSpotLight(Light spot, LightSample* ls) {
//     ls->dir = normalize(spot.pos - ls->hitpoint);  // How much slow down due to derefing hitpoint twice?
//     float dist = Length(spot.pos - ls->hitpoint);
//     ls->dist = dist;
//     ls->pdf = 1.f;

//     float cos_theta = Dot(spot.spot_dir, -1 * ls->dir);

//     // Replace max range with a high intensity
//     // float point_light_falloff = (spot.max_range * spot.max_range / (dist * dist + spot.max_range * spot.max_range));
//     ls->L = spot.color / (dist * dist);

//     float falloff;  // spot light falloff
//     if (cos_theta >= spot.cos_falloff_start) {
//         falloff = 1.f;
//         return;
//     }
//     if (cos_theta < spot.cos_total_width) {
//         falloff = 0.f;
//         ls->L = make_float3(0.f);
//         return;
//     }

//     float delta = (cos_theta - spot.cos_total_width) / (spot.cos_falloff_start - spot.cos_total_width);
//     falloff = (delta * delta) * (delta * delta);

//     ls->L = ls->L * falloff;
//     // printf("falloff: %f | dist: %f | cosTheta: %f\n", falloff, dist, cos_theta*180/CUDART_PI);
// }

static __device__ inline void SampleLight(const ChOptixLight& light, LightSample* ls) {
    switch (light.light_type) {
        case LightType::POINT_LIGHT: {
            SamplePointLight(light.pos, light.specific.point, ls);
            break;
        }

        case LightType::SPOT_LIGHT:
            // printf("Sample Spot!\n");
            // SampleSpotLight(light, ls);
            break;
        default:
            break;
    }
}

// Get texture value in float
static __device__ __inline__ float GetTexValFloat(const cudaTextureObject_t& tex, const float2& text_scale, const float2& uv) {
    return tex2D<float>(tex, uv.x * text_scale.x, uv.y * text_scale.y);
}

static __device__ __inline__ float3 GetTexFloat4ValFloat3(const cudaTextureObject_t& tex, const float2& text_scale, const float2& uv) {
    float4 temp = tex2D<float4>(tex, uv.x * text_scale.x, uv.y * text_scale.y);
    return {temp.x, temp.y, temp.z};
}


static __device__ __inline__ float3 PrincipledBRDF(
    const float3& albedo, const float& roughness, const float& metallic, const float3& specular, const float& contrib_weight,
    const bool& use_specular_workflow, const float& NdV, const float& NdL, const float& NdH, const float& VdH
) {
    // float3 subsurface_albedo_updated = subsurface_albedo;

    // ==== Specular portion of reflection ==== //
    float3 F = make_float3(0.0f);
    // Use dielectric workflow
    if (use_specular_workflow) {
        float3 F0 = specular * 0.08f;
        F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
    }
    else {
        F = metallic * albedo + (1 - metallic) * make_float3(0.04f); // default dielectrics F0 = [0.04, 0.04, 0.04]
    }
    // albedo * (1 - metallic): since metals do not do surface reflection
    float3 light_reflected_ratio = ((make_float3(1.f) - F) * albedo * (1 - metallic)) * contrib_weight * NdL;
    
    // ==== Diffuse portion of reflection ==== //
    // D = NormalDist(NdH, roughness), 1/pi omitted; G = HammonSmith(NdV, NdL, roughness), 4 * NdV * NdL omitted
    float3 f_ct = F * NormalDist(NdH, roughness) * HammonSmith(NdV, NdL, roughness);
    light_reflected_ratio += f_ct * contrib_weight * NdL;

    return clamp(light_reflected_ratio, make_float3(0.f), make_float3(1.f));
}


// Account for Fog effect
static __device__ __inline__ void AddFogEffect(
    PerRayData_camera* prd_camera, const ContextParameters& cntxt_params, const float& ray_dist
) {
    if (prd_camera->use_fog && cntxt_params.fog_scattering > 0.f) {
        float blend_alpha = expf(-cntxt_params.fog_scattering * ray_dist);
        prd_camera->color = blend_alpha * prd_camera->color + (1 - blend_alpha) * cntxt_params.fog_color;
    }
}



static __device__ __inline__ float LambertianBSDFPdf(float3& wo, float3& wi, float3& n) {
    // float WodWi = Dot(wo,wi);
    float NdWi = Dot(n, wi);
    return NdWi > 0 ? NdWi * INV_PI : 0;
}

static __device__ __inline__ void LambertianBSDFSample(BSDFSample& sample,
                                                       const MaterialParameters& mat,
                                                       const float2& uv,
                                                       bool eval,
                                                       float z1,
                                                       float z2) {
    float3 Kd = mat.Kd;
    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x * mat.tex_scale.x, uv.y * mat.tex_scale.y);
        // transfer sRGB texture into linear color space.
        Kd = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
        // printf("Querying Texture map| Kd:(%f,%f,%f)\n", Kd.x, Kd.y, Kd.z);
    }
    sample.f = Kd * INV_PI;
    if (eval)
        return;

    sample.wi = SampleCosineHemisphereDir(z1, z2, sample.n);
    
    // Probability density function (PDF) of the Labertian BSDF for the sampled direction
    sample.pdf = LambertianBSDFPdf(sample.wo, sample.wi, sample.n);
}

// Retroreflective BSDF PDF function
static __device__ __inline__ float RetroreflectiveBSDFPdf(float3& wo, float3& wi, float3& n) {
    float WodWi = Dot(wo, wi);
    float NdWi = Dot(n, wi);
    return WodWi > .999f ? NdWi : 0.f;
}

// Retroreflective BSDF Sampling function
static __device__ __inline__ void RetroreflectiveBSDFSample(BSDFSample& sample,
                                                            const MaterialParameters& mat,
                                                            bool eval,
                                                            float z1,
                                                            float z2) {
    // Use Ks as the efficiency factor
    sample.f = mat.Ks;
    if (eval)
        return;

    sample.wi = sample.wo;
    sample.pdf = RetroreflectiveBSDFPdf(sample.wo, sample.wi, sample.n);
}

static __device__ __inline__ BSDFSample SampleBSDF(BSDFType type,
                                                   BSDFSample& sample,
                                                   const MaterialParameters& mat,
                                                   const float2& uv,
                                                   bool eval = false,
                                                   float z1 = 0,
                                                   float z2 = 0) {
    switch (type) {
        case BSDFType::DIFFUSE:
            LambertianBSDFSample(sample, mat, uv, eval, z1, z2);
            break;
        case BSDFType::SPECULAR:
            break;
        case BSDFType::DIELECTRIC:
            break;
        case BSDFType::GLOSSY:
            break;
        case BSDFType::PRINCIPLED:
            LambertianBSDFSample(sample, mat, uv, eval, z1, z2);
            break;
        case BSDFType::HAPKE:
            break;
        case BSDFType::RETROREFLECTIVE:
            RetroreflectiveBSDFSample(sample, mat, eval, z1, z2);
            break;
        default:
            break;
    }

    return sample;
}

static __device__ __inline__ float EvalBSDFPDF(BSDFType type, float3& wo, float3& wi, float3& n) {
    float pdf;
    switch (type) {
        case BSDFType::DIFFUSE:
            pdf = LambertianBSDFPdf(wo, wi, n);
            break;
        case BSDFType::SPECULAR:
            break;
        case BSDFType::DIELECTRIC:
            break;
        case BSDFType::GLOSSY:
            break;
        case BSDFType::PRINCIPLED:
            break;
        case BSDFType::HAPKE:
            break;
        case BSDFType::RETROREFLECTIVE:
            pdf = RetroreflectiveBSDFPdf(wo, wi, n);
            break;
        default:
            break;
    }

    return pdf;
}


// Importance Sampling power heurustic method
static __device__ __inline__ float ISPowerHeuristic(int nf, float fPdf, int ng, float gPdf) {
    float f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g);
}

// Compute direct lighting contribution from a light source
static __device__ __inline__ float3 ComputeDirectLight(ChOptixLight& l,
                                                       LightSample& ls,
                                                       const MaterialParameters& mat,
                                                       const float2& uv,
                                                       int depth) {
    float3 Ld = make_float3(0.f);
    SampleLight(l, &ls);
    BSDFType bsdf_type = mat.bsdf_type;
    if (ls.pdf > 0 && fmaxf(ls.L) > 0) {
        float NdL = Dot(ls.n, ls.dir);
        if (NdL > 0) {
            // Evaluate BSDF at light direction
            BSDFSample bsdf;
            bsdf.wi = ls.dir;
            bsdf.wo = ls.wo;
            bsdf.n = ls.n;
            SampleBSDF(bsdf_type, bsdf, mat, uv, true);
            float scatterPDF = EvalBSDFPDF(bsdf_type, bsdf.wo, bsdf.wi, bsdf.n);
            if (!(fmaxf(bsdf.f) > 0))
                return Ld;  // If the BSDF is black, direct light contribution  is 0?
            // Shoot shadow rays
            PerRayData_shadow prd_shadow = DefaultShadowPRD();
            prd_shadow.depth = depth + 1;
            prd_shadow.ramaining_dist = ls.dist;
            unsigned int opt1;
            unsigned int opt2;
            pointer_as_ints(&prd_shadow, opt1, opt2);
            unsigned int raytype = static_cast<unsigned int>(RayType::SHADOW_RAY_TYPE);
            optixTrace(params.root, ls.hitpoint, ls.dir, params.scene_epsilon, ls.dist, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

            // light contribution
            float3 light_contrib = bsdf.f * NdL * (prd_shadow.attenuation);
            // printf("L Contr: (%f,%f,%f)\n", light_contrib.x,light_contrib.y, light_contrib.z);
            if (l.delta) {
                Ld += light_contrib * ls.L / ls.pdf;
            } else {
                float is_weight = ISPowerHeuristic(1, ls.pdf, 1, scatterPDF);
                Ld += light_contrib * ls.L * is_weight / ls.pdf;
            }
        }
    }

    // TODO:: Add MIS computation for non delta lights

    return Ld;
}

#endif // SHADER_UTILS_H