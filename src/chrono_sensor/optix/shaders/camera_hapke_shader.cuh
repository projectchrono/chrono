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
// Authors: Nevindu M. Batagoda
// =============================================================================
//
// Camera shader based on Hapke's BRDF model for simulating light interaction with
// particulate surfaces such as planetary regoliths. We implemented based on the
// modern Hapke model descried in  https://doi.org/10.1002/2013JE004580
//
// =============================================================================

#ifndef CAMERA_HAPKE_SHADER_CU
#define CAMERA_HAPKE_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.cuh"
#include "chrono_sensor/optix/shaders/shader_utils.cuh"


/// @brief Camera shader based on Hapke's BRDF model for simulating light interaction with particulate surfaces such as planetary regoliths.
/// @param prd_camera Pointer to the PerRayData_camera struct containing the ray and its contribution to the pixel color
/// @param mat_params Pointer to the MaterialRecordParameters struct containing the material properties of the hit object
/// @param material_id Material ID of the hit object
/// @param num_blended_materials Number of blended materials on the hit object (can use a weight file per material to blend)
/// @param world_normal Normal vector in world coordinates of the hit point
/// @param uv UV coordinates of the hit point on the surface of the object
/// @param tangent Tangent vector in world coordinates of the hit point
/// @param ray_dist Distance from the ray origin to the hit point
/// @param hit_point Position in world coordinates of the hit point
/// @param ray_dir Direction vector in world coordinates of the ray
static __device__ inline void CameraHapkeShader(PerRayData_camera* prd_camera,
                                                const MaterialRecordParameters* mat_params,
                                                unsigned int& material_id,
                                                const unsigned int& num_blended_materials,
                                                const float3& world_normal,
                                                const float2& uv,
                                                const float3& tangent,
                                                const float& ray_dist,
                                                const float3& hit_point,
                                                const float3& ray_dir) {
    // printf("Distance: %.2f\n", ray_dist);
    // prd_camera->color += make_float3(ray_dist, ray_dist, ray_dist);
    // prd_camera->color += make_float3(1.f,1.f,1.f);
    // return;
    // printf("Hapke Shader!\n");
    //  float w = 0.32357f; // average single scattering albedo
    //  float b = 0.23955f; // shape controlling parameter for the amplitude of backward and forward scatter of
    //  particles float c = 0.30452f; // weighting factor that controls the contribution of backward and forward
    //  scatter. float B_s0 = 1.80238f; float h_s = 0.07145f; float B_c0 = 0.0f; float h_c = 1.0f; float phi = 0.3f;
    //  //float K = 1.0f;
    //  float theta_p = 23.4f*(CUDART_PI_F/180);

    const MaterialParameters& mat = params.material_pool[material_id];  // Assume no blended materials for now
    float3 subsarface_albedo = mat.Kd;
    float3 specular = mat.Ks;

    // Get Hapke material parameters
    float w = mat.w;
    float b = mat.b;
    float c = mat.c;
    float B_s0 = mat.B_s0;
    float h_s = mat.h_s;
    float B_c0 = 0.0f;
    float h_c = 1.0f;
    float phi = mat.phi;
    float theta_p = mat.theta_p;
    float cos_e = Dot(world_normal, -ray_dir);

    float3 reflected_color = make_float3(0.0f);
    {
        for (int i = 0; i < params.num_lights; i++) {
            ChOptixLight l = params.lights[i];
            LightSample ls;
            ls.hitpoint = hit_point;
            ls.wo = -ray_dir;
            ls.n = world_normal;
            SampleLight(l, &ls);

            float dist_to_light = ls.dist;  // Length(l.pos - hit_point);
            // printf("dist_to_light:%.4f\n", dist_to_light);
            if (1) {  // dist_to_light < 2 * l.max_range{ // Sun should have infinity range, so this condition will
                      // always be true for ths sun
                float3 dir_to_light = ls.dir;  // normalize(l.pos - hit_point);
                float cos_i = Dot(dir_to_light, world_normal);
                // printf("cos_i:%.2f",cos_i);
                if (cos_i > 0) {
                    // Cast a shadow ray to see any attenuation of light
                    PerRayData_shadow prd_shadow = DefaultShadowPRD();
                    prd_shadow.depth = prd_camera->depth + 1;
                    prd_shadow.ramaining_dist = dist_to_light;
                    unsigned int opt1;
                    unsigned int opt2;
                    pointer_as_ints(&prd_shadow, opt1, opt2);
                    unsigned int raytype = (unsigned int)RayType::SHADOW_RAY_TYPE;
                    optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light,
                               optixGetRayTime(), OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2,
                               raytype);

                    float3 light_attenuation = prd_shadow.attenuation;

                    float point_light_falloff = 1.0f;  // ??
                    float3 incoming_light_ray = ls.L * light_attenuation * cos_i;
                    // float3 incoming_light_ray = l.color * cos_i * light_attenuation; // Add attenuation later
                    // printf("incoming_light_ray: (%.2f,%.2f,%.2f)\n", incoming_light_ray.x, incoming_light_ray.y,
                    // incoming_light_ray.z);
                    if (fmaxf(incoming_light_ray) > 0.0f) {
                        float cos_g = Dot(dir_to_light, -ray_dir);
                        float sin_i = sqrt(1 - (cos_i * cos_i));  // + sqrt
                        float sin_e = sqrt(1 - (cos_e * cos_e));
                        float sin_g = sqrt(1 - (cos_g * cos_g));

                        float tan_i = sin_i / cos_i;
                        float tan_e = sin_e / cos_e;

                        float cot_i = 1 / tan_i;
                        float cot_e = 1 / tan_e;
                        float cot_i_sq = cot_i * cot_i;
                        float cot_e_sq = cot_e * cot_e;

                        // Calculate Psi
                        float cos_psi = Dot(normalize(dir_to_light - (cos_i * world_normal)),
                                            normalize(-ray_dir - (cos_e * world_normal)));
                        float psi = acos(cos_psi);
                        float psi_half = psi / 2;
                        float f_psi = expf(-2 * tan(psi_half));
                        float sin_psi_half = sin(psi_half);
                        float sin_psi_half_sq = sin_psi_half * sin_psi_half;
                        float psi_per_pi = psi / CUDART_PI_F;  // TODO: Define 1/PI as a constant

                        float tan_theta_p = tan(theta_p);
                        float tan_theta_p_sq = tan_theta_p * tan_theta_p;

                        float cot_theta_p = 1 / tan_theta_p;
                        float cot_theta_p_sq = cot_theta_p * cot_theta_p;

                        float E_1_i = expf((-2 / CUDART_PI_F) * cot_theta_p * cot_i);
                        float E_2_i = expf((-1 / CUDART_PI_F) * cot_theta_p * cot_theta_p * cot_i * cot_i);

                        float E_1_e = expf((-2 / CUDART_PI_F) * cot_theta_p * cot_e);
                        float E_2_e = expf((-1 / CUDART_PI_F) * cot_theta_p * cot_theta_p * cot_e * cot_e);

                        float chi_theta_p = 1 / sqrtf(1 + CUDART_PI_F * tan_theta_p_sq);

                        float eta_i = chi_theta_p * (cos_i + sin_i * tan_theta_p * E_2_i / (2 - E_1_i));
                        float eta_e = chi_theta_p * (cos_e + sin_e * tan_theta_p * E_2_e / (2 - E_1_e));

                        float mu0 = cos_i;
                        float mu = cos_e;
                        float mu0_e = chi_theta_p;
                        float mu_e = chi_theta_p;
                        float S = 0.0f;
                        if (cos_i >= cos_g) {  // for x,y \in [0,pi], if x <= y => cos(x) >= cos(y)
                            mu0_e *= cos_i + sin_i * tan_theta_p * (cos_psi * E_2_e + sin_psi_half_sq * E_2_i) /
                                                 (2 - E_1_e - psi_per_pi * E_1_i);

                            mu_e *= cos_e + sin_e * tan_theta_p * (E_2_e - sin_psi_half_sq * E_2_i) /
                                                (2 - E_1_e - psi_per_pi * E_1_i);

                            S = mu_e / eta_e * mu0 / eta_i * chi_theta_p /
                                (1 - f_psi + f_psi * chi_theta_p * (mu0 / eta_i));
                        } else {
                            mu0_e *= cos_i + sin_i * tan_theta_p * (E_2_i - sin_psi_half_sq * E_2_e) /
                                                 (2 - E_1_i - psi_per_pi * E_1_e);

                            mu_e *= cos_e + sin_e * tan_theta_p * (cos_psi * E_2_i + sin_psi_half_sq * E_2_e) /
                                                (2 - E_1_i - psi_per_pi * E_1_e);

                            S = mu_e / eta_e * mu0 / eta_i * chi_theta_p /
                                (1 - f_psi + f_psi * chi_theta_p * (mu / eta_e));
                        }

                        float KPhi = 1.209 * pow(phi, 2.0f / 3);
                        float K = -log(1 - KPhi) / KPhi;

                        float tan_ghalf = sin_g / (1 + cos_g);
                        float tan_ghalf_per_hC = tan_ghalf / h_c;

                        float B_C = 0.0f;
                        if (cos_g < 1.0f)
                            B_C = (1 + (1 - exp(-tan_ghalf_per_hC)) / tan_ghalf_per_hC) /
                                  (2 * pow(1 + tan_ghalf_per_hC, 2.0f));
                        else if (cos_g == 1)
                            B_C = 1;

                        float r0Term = sqrt(1 - w);
                        float r0 = (1 - r0Term) / (1 + r0Term);

                        float LS = mu0_e / (mu0_e + mu_e);
                        float b_sq = b * b;

                        float twobcos_g = 2 * b * cos_g;

                        float p_g = (1 + c) / 2 * (1 - b_sq) / pow(1 - (2 * b * cos_g) + b_sq, 1.5f) +
                                    (1 - c) / 2 * (1 - b_sq) / pow(1 + (2 * b * cos_g) + b_sq, 1.5f);

                        float B_S = 1 / (1 + tan_ghalf / h_s);

                        float x_i = mu0_e / K;
                        float x_e = mu_e / K;
                        float H_i = 1 / (1 - w * x_i * (r0 + (1 - 2 * r0 * x_i) / 2 * log((1 + x_i) / x_i)));
                        float H_e = 1 / (1 - w * x_e * (r0 + (1 - 2 * r0 * x_e) / 2 * log((1 + x_e) / x_e)));

                        float M = H_i * H_e - 1;
                        float f_ct = LS * K * w / (4 * CUDART_PI_F) * (p_g * (1 + B_s0 * B_S) + M) * (1 + B_c0 * B_C) *
                                     S / cos_i;
                        // printf("fct:%.2f\n", f_ct);
                        reflected_color += f_ct * incoming_light_ray * subsarface_albedo;
                        // printf("reflected_color:(%.2f,%.2f,%.2f)\n", reflected_color.x, reflected_color.y,
                        // reflected_color.z);
                    }
                }
            }
        }
    }
    // printf("reflected_color:(%.2f,%.2f,%.2f)\n", reflected_color.x, reflected_color.y, reflected_color.z);
    prd_camera->color += reflected_color;
}

#endif  // CAMERA_HAPKE_SHADER_CU