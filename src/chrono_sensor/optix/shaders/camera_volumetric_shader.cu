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
// Camera volumetric shader using NanoVDB for volume rendering
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"


static __device__ __inline__ float SchlickPhase(float VdL, float k) {
    float numerator = 1 - (k * k);
    float denominator = 4 * CUDART_PI * (1 - k * VdL) * (1 - k * VdL);
    return numerator / denominator;
}

/// @brief Camera volumetric shader using NanoVDB for volume rendering
/// @param prd_camera Pointer to the PerRayData_camera struct containing the ray and its contribution to the pixel color
/// @param mat_params Pointer to the MaterialRecordParameters struct containing material properties of the hit object
/// @param material_id Material ID of the hit object
/// @param num_blended_materials Number of blended materials on the hit object
/// @param world_normal Normal vector in world coordinates of the hit point
/// @param uv UV coordinates of the hit point on the surface of the object
/// @param tangent Tangent vector in world coordinates of the hit point
/// @param ray_dist Distance from the ray origin to the hit point
/// @param ray_orig Origin of the incoming ray in world coordinates
/// @param ray_dir Direction of the incoming ray in world coordinates
static __device__ inline void CameraVolumetricShader(PerRayData_camera* prd_camera,
                                                     const MaterialRecordParameters* mat_params,
                                                     unsigned int& material_id,
                                                     const unsigned int& num_blended_materials,
                                                     const float3& world_normal,
                                                     const float2& uv,
                                                     const float3& tangent,
                                                     const float& ray_dist,
                                                     const float3& ray_orig,
                                                     const float3& ray_dir) {
#ifdef USE_SENSOR_NVDB
    // printf("VOL SHADER!\n");
    const MaterialParameters& mat = params.material_pool[material_id];
    nanovdb::NanoGrid<float>* grid =
        params.density_grid_ptr;  // TODO:: Add Multiple handle types for volume and CRM VDB grids
    const nanovdb::Vec3f ray_orig_v(ray_orig.x, ray_orig.y, ray_orig.z);
    const nanovdb::Vec3f ray_dir_v(ray_dir.x, ray_dir.y, ray_dir.z);
    float3 hitPoint = ray_orig + ray_dir * ray_dist;

    nanovdb::Vec3d hitPointIdx = grid->worldToIndex(nanovdb::Vec3d(hitPoint.x, hitPoint.y, hitPoint.z));
    nanovdb::Vec3d rayDirIdx = grid->worldToIndex(ray_dir_v);
    nanovdb::Vec3d rayOrigIdx = grid->worldToIndex(ray_orig_v);

    nanovdb::Ray<float> ray(rayOrigIdx, rayDirIdx, ray_dist, 1e20);
    /*printf("VolShader: ray_dist: %f| hitP: %f,%f,%f | hitP Idx: %f,%f,%f | rayStartIdx: %f %f %f | rayDirIdx:
       %f,%f,%f\n", ray_dist, hitPoint.x, hitPoint.y, hitPoint.z, hitPointIdx[0], hitPointIdx[1], hitPointIdx[2],
       ray.start()[0], ray.start()[1], ray.start()[2], rayDirIdx[0], rayDirIdx[1], rayDirIdx[2]);*/

    nanovdb::Coord ijk = nanovdb::RoundDown<nanovdb::Coord>(ray.start());  // first hit of bbox
    // printf("ZCrossing::ray.start(): (%f,%f,%f) | ray.dir(): (%f,%f,%f)\n", ray.start()[0], ray.start()[1],
    // ray.start()[2], ray.dir()[0],ray.dir()[1],ray.dir()[2]);

    float v;
    nanovdb::DefaultReadAccessor<float> acc = grid->tree().getAccessor();
    nanovdb::HDDA<nanovdb::Ray<float>, nanovdb::Coord> hdda(ray, acc.getDim(ijk, ray));
    const auto v0 = acc.getValue(ijk);

    // printf("Start Value: %f | Start Idx: %f,%f,%f\n", v0, ijk.asVec3d()[0], ijk.asVec3d()[1], ijk.asVec3d()[2]);
    static const float Delta = 1e-3;  // 1.0001f;
    int nsteps = 0;
    float transmittance = 1.0f;
    float absorptionCoeff = mat.absorption_coefficient;
    float scatteringCoeff = mat.scattering_coefficient;
    float extinctionCoeff = absorptionCoeff + scatteringCoeff;
    float3 inScattering = make_float3(0);
    float outScattering = 0;
    float k = 0;  // isotropic reflections
    float3 volAlbedo = mat.Kd;

    int inactiveSteps = 0;
    float3 volumeLight = make_float3(0);
    while (hdda.step() && nsteps < 100) {
        ijk = nanovdb::RoundDown<nanovdb::Coord>(ray(hdda.time() + Delta));
        hdda.update(ray, acc.getDim(ijk, ray));
        if (hdda.dim() > 1 || !acc.isActive(ijk)) {
            inactiveSteps++;
            if (inactiveSteps > 1000)
                break;
            continue;  // either a tile value or an inactive voxel
        }

        // sample lights
        while (hdda.step() && acc.isActive(hdda.voxel())) {  // in the narrow band
            v = acc.getValue(hdda.voxel());                  // density
            // printf("density: %f\n", v);
            ijk = hdda.voxel();
            nanovdb::Vec3f volPntIdx =
                grid->indexToWorld(ijk.asVec3s());  // TODO: Make VDB to chrono data type conversion function
            float3 volPnt = make_float3(volPntIdx[0], volPntIdx[1], volPntIdx[2]);

            transmittance *= exp((-v * extinctionCoeff * 1.0001));
            outScattering = scatteringCoeff * v;
            // printf("density: %f| trans: %f | outscat: %f\n", v, transmittance, outScattering);
            for (int i = 0; i < params.num_lights; i++) {
                Light l = params.lights[i];
                if (l.type != LightType::POINT_LIGHT)
                    continue;
                float dist_to_light = Length(l.pos - volPnt);
                if (dist_to_light < 2 * l.max_range) {
                    float3 dir_to_light = normalize(l.pos - volPnt);
                    float VdL = Dot(-1 * ray_dir, dir_to_light);
                    // printf("VdL: %f | dirL: (%f,%f,%f) | dirR: (%f,%f,%f)\n", VdL, dir_to_light.x, dir_to_light.y,
                    // dir_to_light.z, ray_dir.x, ray_dir.y, ray_dir.z);
                    if (VdL > 0) {
                        float3 light_attenuation = make_float3(0);  // TODO: shoot shadow ray
                        {
                            // Ray march to determine light attenuation
                            nanovdb::Ray<float> sRay(
                                ijk.asVec3d(),
                                grid->worldToIndex(nanovdb::Vec3d(dir_to_light.x, dir_to_light.y, dir_to_light.z)), 0.f,
                                1e20);
                            nanovdb::Coord sijk = nanovdb::RoundDown<nanovdb::Coord>(sRay.start());
                            nanovdb::HDDA<nanovdb::Ray<float>, nanovdb::Coord> shdda(sRay, acc.getDim(sijk, sRay));
                            int sinactiveSteps = 0;
                            int ssteps = 0;
                            float sV = 0;
                            float sTransmittance = 1.0f;
                            while (shdda.step() && ssteps < 50) {
                                sijk = nanovdb::RoundDown<nanovdb::Coord>(sRay(shdda.time() + Delta));
                                shdda.update(sRay, acc.getDim(sijk, sRay));
                                if (shdda.dim() > 1 || !acc.isActive(sijk)) {
                                    sinactiveSteps++;
                                    if (sinactiveSteps > 20)
                                        break;
                                    continue;  // either a tile value or an inactive voxel
                                }
                                while (shdda.step() && acc.isActive(shdda.voxel())) {  // in the narrow band
                                    sV = acc.getValue(shdda.voxel());
                                    sTransmittance *= exp((-sV * extinctionCoeff * 1.0001f));  // density
                                    ssteps++;
                                }
                            }
                            light_attenuation = make_float3(clamp(sTransmittance, 0.f, 1.f));
                        }
                        // printf("Light Atten: %f\n", light_attenuation.x);
                        float point_light_falloff =
                            (l.max_range * l.max_range / (dist_to_light * dist_to_light + l.max_range * l.max_range));

                        float3 incoming_light_ray =
                            l.color * light_attenuation * point_light_falloff * VdL * mat.emissive_power;
                        float phase = SchlickPhase(VdL, k);
                        inScattering = params.ambient_light_color + incoming_light_ray * phase;

                        volumeLight += transmittance * inScattering * outScattering * 1.0001f * volAlbedo;
                        // print transmittance, inScattering, outScattering, Delta, volAlbedo
                        // printf("transmittance: %f | inScattering: (%f,%f,%f) | outScattering: %f | Delta: %f |
                        // volAlbedo: (%f,%f,%f)\n", transmittance, inScattering.x, inScattering.y, inScattering.z,
                        // outScattering, Delta, volAlbedo.x, volAlbedo.y, volAlbedo.z);
                    }
                }
            }
            nsteps++;
            // break;
        }
    }
    float alpha = 1 - clamp(transmittance, 0, 1);
    if (nsteps > 0) {
        prd_camera->transparency = 1 - alpha;
        prd_camera->color = volumeLight;  // make_float3(1-alpha, 1-alpha, 1-alpha);
        // printf("transparency: %f | color: (%f,%f,%f)\n", prd_camera->transparency,
        // prd_camera->color.x,prd_camera->color.y,prd_camera->color.z);
        //  prd_camera->color += make_float3(0, 0, 1);
    } else {
        prd_camera->transparency = 1.f;
        prd_camera->color = make_float3(0, 0, 0);  // 0.1f, 0.2f, 0.4f
    }
#endif
}