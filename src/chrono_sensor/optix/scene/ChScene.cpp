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
// Authors: Asher Elmquist
// =============================================================================
//
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_sensor/optix/scene/ChScene.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChScene::ChScene() {
    m_background.mode = BackgroundMode::GRADIENT;
    m_background.color_zenith = {0.4f, 0.5f, 0.6f};
    m_background.color_horizon = {0.7f, 0.8f, 0.9f};
    m_background.env_tex = "";

    m_ambient_light = ChVector3f({.2f, .2f, .2f});
    m_lights = std::vector<ChOptixLight>();
    lights_changed = true;
    background_changed = true;

    m_fog_color = ChVector3f(1.f, 1.f, 1.f);
    m_fog_scattering = 0.f;

    m_scene_epsilon = 1e-3f;
    m_dynamic_origin_threshold = 100.f;
    m_dynamic_origin_offset = false;
    m_sprites = std::vector <std::shared_ptr<ChBody>>();

    //m_nvdb = nullptr;
}

CH_SENSOR_API ChScene::~ChScene() {
    // free environment light CDF arrays if they exist
    for (int light_idx = 0; light_idx < m_lights.size(); light_idx++) {
        if (m_lights[light_idx].light_type == LightType::ENVIRONMENT_LIGHT) {
            cudaFree(m_lights[light_idx].specific.environment.dev_cdf_lat);
            cudaFree(m_lights[light_idx].specific.environment.dev_cdf_lon);
        }
        // ---- Register Your Customized Light Here (add destructor code if necessary) ---- //
    }
}

/// Point light ///

CH_SENSOR_API unsigned int ChScene::AddPointLight(ChVector3f pos, ChColor color, float max_range, bool const_color) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::POINT_LIGHT;
    light.pos = {pos.x(), pos.y(), pos.z()};
    light.delta = true;
    light.specific.point.color = {color.R, color.G, color.B};
    light.specific.point.max_range = max_range;
    light.specific.point.const_color = const_color;
    
    // Calculate extended parameters
    light.specific.point.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;

    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyPointLight(unsigned int id, const ChOptixLight& point_light) {
    if (id <= m_lights.size() && point_light.light_type == LightType::POINT_LIGHT) {
        m_lights[id] = point_light;
        lights_changed = true;
    }
}

/// ---- Directional light ---- ///

CH_SENSOR_API unsigned int ChScene::AddDirectionalLight(ChColor color, float elevation, float azimuth) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::DIRECTIONAL_LIGHT;
    light.pos = {0.f, 0.f, 0.f};
    light.delta = true;
    light.specific.directional.color = {color.R, color.G, color.B};
    light.specific.directional.elevation = elevation;
    light.specific.directional.azimuth = azimuth;

    // Calculate extended parameters
    light.specific.directional.light_dir = {cosf(elevation) * cosf(azimuth), cosf(elevation) * sinf(azimuth), sinf(elevation)};

    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyDirectionalLight(unsigned int id, const ChOptixLight& directional_light) {
    if (id <= m_lights.size() && directional_light.light_type == LightType::DIRECTIONAL_LIGHT) {
        m_lights[id] = directional_light;
        lights_changed = true;
    }
}

/// ---- Spot light ---- ///

CH_SENSOR_API unsigned int ChScene::AddSpotLight(
    ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float angle_falloff_start, float angle_range, bool const_color
) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::SPOT_LIGHT;
    light.pos = make_float3(pos.x(), pos.y(), pos.z());
    light.delta = true;
    light.specific.spot.color = make_float3(color.R, color.G, color.B);
    light.specific.spot.max_range = max_range;
    ChVector3f nrmlz_light_dir = light_dir.GetNormalized();
    light.specific.spot.light_dir = make_float3(nrmlz_light_dir.x(), nrmlz_light_dir.y(), nrmlz_light_dir.z());
    light.specific.spot.const_color = const_color;
    light.specific.spot.angle_range = angle_range;
    if (angle_falloff_start < angle_range - 1e-6f) {
        light.specific.spot.angle_falloff_start = angle_falloff_start;
        light.specific.spot.angle_atten_rate = 1.f / (angle_range - angle_falloff_start);
    }
    // ineffective value of angle_falloff_start, set to no angular attenuation
    else {
        light.specific.spot.angle_falloff_start = angle_range;
        light.specific.spot.angle_atten_rate = -1.f;
    }

    // Calculate extended parameters
    light.specific.spot.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;

    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifySpotLight(unsigned int id, const ChOptixLight& spot_light) {
    if (id <= m_lights.size() && spot_light.light_type == LightType::SPOT_LIGHT) {
        m_lights[id] = spot_light;
        lights_changed = true;
    }
}

/// ---- Rectangle light ---- ///

CH_SENSOR_API unsigned int ChScene::AddRectangleLight(
    ChVector3f pos, ChColor color, float max_range, ChVector3f length_vec, ChVector3f width_vec, bool const_color
) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::RECTANGLE_LIGHT;
    light.pos = make_float3(pos.x(), pos.y(), pos.z());
    light.delta = false;
    light.specific.rectangle.color = {color.R, color.G, color.B};
    light.specific.rectangle.max_range = max_range;
    light.specific.rectangle.length_vec = make_float3(length_vec.x(), length_vec.y(), length_vec.z());
    light.specific.rectangle.width_vec = make_float3(width_vec.x(), width_vec.y(), width_vec.z());
    light.specific.rectangle.const_color = const_color;

    // Calculate extended parameters
    light.specific.rectangle.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;
    ChVector3f light_dir = Vcross(length_vec, width_vec);
    light.specific.rectangle.area = light_dir.Length();
    light_dir = light_dir / light_dir.Length();
    light.specific.rectangle.light_dir = make_float3(light_dir.x(), light_dir.y(), light_dir.z());
    
    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyRectangleLight(unsigned int id, const ChOptixLight& rectangle_light) {
    if (id <= m_lights.size() && rectangle_light.light_type == LightType::RECTANGLE_LIGHT) {
        m_lights[id] = rectangle_light;
        lights_changed = true;
    }
}

/// ---- Disk light ---- ///

CH_SENSOR_API unsigned int ChScene::AddDiskLight(
   ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float radius, bool const_color
) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::DISK_LIGHT;
    light.pos = make_float3(pos.x(), pos.y(), pos.z());
    light.delta = false;
    light.specific.disk.color = {color.R, color.G, color.B};
    light.specific.disk.max_range = max_range;
    ChVector3f nrmlz_light_dir = light_dir.GetNormalized();
    light.specific.disk.light_dir = make_float3(nrmlz_light_dir.x(), nrmlz_light_dir.y(), nrmlz_light_dir.z());
    light.specific.disk.radius = radius;
    light.specific.disk.const_color = const_color;

    // Calculate extended parameters
    light.specific.disk.atten_scale = (max_range > 0) ? (0.01 * max_range * max_range) : 1.f;
    light.specific.disk.area = CH_PI * radius * radius;
    
    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyDiskLight(unsigned int id, const ChOptixLight& disk_light) {
    if (id <= m_lights.size() && disk_light.light_type == LightType::DISK_LIGHT) {
        m_lights[id] = disk_light;
        lights_changed = true;
    }
}

/// ---- Environment light ---- ///

/// @brief Build the cumulative distribution function (CDF) for importance sampling of the environment map on the device.
/// @param img the environment map image data, assumed to be in RGB format with 8 bits per channel
/// @param dev_cdf_lat device pointer to the CDF for latitude sampling, size = height x 2 Bytes, to be filled in this function
/// @param cdf_lon device pointer to the CDF for longitude sampling, size = height x width x 2 Bytes, to be filled in this function 
void BuildCDFOnDevice(const ByteImageData& img, EnvironmentLightData& env_light_data) {
    
    const int img_w = img.w;
    const int img_h = img.h;
    const int num_chs = img.c;
    std::vector<float> cdf_lat_float(img_h, 0.f);
    std::vector<float> cdf_lon_float(img_w * img_h, 0.f);
    std::vector<unsigned short> host_cdf_lat(img_h, 0);
    std::vector<unsigned short> host_cdf_lon(img_w * img_h, 0);

    float max_luminance = 0.f;
    float min_luminance = 1.f;
    for (int v = 0; v < img_h; ++v) {
        // sin(polar) for importance sampling, polar in 0 --> pi, so sin(polar) is: 0 --> 1 --> 0
        // equivalent to cos(elevation), elevation in pi/2 --> -pi/2, so cos(elevation) is: 0 --> 1 --> 0
        float cos_elev = cosf((static_cast<float>(v) / (img_h - 1) - 0.5f) * CH_PI);
        
        // Calculate the CDF for longitude (u) sampling at this latitude (v)
        for (int u = 0; u < img_w; ++u) {
            unsigned char red   = img.data[num_chs * ((img_h - v - 1) * img_w + u) + 0];
            unsigned char green = img.data[num_chs * ((img_h - v - 1) * img_w + u) + 1];
            unsigned char blue  = img.data[num_chs * ((img_h - v - 1) * img_w + u) + 2];
            
            // Convert RGB to luminance using Rec. 709 luma coefficients
            float luminance = (static_cast<float>(red) / 255.f * 0.2126f +
                               static_cast<float>(green) / 255.f * 0.7152f +
                               static_cast<float>(blue) / 255.f * 0.0722f);
            if (luminance > max_luminance) max_luminance = luminance;
            if (luminance < min_luminance) min_luminance = luminance;
            if (u == 0) {
                cdf_lon_float[v * img_w + u] = luminance;
            }
            else {
                cdf_lon_float[v * img_w + u] = cdf_lon_float[v * img_w + u - 1] + luminance;
            }
        }

        // Calculate the marginal CDF for latitude sampling at this latitude (v)
        if (v == 0) {
            cdf_lat_float[v] = cos_elev * cdf_lon_float[v * img_w + img_w - 1];
        }
        else {
            cdf_lat_float[v] = cdf_lat_float[v - 1] + cos_elev * cdf_lon_float[v * img_w + img_w - 1];
        }

        // Normalize the longitude CDF by the marginal CDF at this latitude (v) to [0, 65535]-scale
        for (int u = 0; u < img_w; ++u) {
            float denom = cdf_lon_float[v * img_w + img_w - 1];
            if (denom > 0.f) {
                host_cdf_lon[v * img_w + u] = static_cast<unsigned short>(
                    cdf_lon_float[v * img_w + u] / denom * 65535.f
                );
            }
            else {
                host_cdf_lon[v * img_w + u] = 0;
            }
            // std::cout << std::setw(6) << host_cdf_lon[v * img_w + u]; // debug    
        }
        // printf("\n"); // debug
        // printf("last value in host_cdf_lon: %d\n", host_cdf_lon[v * img_w + img_w - 1]); // debug
    }

    printf("\nMax luminance: %f, Min luminance: %f\n\n", max_luminance, min_luminance);
    
    // Normalize the latitude CDF to [0, 65535]-scale
    for (int v = 0; v < img_h; ++v) {
        float denom = cdf_lat_float[img_h - 1];
        if (denom > 0.f) {
            host_cdf_lat[v] = static_cast<unsigned short>(cdf_lat_float[v] / denom * 65535.f);
        }
        else {
            host_cdf_lat[v] = 0;
        }
        // std::cout << std::setw(6) << host_cdf_lat[v]; // debug
    }
    // printf("\n"); // debug

    // Copy CDFs to device memory
    CUDA_ERROR_CHECK(cudaMalloc((void**)&env_light_data.dev_cdf_lat, sizeof(unsigned short) * img_h));
    CUDA_ERROR_CHECK(cudaMalloc((void**)&env_light_data.dev_cdf_lon, sizeof(unsigned short) * img_w * img_h));
    CUDA_ERROR_CHECK(cudaMemcpy(
        env_light_data.dev_cdf_lat, host_cdf_lat.data(), sizeof(unsigned short) * img_h, cudaMemcpyHostToDevice
    ));
    CUDA_ERROR_CHECK(cudaMemcpy(
        env_light_data.dev_cdf_lon, host_cdf_lon.data(), sizeof(unsigned short) * img_w * img_h, cudaMemcpyHostToDevice
    ));
}

CH_SENSOR_API unsigned int ChScene::AddEnvironmentLight(std::string env_tex_path, float intensity_scale) {
    ChOptixLight light{};   // zero-initialize everything
    light.light_type  = LightType::ENVIRONMENT_LIGHT;
    light.pos = {0.f, 0.f, 0.f};
    light.delta = false;
    light.specific.environment.intensity_scale = intensity_scale;

    // Calculate extended parameters
    // Note: the environment map sampler will be set in ChOptixPipeline when processing the light data
    ByteImageData img = LoadByteImage(env_tex_path);
    printf("Environment light texture loaded, with resolution %d x %d\n", img.w, img.h);

    // Extended parameters
    light.specific.environment.width = img.w;
    light.specific.environment.height = img.h;

    BuildCDFOnDevice(img, light.specific.environment);

    m_lights.push_back(light);
    lights_changed = true;

    return static_cast<unsigned int>(m_lights.size() - 1);
}

//// ---- Register Your Customized Light Types Here (AddLight and ModifyLight functions) ---- ////


/// @brief Interpolate between two vectors
/// @param a the first vector
/// @param b the second vector
/// @param x the interpolation factor, between 0 and 1
/// @return the interpolated vector
ChVector3f ChLerp(ChVector3f& a, ChVector3f& b, double x) {
    return a + (b - a) * x;
}

CH_SENSOR_API void ChScene::SetBackground(Background b) {
    m_background = b;
    background_changed = true;
}

CH_SENSOR_API void ChScene::SetSceneEpsilon(float epsilon) {
    m_scene_epsilon = epsilon;
    background_changed = true;
}

/// Function to set the fog color
CH_SENSOR_API void ChScene::SetFogColor(ChVector3f color) {
    m_fog_color = ChClamp(color, ChVector3f(0.f, 0.f, 0.f), ChVector3f(1.f, 1.f, 1.f));
    background_changed = true;
}

/// Function to set the fog scattering coefficient
CH_SENSOR_API void ChScene::SetFogScattering(float coefficient) {
    m_fog_scattering = ChClamp(coefficient, 0.f, 1.f);
    background_changed = true;
}

/// Function to set the fog scattering coefficient
CH_SENSOR_API void ChScene::SetFogScatteringFromDistance(float distance) {
    distance = ChClamp(distance, 1e-3f, 1e16f);
    m_fog_scattering = log(256.0) / distance;
    background_changed = true;
}

/// @brief Add a sprite (a Chrono body that is only used for visualization and does not affect the physics simulation) to the scene
/// @param sprite the Chrono body to be added as a sprite in the scene
CH_SENSOR_API void ChScene::AddSprite(std::shared_ptr<ChBody> sprite) {
    m_sprites.push_back(sprite);
}

void ChScene::UpdateOriginOffset(ChVector3f sensor_pos, bool force) {
    if (force || (m_dynamic_origin_offset && (sensor_pos - m_origin_offset).Length() > m_dynamic_origin_threshold)) {
        // set the new origin offset
        m_origin_offset = sensor_pos;
        m_origin_changed = true;
    }
}

}  // namespace sensor
}  // namespace chrono
