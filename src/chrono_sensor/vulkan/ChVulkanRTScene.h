// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Florian Reinle
// =============================================================================
// Vulkan RT scene staging layer.
// =============================================================================

#ifndef CH_VULKAN_RT_SCENE_H
#define CH_VULKAN_RT_SCENE_H

#include <cstdint>
#include <string>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChSensorRenderTypes.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_vulkan
/// @{

struct CH_SENSOR_API ChVulkanRTSceneStats {
    uint32_t bodies = 0;
    uint32_t other_items = 0;
    uint32_t visible_shapes = 0;
    uint32_t boxes = 0;
    uint32_t spheres = 0;
    uint32_t cylinders = 0;
    uint32_t triangle_meshes = 0;
    uint32_t unsupported_shapes = 0;
};

enum class ChVulkanRTPrimitiveType { BOX, SPHERE, CYLINDER, TRIANGLE_MESH, MESH_PROXY };

struct CH_SENSOR_API ChVulkanRTTexCoord {
    float u = 0.f;
    float v = 0.f;
};

struct CH_SENSOR_API ChVulkanRTMaterial {
    ChVector3f diffuse = ChVector3f(0.75f, 0.75f, 0.75f);
    ChVector3f ambient = ChVector3f(0.75f, 0.75f, 0.75f);
    ChVector3f specular = ChVector3f(0.2f, 0.2f, 0.2f);
    ChVector3f emissive = ChVector3f(0.f, 0.f, 0.f);

    // Mirrors the fields consumed by the OptiX camera shaders. OptiX names this
    // value "transparency", but it is used as opacity/surface weight: 1 is
    // opaque, 0 is fully transparent and traces through the surface.
    float opacity = 1.f;
    float roughness = 1.f;
    float metallic = 0.f;
    float emissive_power = 0.f;
    float shininess = 32.f;
    bool use_specular_workflow = false;

    // Non-camera sensor response parameters. OptiX keeps these in its material
    // record; mirror them here so LiDAR/Radar parity does not depend on OptiX.
    float lidar_intensity = 1.f;
    float radar_backscatter = 1.f;

    float tex_scale_u = 1.f;
    float tex_scale_v = 1.f;
    std::string diffuse_texture;
    std::string specular_texture;
    std::string emissive_texture;
    std::string normal_texture;
    std::string roughness_texture;
    std::string metallic_texture;
    std::string opacity_texture;
    std::string weight_texture;

    unsigned short int class_id = 0;
    unsigned short int instance_id = 0;
};

struct CH_SENSOR_API ChVulkanRTTriangle {
    ChVector3d v0 = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d v1 = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d v2 = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d normal = ChVector3d(0.0, 0.0, 1.0);
    ChVector3d n0 = ChVector3d(0.0, 0.0, 1.0);
    ChVector3d n1 = ChVector3d(0.0, 0.0, 1.0);
    ChVector3d n2 = ChVector3d(0.0, 0.0, 1.0);
    ChVulkanRTTexCoord uv0;
    ChVulkanRTTexCoord uv1;
    ChVulkanRTTexCoord uv2;
    ChVector3d tangent = ChVector3d(1.0, 0.0, 0.0);
    bool has_vertex_normals = false;
    bool has_uvs = false;
    ChVulkanRTMaterial material;
};

struct CH_SENSOR_API ChVulkanRTPrimitive {
    ChVulkanRTPrimitiveType type = ChVulkanRTPrimitiveType::BOX;
    ChFrame<double> frame;
    ChVector3d scale = ChVector3d(1.0, 1.0, 1.0);
    ChVulkanRTMaterial material;

    // Triangle-mesh data in primitive-local coordinates. This mirrors the OptiX
    // triangle GAS input: vertices are kept untransformed except for the visual
    // shape scale; the body/asset transform remains in frame.
    std::vector<ChVulkanRTTriangle> triangles;
    ChVector3d aabb_min = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d aabb_max = ChVector3d(0.0, 0.0, 0.0);
    bool has_aabb = false;
    bool backface_cull = false;

    // Body motion staged for radar Doppler/velocity returns.  Static/non-body
    // items stay at zero, matching the OptiX convention for stationary hits.
    ChVector3d translational_velocity = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d angular_velocity = ChVector3d(0.0, 0.0, 0.0);
    float object_id = 0.f;
};

struct CH_SENSOR_API ChVulkanRTLight {
    LightType type = LightType::POINT_LIGHT;
    ChVector3f pos = ChVector3f(0.f, 0.f, 0.f);
    ChVector3f dir = ChVector3f(0.f, 0.f, -1.f);
    ChVector3f color = ChVector3f(1.f, 1.f, 1.f);
    float range = 100.f;
    float angle = 0.f;
    bool const_color = true;
    float atten_scale = 1.f;
    float angle_falloff_start = 0.f;
    float angle_atten_rate = -1.f;

    // Area-light parameters. Vulkan RT currently evaluates area lights from
    // their center point, but keeping the full public data here preserves the
    // OptiX ChScene API and allows a renderer upgrade without API changes.
    ChVector3f length_vec = ChVector3f(0.f, 0.f, 0.f);
    ChVector3f width_vec = ChVector3f(0.f, 0.f, 0.f);
    float radius = 0.f;
    float area = 0.f;

    std::string texture;
};

/// Staging scene for the Vulkan backend.
///
/// This object mirrors the public ChScene methods used by existing Sensor demos
/// while collecting a compact renderable representation. The representation is
/// intentionally independent from OptiX/CUDA and can feed either the Vulkan RT
/// BLAS/TLAS builder or the host fallback used for bring-up/testing.
class CH_SENSOR_API ChVulkanRTScene {
  public:
    ChVulkanRTScene() = default;

    void SyncFromSystem(ChSystem* system);

    const ChVulkanRTSceneStats& GetStats() const { return m_stats; }
    uint64_t GetRevision() const { return m_revision; }
    const std::vector<ChVulkanRTPrimitive>& GetPrimitives() const { return m_primitives; }

    void SetAmbientLight(const ChVector3f& color);
    const ChVector3f& GetAmbientLight() const { return m_ambient_light; }

    void SetBackground(const Background& background);
    const Background& GetBackground() const { return m_background; }

    unsigned int AddPointLight(ChVector3f pos, ChColor color, float max_range, bool const_color = true);

    unsigned int AddDirectionalLight(const ChVector3f& dir, const ChVector3f& color);
    unsigned int AddDirectionalLight(ChColor color, float elevation, float azimuth);

    unsigned int AddSpotLight(const ChVector3f& pos, const ChVector3f& dir, const ChVector3f& color, float range, float angle);
    unsigned int AddSpotLight(ChVector3f pos,
                              ChColor color,
                              float max_range,
                              ChVector3f light_dir,
                              float angle_falloff_start,
                              float angle_range,
                              bool const_color = true);

    unsigned int AddRectangleLight(ChVector3f pos,
                                   ChColor color,
                                   float max_range,
                                   ChVector3f length_vec,
                                   ChVector3f width_vec,
                                   bool const_color = true);

    unsigned int AddDiskLight(ChVector3f pos,
                              ChColor color,
                              float max_range,
                              ChVector3f light_dir,
                              float radius,
                              bool const_color = true);

    unsigned int AddEnvironmentLight(const std::string& env_tex, const ChVector3f& color = ChVector3f(1.f, 1.f, 1.f));
    unsigned int AddEnvironmentLight(std::string env_tex_path, float intensity_scale);

    void SetLights(const std::vector<ChVulkanRTLight>& lights);
    void ClearLights() { if (!m_lights.empty()) { m_lights.clear(); Touch(); } }
    const std::vector<ChVulkanRTLight>& GetLights() const { return m_lights; }

  private:
    void Touch() { ++m_revision; }

    ChVulkanRTMaterial ExtractMaterial(const std::shared_ptr<ChVisualShape>& shape) const;

    ChVulkanRTSceneStats m_stats;
    std::vector<ChVulkanRTPrimitive> m_primitives;
    std::vector<ChVulkanRTLight> m_lights;
    ChVector3f m_ambient_light = ChVector3f(0.1f, 0.1f, 0.1f);
    Background m_background;
    uint64_t m_revision = 1;
    uint64_t m_system_signature = 0;
};

/// @} sensor_vulkan

}  // namespace sensor
}  // namespace chrono

#endif
