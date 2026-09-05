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
// Authors: Kyle Sha
// =============================================================================
//
// Backend-agnostic description of a scene to render: geometry, instances and a camera,
// with no GPU, no OS and no rendering API in sight. ChMetalSceneBuilder fills these in
// from a ChSystem and ChMetalRTRenderer consumes them.
//
// =============================================================================

#ifndef CH_METAL_RENDER_TYPES_H
#define CH_METAL_RENDER_TYPES_H

#include <cstdint>
#include <string>
#include <vector>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

/// One unique geometry in object space, shared by every instance that references it.
///
/// Stored as flat per-triangle arrays rather than as indexed vertices with a material struct,
/// because that is the shape the Metal ray-tracing pipeline consumes: each array becomes one
/// device buffer, and a shader reads element i for triangle i with no indirection.
struct MetalGeometry {
    std::vector<float> verts;           ///< 9 floats / triangle (3 xyz)
    std::vector<float> normals;         ///< 9 floats / triangle (3 vertex normals)
    std::vector<float> tangents;        ///< 9 floats / triangle (object-space tangent, for normal mapping)
    std::vector<float> uv;              ///< 6 floats / triangle (3 uv); zeros if none
    std::vector<float> colors;          ///< 3 floats / triangle (base albedo)
    std::vector<float> opacity;         ///< 1 float / triangle: material opacity d (1 = opaque)
    std::vector<float> roughness;       ///< 1 float / triangle: material roughness (1 = matte, low = glossy)
    std::vector<float> metallic;        ///< 1 float / triangle: material metallic (0 = dielectric, 1 = metal)
    std::vector<int> texId;             ///< 1 int / triangle: index into MetalRenderScene.texturePaths, or -1
    std::vector<int> roughTexId;        ///< 1 int / triangle: roughness map (map_Pr) index, or -1
    std::vector<int> metalTexId;        ///< 1 int / triangle: metallic  map (map_Pm) index, or -1
    std::vector<int> opacityTexId;      ///< 1 int / triangle: opacity map (map_d) index, or -1
    std::vector<int> normalTexId;       ///< 1 int / triangle: normal map (norm/map_Bump) index, or -1
    std::vector<float> specular;        ///< 4 floats / triangle: Ks.rgb + use_specular_workflow flag (0/1)
    std::vector<float> emissive;        ///< 4 floats / triangle: Ke.rgb + emissive_power
    std::vector<float> texScale;        ///< 2 floats / triangle: texture UV scale (default 1,1)
    std::vector<int> ksTexId;           ///< 1 int / triangle: specular map (map_Ks) index, or -1
    std::vector<int> keTexId;           ///< 1 int / triangle: emissive map (map_Ke) index, or -1
    std::vector<int> blendKdTexId;      ///< weight-blended materials: 2nd-layer Kd map index, or -1 (no blend)
    std::vector<int> blendWeightTexId;  ///< weight-blended materials: blend weight map index, or -1
    bool dynamic = false;               ///< deforming (re-extracted + refit every frame)
    /// Number of triangles, which every per-triangle array above is sized against.
    int triCount() const { return (int)(verts.size() / 9); }
};

/// One placement of a geometry in the world.
struct MetalInstance {
    int geom = 0;     ///< index into MetalRenderScene.geometries
    float xform[12];  ///< object->world 4x3, column-major: col0,col1,col2 (basis), col3 (translation)
    float rot[9];     ///< rotation (basis columns) used to transform normals to world
    float tint[3] = {1, 1, 1};
    uint32_t mat = 0;          ///< material class (always 0 = normal material-shaded mesh; reserved)
    uint32_t classId = 0;      ///< semantic class id (from ChVisualMaterial)
    uint32_t instanceId = 0;   ///< semantic instance id (from ChVisualMaterial)
    float vel[3] = {0, 0, 0};  ///< world linear velocity (for radar Doppler)
};

/// A whole scene, as handed to the renderer.
struct MetalRenderScene {
    std::vector<MetalGeometry> geometries;
    std::vector<MetalInstance> instances;
    std::vector<std::string> texturePaths;  ///< texId values index into this list
};

/// Where the camera is looking.
/// Orbit state belongs to the interactive window; this is only the resolved target.
struct MetalSceneCamera {
    float target[3] = {0, 0, 0};
    float fovDeg = 50.0f;
};

/// @} sensor_metal

}  // namespace sensor
}  // namespace chrono

#endif
