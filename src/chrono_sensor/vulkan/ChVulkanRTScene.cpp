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

#include "chrono_sensor/vulkan/ChVulkanRTScene.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>
#include <unordered_map>

#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"

namespace chrono {
namespace sensor {

namespace {

constexpr double CH_VKRT_SCENE_EPS = 1e-12;
constexpr float CH_VKRT_PI = 3.14159265358979323846f;

ChVector3f Cross(const ChVector3f& a, const ChVector3f& b) {
    return ChVector3f(a.y() * b.z() - a.z() * b.y(),
                      a.z() * b.x() - a.x() * b.z(),
                      a.x() * b.y() - a.y() * b.x());
}

ChVector3f NormalizeOrDefault(const ChVector3f& v, const ChVector3f& fallback) {
    const float len = v.Length();
    return (len > 1e-12f) ? (v / len) : fallback;
}

ChVector3d ScaleVertex(const ChVector3d& v, const ChVector3d& scale) {
    return ChVector3d(v.x() * scale.x(), v.y() * scale.y(), v.z() * scale.z());
}

ChVulkanRTMaterial MaterialFromVisual(const std::shared_ptr<ChVisualMaterial>& visual_mat, const ChVulkanRTMaterial& fallback) {
    if (!visual_mat)
        return fallback;

    ChVulkanRTMaterial mat = fallback;
    const auto& kd = visual_mat->GetDiffuseColor();
    const auto& ka = visual_mat->GetAmbientColor();
    const auto& ks = visual_mat->GetSpecularColor();
    const auto& ke = visual_mat->GetEmissiveColor();
    mat.diffuse = ChVector3f(kd.R, kd.G, kd.B);
    mat.ambient = ChVector3f(ka.R, ka.G, ka.B);
    mat.specular = ChVector3f(ks.R, ks.G, ks.B);
    mat.emissive = ChVector3f(ke.R, ke.G, ke.B);
    mat.opacity = std::max(0.f, std::min(1.f, visual_mat->GetOpacity()));
    mat.roughness = std::max(0.02f, std::min(1.f, visual_mat->GetRoughness()));
    mat.metallic = std::max(0.f, std::min(1.f, visual_mat->GetMetallic()));
    mat.emissive_power = std::max(0.f, visual_mat->GetEmissivePower());
    mat.shininess = std::max(1.f, visual_mat->GetSpecularExponent());
    mat.use_specular_workflow = visual_mat->GetUseSpecularWorkflow();
    mat.tex_scale_u = visual_mat->GetTextureScale().x();
    mat.tex_scale_v = visual_mat->GetTextureScale().y();
    mat.diffuse_texture = visual_mat->GetKdTexture();
    mat.specular_texture = visual_mat->GetKsTexture();
    mat.emissive_texture.clear();
    mat.normal_texture = visual_mat->GetNormalMapTexture();
    mat.roughness_texture = visual_mat->GetRoughnessTexture();
    mat.metallic_texture = visual_mat->GetMetallicTexture();
    mat.opacity_texture = visual_mat->GetOpacityTexture();
    mat.weight_texture = visual_mat->GetWeightTexture();
    mat.class_id = visual_mat->GetClassID();
    mat.instance_id = visual_mat->GetInstanceID();
    return mat;
}

bool FileExists(const std::string& path) {
    if (path.empty())
        return false;
    std::ifstream file(path.c_str(), std::ios::binary);
    return file.good();
}

bool IsAbsolutePath(const std::string& path) {
    return !path.empty() &&
           (path[0] == '/' || path[0] == '\\' || (path.size() > 1 && path[1] == ':'));
}

std::string DirectoryOf(const std::string& path) {
    const auto pos = path.find_last_of("/\\");
    if (pos == std::string::npos)
        return std::string();
    return path.substr(0, pos + 1);
}

void ResolveTexturePath(std::string& texture, const std::string& base_dir) {
    if (texture.empty() || base_dir.empty() || IsAbsolutePath(texture) || FileExists(texture))
        return;

    const std::string candidate = base_dir + texture;
    if (FileExists(candidate))
        texture = candidate;
}

void ResolveMaterialTexturePaths(ChVulkanRTMaterial& mat, const std::string& base_dir) {
    ResolveTexturePath(mat.diffuse_texture, base_dir);
    ResolveTexturePath(mat.specular_texture, base_dir);
    ResolveTexturePath(mat.emissive_texture, base_dir);
    ResolveTexturePath(mat.normal_texture, base_dir);
    ResolveTexturePath(mat.roughness_texture, base_dir);
    ResolveTexturePath(mat.metallic_texture, base_dir);
    ResolveTexturePath(mat.opacity_texture, base_dir);
    ResolveTexturePath(mat.weight_texture, base_dir);
}

void ResolveMaterialTexturePaths(std::vector<ChVulkanRTMaterial>& materials, const std::string& base_dir) {
    for (auto& material : materials)
        ResolveMaterialTexturePaths(material, base_dir);
}

std::vector<ChVulkanRTMaterial> BuildMaterialTable(const std::shared_ptr<ChVisualShape>& shape, const ChVulkanRTMaterial& fallback) {
    std::vector<ChVulkanRTMaterial> materials;
    if (!shape)
        return materials;

    materials.reserve(shape->GetNumMaterials());
    for (unsigned int i = 0; i < shape->GetNumMaterials(); ++i)
        materials.push_back(MaterialFromVisual(shape->GetMaterial(static_cast<int>(i)), fallback));
    return materials;
}

void ExpandAABB(ChVulkanRTPrimitive& primitive, const ChVector3d& p) {
    if (!primitive.has_aabb) {
        primitive.aabb_min = p;
        primitive.aabb_max = p;
        primitive.has_aabb = true;
        return;
    }

    primitive.aabb_min = ChVector3d(std::min(primitive.aabb_min.x(), p.x()),
                                    std::min(primitive.aabb_min.y(), p.y()),
                                    std::min(primitive.aabb_min.z(), p.z()));
    primitive.aabb_max = ChVector3d(std::max(primitive.aabb_max.x(), p.x()),
                                    std::max(primitive.aabb_max.y(), p.y()),
                                    std::max(primitive.aabb_max.z(), p.z()));
}


std::shared_ptr<ChVisualShapeTriangleMesh> LoadModelShapeCached(const std::string& filename) {
    static std::unordered_map<std::string, std::shared_ptr<ChVisualShapeTriangleMesh>> cache;
    const auto found = cache.find(filename);
    if (found != cache.end())
        return found->second;

    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    auto shape = std::make_shared<ChVisualShapeTriangleMesh>();
    if (mesh)
        shape->SetMesh(mesh);
    cache.emplace(filename, shape);
    return shape;
}

bool FillTrianglePrimitive(ChVulkanRTPrimitive& primitive,
                           const std::shared_ptr<ChTriangleMeshConnected>& mesh,
                           const ChVector3d& scale,
                           bool backface_cull,
                           const std::vector<ChVulkanRTMaterial>& materials) {
    if (!mesh)
        return false;

    const auto& vertices = mesh->GetCoordsVertices();
    const auto& faces = mesh->GetIndicesVertices();
    const auto& normals = mesh->GetCoordsNormals();
    const auto& normal_indices = mesh->GetIndicesNormals();
    const auto& uvs = mesh->GetCoordsUV();
    const auto& uv_indices = mesh->GetIndicesUV();
    const auto& face_materials = mesh->GetIndicesMaterials();
    if (vertices.empty() || faces.empty())
        return false;

    primitive.type = ChVulkanRTPrimitiveType::TRIANGLE_MESH;
    primitive.scale = scale;
    primitive.backface_cull = backface_cull;
    primitive.triangles.clear();
    primitive.triangles.reserve(faces.size());
    primitive.has_aabb = false;

    auto valid_tri_index = [](int i, size_t count) { return i >= 0 && static_cast<size_t>(i) < count; };
    auto get_uv = [&](int idx) {
        ChVulkanRTTexCoord out;
        if (valid_tri_index(idx, uvs.size())) {
            out.u = static_cast<float>(uvs[static_cast<size_t>(idx)].x());
            out.v = static_cast<float>(uvs[static_cast<size_t>(idx)].y());
        }
        return out;
    };
    auto get_normal = [&](int idx, const ChVector3d& fallback) {
        if (!valid_tri_index(idx, normals.size()))
            return fallback;
        ChVector3d n = normals[static_cast<size_t>(idx)];
        const double len = n.Length();
        return len > CH_VKRT_SCENE_EPS ? n / len : fallback;
    };

    for (size_t face_id = 0; face_id < faces.size(); ++face_id) {
        const auto& face = faces[face_id];
        const int i0 = face.x();
        const int i1 = face.y();
        const int i2 = face.z();
        if (!valid_tri_index(i0, vertices.size()) || !valid_tri_index(i1, vertices.size()) || !valid_tri_index(i2, vertices.size()))
            continue;

        ChVulkanRTTriangle tri;
        tri.v0 = ScaleVertex(vertices[static_cast<size_t>(i0)], scale);
        tri.v1 = ScaleVertex(vertices[static_cast<size_t>(i1)], scale);
        tri.v2 = ScaleVertex(vertices[static_cast<size_t>(i2)], scale);

        const ChVector3d e1 = tri.v1 - tri.v0;
        const ChVector3d e2 = tri.v2 - tri.v0;
        tri.normal = e1.Cross(e2);
        const double n_len = tri.normal.Length();
        if (n_len < CH_VKRT_SCENE_EPS)
            continue;
        tri.normal /= n_len;
        tri.n0 = tri.normal;
        tri.n1 = tri.normal;
        tri.n2 = tri.normal;
        tri.tangent = e1.Length() > CH_VKRT_SCENE_EPS ? e1 / e1.Length() : ChVector3d(1.0, 0.0, 0.0);

        if (face_id < normal_indices.size() && !normals.empty()) {
            const auto& ni = normal_indices[face_id];
            tri.n0 = get_normal(ni.x(), tri.normal);
            tri.n1 = get_normal(ni.y(), tri.normal);
            tri.n2 = get_normal(ni.z(), tri.normal);
            tri.has_vertex_normals = true;
        }

        if (!uvs.empty()) {
            int ui0 = i0;
            int ui1 = i1;
            int ui2 = i2;
            if (face_id < uv_indices.size()) {
                const auto& ui = uv_indices[face_id];
                ui0 = ui.x();
                ui1 = ui.y();
                ui2 = ui.z();
            }
            if (valid_tri_index(ui0, uvs.size()) && valid_tri_index(ui1, uvs.size()) && valid_tri_index(ui2, uvs.size())) {
                tri.uv0 = get_uv(ui0);
                tri.uv1 = get_uv(ui1);
                tri.uv2 = get_uv(ui2);
                tri.has_uvs = true;

                const ChVector3d delta_pos1 = tri.v1 - tri.v0;
                const ChVector3d delta_pos2 = tri.v2 - tri.v0;
                const float du1 = tri.uv1.u - tri.uv0.u;
                const float dv1 = tri.uv1.v - tri.uv0.v;
                const float du2 = tri.uv2.u - tri.uv0.u;
                const float dv2 = tri.uv2.v - tri.uv0.v;
                const float det = du1 * dv2 - du2 * dv1;
                if (std::abs(det) > static_cast<float>(CH_VKRT_SCENE_EPS)) {
                    const double inv_det = 1.0 / static_cast<double>(det);
                    ChVector3d tangent = inv_det * (static_cast<double>(dv2) * delta_pos1 - static_cast<double>(dv1) * delta_pos2);
                    const double t_len = tangent.Length();
                    if (t_len > CH_VKRT_SCENE_EPS)
                        tri.tangent = tangent / t_len;
                }
            }
        }

        tri.material = primitive.material;
        if (face_id < face_materials.size()) {
            const int mat_id = face_materials[face_id];
            if (mat_id >= 0 && static_cast<size_t>(mat_id) < materials.size())
                tri.material = materials[static_cast<size_t>(mat_id)];
        }

        ExpandAABB(primitive, tri.v0);
        ExpandAABB(primitive, tri.v1);
        ExpandAABB(primitive, tri.v2);
        primitive.triangles.push_back(tri);
    }

    return !primitive.triangles.empty();
}

bool AlmostEqual(double a, double b, double eps = 1e-8) {
    return std::abs(a - b) <= eps;
}

bool AlmostEqual(float a, float b, float eps = 1e-6f) {
    return std::abs(a - b) <= eps;
}

bool SameVec3d(const ChVector3d& a, const ChVector3d& b) {
    return AlmostEqual(a.x(), b.x()) && AlmostEqual(a.y(), b.y()) && AlmostEqual(a.z(), b.z());
}

bool SameVec3f(const ChVector3f& a, const ChVector3f& b) {
    return AlmostEqual(a.x(), b.x()) && AlmostEqual(a.y(), b.y()) && AlmostEqual(a.z(), b.z());
}

bool SameTexCoord(const ChVulkanRTTexCoord& a, const ChVulkanRTTexCoord& b) {
    return AlmostEqual(a.u, b.u) && AlmostEqual(a.v, b.v);
}

bool SameFrame(const ChFrame<double>& a, const ChFrame<double>& b) {
    const auto& qa = a.GetRot();
    const auto& qb = b.GetRot();
    return SameVec3d(a.GetPos(), b.GetPos()) &&
           AlmostEqual(qa.e0(), qb.e0()) && AlmostEqual(qa.e1(), qb.e1()) &&
           AlmostEqual(qa.e2(), qb.e2()) && AlmostEqual(qa.e3(), qb.e3());
}

bool SameMaterial(const ChVulkanRTMaterial& a, const ChVulkanRTMaterial& b) {
    return SameVec3f(a.diffuse, b.diffuse) && SameVec3f(a.ambient, b.ambient) &&
           SameVec3f(a.specular, b.specular) && SameVec3f(a.emissive, b.emissive) &&
           AlmostEqual(a.opacity, b.opacity) && AlmostEqual(a.roughness, b.roughness) &&
           AlmostEqual(a.metallic, b.metallic) && AlmostEqual(a.emissive_power, b.emissive_power) &&
           AlmostEqual(a.shininess, b.shininess) && a.use_specular_workflow == b.use_specular_workflow &&
           AlmostEqual(a.lidar_intensity, b.lidar_intensity) && AlmostEqual(a.radar_backscatter, b.radar_backscatter) &&
           AlmostEqual(a.tex_scale_u, b.tex_scale_u) && AlmostEqual(a.tex_scale_v, b.tex_scale_v) &&
           a.diffuse_texture == b.diffuse_texture && a.specular_texture == b.specular_texture &&
           a.emissive_texture == b.emissive_texture && a.normal_texture == b.normal_texture &&
           a.roughness_texture == b.roughness_texture && a.metallic_texture == b.metallic_texture &&
           a.opacity_texture == b.opacity_texture && a.weight_texture == b.weight_texture &&
           a.class_id == b.class_id && a.instance_id == b.instance_id;
}

bool SameBackground(const Background& a, const Background& b) {
    return a.mode == b.mode && SameVec3f(a.color_zenith, b.color_zenith) &&
           SameVec3f(a.color_horizon, b.color_horizon) && a.env_tex == b.env_tex;
}

bool SameLight(const ChVulkanRTLight& a, const ChVulkanRTLight& b) {
    return a.type == b.type && SameVec3f(a.pos, b.pos) && SameVec3f(a.dir, b.dir) &&
           SameVec3f(a.color, b.color) && AlmostEqual(a.range, b.range) &&
           AlmostEqual(a.angle, b.angle) && a.const_color == b.const_color &&
           AlmostEqual(a.atten_scale, b.atten_scale) &&
           AlmostEqual(a.angle_falloff_start, b.angle_falloff_start) &&
           AlmostEqual(a.angle_atten_rate, b.angle_atten_rate) &&
           SameVec3f(a.length_vec, b.length_vec) && SameVec3f(a.width_vec, b.width_vec) &&
           AlmostEqual(a.radius, b.radius) && AlmostEqual(a.area, b.area) && a.texture == b.texture;
}

bool SameLights(const std::vector<ChVulkanRTLight>& a, const std::vector<ChVulkanRTLight>& b) {
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (!SameLight(a[i], b[i]))
            return false;
    }
    return true;
}

bool SameTriangle(const ChVulkanRTTriangle& a, const ChVulkanRTTriangle& b) {
    return SameVec3d(a.v0, b.v0) && SameVec3d(a.v1, b.v1) && SameVec3d(a.v2, b.v2) &&
           SameVec3d(a.normal, b.normal) && SameVec3d(a.n0, b.n0) && SameVec3d(a.n1, b.n1) &&
           SameVec3d(a.n2, b.n2) && SameTexCoord(a.uv0, b.uv0) && SameTexCoord(a.uv1, b.uv1) &&
           SameTexCoord(a.uv2, b.uv2) && SameVec3d(a.tangent, b.tangent) &&
           a.has_vertex_normals == b.has_vertex_normals && a.has_uvs == b.has_uvs &&
           SameMaterial(a.material, b.material);
}

bool SamePrimitive(const ChVulkanRTPrimitive& a, const ChVulkanRTPrimitive& b) {
    if (a.type != b.type || !SameFrame(a.frame, b.frame) || !SameVec3d(a.scale, b.scale) ||
        !SameMaterial(a.material, b.material) || !SameVec3d(a.aabb_min, b.aabb_min) ||
        !SameVec3d(a.aabb_max, b.aabb_max) || a.has_aabb != b.has_aabb || a.backface_cull != b.backface_cull ||
        !SameVec3d(a.translational_velocity, b.translational_velocity) ||
        !SameVec3d(a.angular_velocity, b.angular_velocity) || !AlmostEqual(a.object_id, b.object_id) ||
        a.triangles.size() != b.triangles.size()) {
        return false;
    }

    for (size_t i = 0; i < a.triangles.size(); ++i) {
        if (!SameTriangle(a.triangles[i], b.triangles[i]))
            return false;
    }
    return true;
}

bool SameStats(const ChVulkanRTSceneStats& a, const ChVulkanRTSceneStats& b) {
    return a.bodies == b.bodies && a.other_items == b.other_items && a.visible_shapes == b.visible_shapes &&
           a.boxes == b.boxes && a.spheres == b.spheres && a.cylinders == b.cylinders &&
           a.triangle_meshes == b.triangle_meshes && a.unsupported_shapes == b.unsupported_shapes;
}

bool SamePrimitiveList(const std::vector<ChVulkanRTPrimitive>& a, const std::vector<ChVulkanRTPrimitive>& b) {
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (!SamePrimitive(a[i], b[i]))
            return false;
    }
    return true;
}

uint64_t HashCombine(uint64_t seed, uint64_t value) {
    return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2));
}

uint64_t HashDouble(uint64_t seed, double v) {
    const auto q = static_cast<int64_t>(std::llround(v * 100000000.0));
    return HashCombine(seed, static_cast<uint64_t>(q));
}

uint64_t HashFloat(uint64_t seed, float v) {
    const auto q = static_cast<int64_t>(std::llround(static_cast<double>(v) * 1000000.0));
    return HashCombine(seed, static_cast<uint64_t>(q));
}

uint64_t HashVec3d(uint64_t seed, const ChVector3d& v) {
    seed = HashDouble(seed, v.x());
    seed = HashDouble(seed, v.y());
    seed = HashDouble(seed, v.z());
    return seed;
}

uint64_t HashVec3f(uint64_t seed, const ChVector3f& v) {
    seed = HashFloat(seed, v.x());
    seed = HashFloat(seed, v.y());
    seed = HashFloat(seed, v.z());
    return seed;
}

uint64_t HashString(uint64_t seed, const std::string& text) {
    return HashCombine(seed, static_cast<uint64_t>(std::hash<std::string>{}(text)));
}

uint64_t HashFrame(uint64_t seed, const ChFrame<double>& f) {
    seed = HashVec3d(seed, f.GetPos());
    const auto& q = f.GetRot();
    seed = HashDouble(seed, q.e0());
    seed = HashDouble(seed, q.e1());
    seed = HashDouble(seed, q.e2());
    seed = HashDouble(seed, q.e3());
    return seed;
}

uint64_t HashMaterial(uint64_t seed, const ChVulkanRTMaterial& mat) {
    seed = HashVec3f(seed, mat.diffuse);
    seed = HashVec3f(seed, mat.ambient);
    seed = HashVec3f(seed, mat.specular);
    seed = HashVec3f(seed, mat.emissive);
    seed = HashFloat(seed, mat.opacity);
    seed = HashFloat(seed, mat.roughness);
    seed = HashFloat(seed, mat.metallic);
    seed = HashFloat(seed, mat.emissive_power);
    seed = HashFloat(seed, mat.shininess);
    seed = HashCombine(seed, mat.use_specular_workflow ? 1u : 0u);
    seed = HashFloat(seed, mat.lidar_intensity);
    seed = HashFloat(seed, mat.radar_backscatter);
    seed = HashFloat(seed, mat.tex_scale_u);
    seed = HashFloat(seed, mat.tex_scale_v);
    seed = HashString(seed, mat.diffuse_texture);
    seed = HashString(seed, mat.specular_texture);
    seed = HashString(seed, mat.emissive_texture);
    seed = HashString(seed, mat.normal_texture);
    seed = HashString(seed, mat.roughness_texture);
    seed = HashString(seed, mat.metallic_texture);
    seed = HashString(seed, mat.opacity_texture);
    seed = HashString(seed, mat.weight_texture);
    seed = HashCombine(seed, mat.class_id);
    seed = HashCombine(seed, mat.instance_id);
    return seed;
}

uint64_t HashShapeMaterialState(uint64_t seed, const std::shared_ptr<ChVisualShape>& shape) {
    if (!shape)
        return HashCombine(seed, 0u);
    ChVulkanRTMaterial fallback;
    ChColor color = shape->GetColor();
    fallback.diffuse = ChVector3f(color.R, color.G, color.B);
    fallback.ambient = fallback.diffuse;
    seed = HashMaterial(seed, fallback);
    seed = HashCombine(seed, shape->GetNumMaterials());
    for (unsigned int i = 0; i < shape->GetNumMaterials(); ++i)
        seed = HashMaterial(seed, MaterialFromVisual(shape->GetMaterial(static_cast<int>(i)), fallback));
    return seed;
}

uint64_t HashVisualModelState(uint64_t seed,
                              const std::shared_ptr<ChVisualModel>& model,
                              const ChFrame<double>& parent_frame,
                              const ChVector3d& lin_vel,
                              const ChVector3d& ang_vel,
                              float object_id) {
    if (!model)
        return HashCombine(seed, 0u);

    seed = HashCombine(seed, reinterpret_cast<uintptr_t>(model.get()));
    seed = HashFrame(seed, parent_frame);
    seed = HashVec3d(seed, lin_vel);
    seed = HashVec3d(seed, ang_vel);
    seed = HashFloat(seed, object_id);
    seed = HashCombine(seed, model->GetShapeInstances().size());

    for (const auto& instance : model->GetShapeInstances()) {
        auto shape = instance.shape;
        seed = HashCombine(seed, reinterpret_cast<uintptr_t>(shape.get()));
        seed = HashFrame(seed, instance.frame);
        seed = HashCombine(seed, shape && shape->IsVisible() ? 1u : 0u);
        if (!shape)
            continue;

        seed = HashShapeMaterialState(seed, shape);
        if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
            seed = HashCombine(seed, 1u);
            seed = HashVec3d(seed, box->GetLengths());
        } else if (auto sphere = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
            seed = HashCombine(seed, 2u);
            seed = HashDouble(seed, sphere->GetRadius());
        } else if (auto cylinder = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
            seed = HashCombine(seed, 3u);
            seed = HashDouble(seed, cylinder->GetRadius());
            seed = HashDouble(seed, cylinder->GetHeight());
        } else if (auto trimesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
            seed = HashCombine(seed, 4u);
            seed = HashCombine(seed, reinterpret_cast<uintptr_t>(trimesh->GetMesh().get()));
            seed = HashVec3d(seed, trimesh->GetScale());
            seed = HashCombine(seed, trimesh->IsBackfaceCull() ? 1u : 0u);
            seed = HashCombine(seed, trimesh->IsWireframe() ? 1u : 0u);
        } else if (auto obj = std::dynamic_pointer_cast<ChVisualShapeModelFile>(shape)) {
            seed = HashCombine(seed, 5u);
            seed = HashString(seed, obj->GetFilename());
            seed = HashVec3d(seed, obj->GetScale());
        } else {
            seed = HashCombine(seed, 0xffffffffu);
        }
    }
    return seed;
}

uint64_t ComputeSystemSignature(ChSystem* system) {
    if (!system)
        return 0;

    uint64_t seed = 1469598103934665603ull;
    seed = HashCombine(seed, system->GetBodies().size());
    for (const auto& body : system->GetBodies()) {
        seed = HashCombine(seed, reinterpret_cast<uintptr_t>(body.get()));
        if (!body)
            continue;
        seed = HashVisualModelState(seed,
                                    body->GetVisualModel(),
                                    body->GetFrameRefToAbs(),
                                    body->GetPosDt(),
                                    body->GetAngVelParent(),
                                    static_cast<float>(body->GetIdentifier()));
    }

    seed = HashCombine(seed, system->GetOtherPhysicsItems().size());
    const ChFrame<double> identity_frame;
    const ChVector3d zero_velocity(0.0, 0.0, 0.0);
    for (const auto& item : system->GetOtherPhysicsItems()) {
        seed = HashCombine(seed, reinterpret_cast<uintptr_t>(item.get()));
        if (!item)
            continue;
        seed = HashVisualModelState(seed, item->GetVisualModel(), identity_frame, zero_velocity, zero_velocity, 0.f);
    }
    return seed;
}

}  // namespace

ChVulkanRTMaterial ChVulkanRTScene::ExtractMaterial(const std::shared_ptr<ChVisualShape>& shape) const {
    ChVulkanRTMaterial mat;
    if (!shape)
        return mat;

    ChColor color = shape->GetColor();
    mat.diffuse = ChVector3f(color.R, color.G, color.B);
    mat.ambient = mat.diffuse;

    if (shape->GetNumMaterials() > 0)
        mat = MaterialFromVisual(shape->GetMaterial(0), mat);

    return mat;
}

void ChVulkanRTScene::SetAmbientLight(const ChVector3f& color) {
    if (!SameVec3f(m_ambient_light, color)) {
        m_ambient_light = color;
        Touch();
    }
}

void ChVulkanRTScene::SetBackground(const Background& background) {
    if (!SameBackground(m_background, background)) {
        m_background = background;
        Touch();
    }
}

void ChVulkanRTScene::SetLights(const std::vector<ChVulkanRTLight>& lights) {
    if (!SameLights(m_lights, lights)) {
        m_lights = lights;
        Touch();
    }
}

void ChVulkanRTScene::SyncFromSystem(ChSystem* system) {
    ChVulkanRTSceneStats next_stats{};
    std::vector<ChVulkanRTPrimitive> next_primitives;

    if (!system) {
        if (!m_primitives.empty() || !SameStats(m_stats, next_stats) || m_system_signature != 0) {
            m_stats = next_stats;
            m_primitives.clear();
            m_system_signature = 0;
            Touch();
        }
        return;
    }

    const uint64_t next_signature = ComputeSystemSignature(system);
    if (next_signature == m_system_signature)
        return;

    auto stage_visual_model = [&](const std::shared_ptr<ChVisualModel>& model,
                                  const ChFrame<double>& body_frame,
                                  const ChVector3d& body_lin_vel,
                                  const ChVector3d& body_ang_vel,
                                  float object_id) {
        if (!model)
            return;

        for (const auto& instance : model->GetShapeInstances()) {
            auto shape = instance.shape;
            if (!shape || !shape->IsVisible())
                continue;

            ++next_stats.visible_shapes;

            ChVulkanRTPrimitive primitive;
            // Match OptiX geometry instancing: body reference frame times the per-shape
            // visual asset frame.  GetVisualModelFrame() is for sensor/camera parents;
            // using it here would double-apply translations for normal body visuals.
            primitive.frame = body_frame * instance.frame;
            primitive.material = ExtractMaterial(shape);
            primitive.translational_velocity = body_lin_vel;
            primitive.angular_velocity = body_ang_vel;
            primitive.object_id = object_id;

            if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
                primitive.type = ChVulkanRTPrimitiveType::BOX;
                primitive.scale = box->GetLengths();
                ++next_stats.boxes;
            } else if (auto sphere = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
                primitive.type = ChVulkanRTPrimitiveType::SPHERE;
                primitive.scale = ChVector3d(sphere->GetRadius(), sphere->GetRadius(), sphere->GetRadius());
                ++next_stats.spheres;
            } else if (auto cylinder = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
                primitive.type = ChVulkanRTPrimitiveType::CYLINDER;
                primitive.scale = ChVector3d(cylinder->GetRadius(), cylinder->GetRadius(), cylinder->GetHeight());
                ++next_stats.cylinders;
            } else if (auto trimesh_shape = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
                if (trimesh_shape->IsWireframe()) {
                    std::cerr << "WARNING: Chrono::Sensor Vulkan RT does not support wireframe meshes. Defaulting back to solid mesh, please check for visual issues.\n";
                }
                const auto materials = BuildMaterialTable(trimesh_shape, primitive.material);
                if (!FillTrianglePrimitive(primitive, trimesh_shape->GetMesh(), trimesh_shape->GetScale(), trimesh_shape->IsBackfaceCull(), materials)) {
                    ++next_stats.unsupported_shapes;
                    continue;
                }
                ++next_stats.triangle_meshes;
            } else if (auto obj = std::dynamic_pointer_cast<ChVisualShapeModelFile>(shape)) {
                auto obj_trimesh_shape = LoadModelShapeCached(obj->GetFilename());
                auto obj_trimesh = obj_trimesh_shape ? obj_trimesh_shape->GetMesh() : std::shared_ptr<ChTriangleMeshConnected>();
                // Match the OptiX model-file path: the OBJ/MTL import creates a
                // temporary triangle-mesh visual shape and the materials from that
                // imported mesh drive the material-index buffer.  Prefer those
                // imported materials; an optional material attached to the
                // ChVisualShapeModelFile is only a fallback override.
                auto materials = BuildMaterialTable(obj_trimesh_shape, primitive.material);
                if (materials.empty())
                    materials = BuildMaterialTable(obj, primitive.material);
                ResolveMaterialTexturePaths(materials, DirectoryOf(obj->GetFilename()));
                if (!FillTrianglePrimitive(primitive, obj_trimesh, obj->GetScale(), false, materials)) {
                    std::cerr << "WARNING: Chrono::Sensor Vulkan RT could not load visual model file: " << obj->GetFilename() << "\n";
                    ++next_stats.unsupported_shapes;
                    continue;
                }
                ++next_stats.triangle_meshes;
            } else {
                ++next_stats.unsupported_shapes;
                continue;
            }

            next_primitives.push_back(std::move(primitive));
        }
    };

    next_stats.bodies = static_cast<uint32_t>(system->GetBodies().size());
    for (const auto& body : system->GetBodies()) {
        if (!body)
            continue;
        stage_visual_model(body->GetVisualModel(),
                           body->GetFrameRefToAbs(),
                           body->GetPosDt(),
                           body->GetAngVelParent(),
                           static_cast<float>(body->GetIdentifier()));
    }

    // OptiX also stages visual models attached to ChSystem::GetOtherPhysicsItems().
    // The previous Vulkan staging pass silently skipped these, so terrain patches,
    // FEA/mesh items, and other non-body visuals could disappear in Vulkan while
    // being present in OptiX.  OptiX currently instances them through a dummy body
    // at identity, so mirror that convention here.
    next_stats.other_items = static_cast<uint32_t>(system->GetOtherPhysicsItems().size());
    const ChFrame<double> identity_frame;
    const ChVector3d zero_velocity(0.0, 0.0, 0.0);
    for (const auto& item : system->GetOtherPhysicsItems()) {
        if (!item)
            continue;
        stage_visual_model(item->GetVisualModel(), identity_frame, zero_velocity, zero_velocity, 0.f);
    }

    // Camera motion does not change renderable geometry.  Do not invalidate the
    // Vulkan scene just because the sensor manager calls SyncFromSystem() every
    // update; otherwise every camera frame rebuilds BLAS/TLAS and reuploads the
    // entire scene, which dominates runtime and hides all GPU parallelism.
    if (!SameStats(m_stats, next_stats) || !SamePrimitiveList(m_primitives, next_primitives)) {
        m_stats = next_stats;
        m_primitives = std::move(next_primitives);
        m_system_signature = next_signature;
        Touch();
    } else {
        // The cheap signature changed, but the renderable representation did
        // not.  Cache the new signature so we also skip the expensive staging
        // pass on subsequent frames.
        m_system_signature = next_signature;
    }
}

unsigned int ChVulkanRTScene::AddPointLight(ChVector3f pos, ChColor color, float max_range, bool const_color) {
    ChVulkanRTLight light;
    light.type = LightType::POINT_LIGHT;
    light.pos = pos;
    light.color = ChVector3f(color.R, color.G, color.B);
    light.range = max_range;
    light.const_color = const_color;
    light.atten_scale = (max_range > 0.f) ? (0.01f * max_range * max_range) : 1.f;
    m_lights.push_back(light);
    Touch();
    return static_cast<unsigned int>(m_lights.size() - 1);
}

unsigned int ChVulkanRTScene::AddDirectionalLight(const ChVector3f& dir, const ChVector3f& color) {
    ChVulkanRTLight light;
    light.type = LightType::DIRECTIONAL_LIGHT;
    light.dir = dir;
    light.color = color;
    m_lights.push_back(light);
    Touch();
    return static_cast<unsigned int>(m_lights.size() - 1);
}

unsigned int ChVulkanRTScene::AddDirectionalLight(ChColor color, float elevation, float azimuth) {
    ChVector3f dir(std::cos(elevation) * std::cos(azimuth),
                   std::cos(elevation) * std::sin(azimuth),
                   std::sin(elevation));
    return AddDirectionalLight(dir, ChVector3f(color.R, color.G, color.B));
}

unsigned int ChVulkanRTScene::AddSpotLight(const ChVector3f& pos,
                                           const ChVector3f& dir,
                                           const ChVector3f& color,
                                           float range,
                                           float angle) {
    ChVulkanRTLight light;
    light.type = LightType::SPOT_LIGHT;
    light.pos = pos;
    light.dir = dir;
    light.color = color;
    light.range = range;
    light.angle = angle;
    light.const_color = true;
    light.atten_scale = (range > 0.f) ? (0.01f * range * range) : 1.f;
    light.angle_falloff_start = angle;
    light.angle_atten_rate = -1.f;
    m_lights.push_back(light);
    Touch();
    return static_cast<unsigned int>(m_lights.size() - 1);
}

unsigned int ChVulkanRTScene::AddSpotLight(ChVector3f pos,
                                           ChColor color,
                                           float max_range,
                                           ChVector3f light_dir,
                                           float angle_falloff_start,
                                           float angle_range,
                                           bool const_color) {
    ChVulkanRTLight light;
    light.type = LightType::SPOT_LIGHT;
    light.pos = pos;
    light.dir = light_dir.GetNormalized();
    light.color = ChVector3f(color.R, color.G, color.B);
    light.range = max_range;
    light.angle = angle_range;
    light.const_color = const_color;
    light.atten_scale = (max_range > 0.f) ? (0.01f * max_range * max_range) : 1.f;
    if (angle_falloff_start < angle_range - 1e-6f) {
        light.angle_falloff_start = angle_falloff_start;
        light.angle_atten_rate = 1.f / (angle_range - angle_falloff_start);
    } else {
        light.angle_falloff_start = angle_range;
        light.angle_atten_rate = -1.f;
    }
    m_lights.push_back(light);
    Touch();
    return static_cast<unsigned int>(m_lights.size() - 1);
}


unsigned int ChVulkanRTScene::AddRectangleLight(ChVector3f pos,
                                                ChColor color,
                                                float max_range,
                                                ChVector3f length_vec,
                                                ChVector3f width_vec,
                                                bool const_color) {
    ChVulkanRTLight light;
    light.type = LightType::RECTANGLE_LIGHT;
    light.pos = pos;
    light.color = ChVector3f(color.R, color.G, color.B);
    light.range = max_range;
    light.const_color = const_color;
    light.atten_scale = (max_range > 0.f) ? (0.01f * max_range * max_range) : 1.f;
    light.length_vec = length_vec;
    light.width_vec = width_vec;
    const ChVector3f normal = Cross(length_vec, width_vec);
    light.area = normal.Length();
    light.dir = NormalizeOrDefault(normal, ChVector3f(0.f, 0.f, -1.f));
    m_lights.push_back(light);
    Touch();
    return static_cast<unsigned int>(m_lights.size() - 1);
}

unsigned int ChVulkanRTScene::AddDiskLight(ChVector3f pos,
                                           ChColor color,
                                           float max_range,
                                           ChVector3f light_dir,
                                           float radius,
                                           bool const_color) {
    ChVulkanRTLight light;
    light.type = LightType::DISK_LIGHT;
    light.pos = pos;
    light.dir = NormalizeOrDefault(light_dir, ChVector3f(0.f, 0.f, -1.f));
    light.color = ChVector3f(color.R, color.G, color.B);
    light.range = max_range;
    light.const_color = const_color;
    light.atten_scale = (max_range > 0.f) ? (0.01f * max_range * max_range) : 1.f;
    light.radius = radius;
    light.area = CH_VKRT_PI * radius * radius;
    m_lights.push_back(light);
    Touch();
    return static_cast<unsigned int>(m_lights.size() - 1);
}

unsigned int ChVulkanRTScene::AddEnvironmentLight(const std::string& env_tex, const ChVector3f& color) {
    ChVulkanRTLight light;
    light.type = LightType::ENVIRONMENT_LIGHT;
    light.texture = env_tex;
    light.color = color;
    m_lights.push_back(light);

    // The OptiX path treats an environment light as both an illumination source
    // and the texture sampled by miss/background rays.  Mirror that behavior so
    // Vulkan RT scenes do not lose their dominant sky light and become dark.
    if (!env_tex.empty()) {
        m_background.mode = BackgroundMode::ENVIRONMENT_MAP;
        m_background.env_tex = env_tex;
    }

    Touch();
    return static_cast<unsigned int>(m_lights.size() - 1);
}

unsigned int ChVulkanRTScene::AddEnvironmentLight(std::string env_tex_path, float intensity_scale) {
    return AddEnvironmentLight(env_tex_path, ChVector3f(intensity_scale, intensity_scale, intensity_scale));
}

}  // namespace sensor
}  // namespace chrono
