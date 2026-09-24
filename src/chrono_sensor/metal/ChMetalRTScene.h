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
// Backend-neutral scene staging for the Metal RT engine. Wraps the ChSystem ->
// RenderScene extraction (ChMetalSceneBuilder) and exposes the flat RenderScene the
// Metal renderer consumes.
//
// PUBLIC API PARITY: the scene-setup surface below mirrors chrono::sensor::ChOptixScene
// (the OptiX scene) signature-for-signature -- same names, same parameter order and
// types, same return values, same units and angle conventions. Simulation code that
// configures lighting/background/fog therefore compiles and behaves identically
// whether `manager->scene` is a ChOptixScene (OptiX build) or a ChMetalRTScene (Metal
// build). See the "Metal-only extensions" section at the bottom for the few members
// that have no ChOptixScene counterpart, and why.
// =============================================================================

#ifndef CH_METAL_RT_SCENE_H
#define CH_METAL_RT_SCENE_H

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "chrono/physics/ChSystem.h"
#include "chrono/assets/ChColor.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChUtils.h"
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChSensorRenderTypes.h"  // Background / BackgroundMode (shared with the OptiX build)
#include "chrono_sensor/metal/ChMetalRenderTypes.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class ChMetalSceneBuilder;

/// A light in the Metal render scene, in the packed form the GPU consumes.
///
/// This is the Metal analogue of ChOptixLight. It is deliberately NOT ChOptixLight:
/// that struct is built from CUDA vector types (float3, cudaTextureObject_t,
/// curand state), so reusing it would drag the CUDA toolchain into a Metal-only
/// build. The public setters above convert into this form, so the difference is
/// invisible to simulation code.
struct MetalSceneLight {
    ChVector3f pos;            ///< world position (point/spot); for directional, the light TRAVEL direction
    float range;               ///< max_range (point/spot falloff; 0 = none)
    ChColor color;             ///< color * intensity
    int type;                  ///< 0 = point, 1 = directional, 2 = spot, 3 = disk, 4 = rectangle
    ChVector3f dir{0, 0, -1};  ///< spot axis / disk normal / rectangle edge-1
    float cosOuter = -1.f;     ///< spot: angle_range (full cone, rad)  | rect: edge-2 .x
    float cosInner = -1.f;     ///< spot: angle_atten_rate              | rect: edge-2 .y
    float p0 = 0.f;            ///< disk: radius                        | rect: edge-2 .z
    bool const_color = true;   ///< constant colour (no distance attenuation)
};

class CH_SENSOR_API ChMetalRTScene {
  public:
    ChMetalRTScene();
    ~ChMetalRTScene();

    /// Build (first call / topology change) or refresh (per frame) the render scene.
    void SyncFromSystem(ChSystem* sys);

    // =========================================================================
    // Scene API -- mirrors chrono::sensor::ChOptixScene exactly. Keep in sync.
    // =========================================================================

    /// Add a point light. Returns the light's ID (its index), as ChOptixScene does.
    unsigned int AddPointLight(ChVector3f pos, ChColor color, float max_range, bool const_color = true) { return Append(MakePointLight(pos, color, max_range, const_color)); }

    /// Redefine an existing point light, matching ChOptixScene::ModifyPointLight's neutral overload.
    /// Same parameters as AddPointLight plus the light ID returned by it.
    void ModifyPointLight(unsigned int light_ID, ChVector3f pos, ChColor color, float max_range, bool const_color = true) {
        Replace(light_ID, MakePointLight(pos, color, max_range, const_color));
    }

    /// Add a directional light from spherical angles, matching ChOptixScene::AddDirectionalLight.
    /// elevation/azimuth are radians and define the direction TO the light:
    ///   light_dir = (cos(el)cos(az), cos(el)sin(az), sin(el))
    unsigned int AddDirectionalLight(ChColor color, float elevation, float azimuth) { return Append(MakeDirectionalLight(color, elevation, azimuth)); }

    /// Redefine an existing directional light, matching ChOptixScene::ModifyDirectionalLight's neutral overload.
    void ModifyDirectionalLight(unsigned int light_ID, ChColor color, float elevation, float azimuth) { Replace(light_ID, MakeDirectionalLight(color, elevation, azimuth)); }

    /// Add a spot light, matching ChOptixScene::AddSpotLight. Angles are radians and are
    /// FULL cone angles: angle_range is the full cone, angle_falloff_start is where the
    /// soft edge begins. Attenuation rate is 1/(angle_range - angle_falloff_start).
    unsigned int AddSpotLight(ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float angle_falloff_start, float angle_range, bool const_color = true) {
        return Append(MakeSpotLight(pos, color, max_range, light_dir, angle_falloff_start, angle_range, const_color));
    }

    /// Redefine an existing spot light, matching ChOptixScene::ModifySpotLight's neutral overload.
    void ModifySpotLight(unsigned int light_ID,
                         ChVector3f pos,
                         ChColor color,
                         float max_range,
                         ChVector3f light_dir,
                         float angle_falloff_start,
                         float angle_range,
                         bool const_color = true) {
        Replace(light_ID, MakeSpotLight(pos, color, max_range, light_dir, angle_falloff_start, angle_range, const_color));
    }

    /// Add a rectangle area light (soft-shadowed), matching ChOptixScene::AddRectangleLight.
    /// length_vec / width_vec are the FULL edge vectors, not half-extents.
    unsigned int AddRectangleLight(ChVector3f pos, ChColor color, float max_range, ChVector3f length_vec, ChVector3f width_vec, bool const_color = true) {
        return Append(MakeRectangleLight(pos, color, max_range, length_vec, width_vec, const_color));
    }

    /// Redefine an existing rectangle light, matching ChOptixScene::ModifyRectangleLight's neutral overload.
    void ModifyRectangleLight(unsigned int light_ID, ChVector3f pos, ChColor color, float max_range, ChVector3f length_vec, ChVector3f width_vec, bool const_color = true) {
        Replace(light_ID, MakeRectangleLight(pos, color, max_range, length_vec, width_vec, const_color));
    }

    /// Add a disk area light (soft-shadowed), matching ChOptixScene::AddDiskLight.
    unsigned int AddDiskLight(ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float radius, bool const_color = true) {
        return Append(MakeDiskLight(pos, color, max_range, light_dir, radius, const_color));
    }

    /// Redefine an existing disk light, matching ChOptixScene::ModifyDiskLight's neutral overload.
    void ModifyDiskLight(unsigned int light_ID, ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float radius, bool const_color = true) {
        Replace(light_ID, MakeDiskLight(pos, color, max_range, light_dir, radius, const_color));
    }

    /// Use an HDR equirectangular map as an image-based light, matching
    /// ChOptixScene::AddEnvironmentLight. Also sets the background to that map, so the same
    /// texture provides sky, reflections, and (in GI mode) bounce lighting.
    unsigned int AddEnvironmentLight(std::string env_tex_path, float intensity_scale = 1.f) {
        m_env_intensity = intensity_scale;
        m_background.mode = BackgroundMode::ENVIRONMENT_MAP;
        m_background.env_tex = env_tex_path;
        background_changed = true;
        lights_changed = true;
        return static_cast<unsigned int>(m_lights.size());
    }

    /// Remove every light from the scene.
    void ClearLights() {
        m_lights.clear();
        lights_changed = true;
    }

    /// All lights currently in the scene.
    /// NOTE: element type is MetalSceneLight rather than ChOptixLight -- see the struct
    /// comment. Simulation code configures lights through the setters above, so this
    /// difference does not affect portability. ChOptixScene additionally overloads each
    /// Modify*Light on ChOptixLight; only its backend-neutral overloads are mirrored here,
    /// since a ChOptixLight cannot be named in a Metal-only build.
    const std::vector<MetalSceneLight>& GetLights() const { return m_lights; }

    void SetAmbientLight(ChVector3f color) { m_ambient = color; }
    ChVector3f GetAmbientLight() const { return m_ambient; }

    void SetBackground(Background b) {
        m_background = b;
        background_changed = true;
    }
    Background GetBackground() const { return m_background; }

    void SetFogColor(ChVector3f color) {
        m_fog_color = color;
        background_changed = true;
    }
    ChVector3f GetFogColor() const { return m_fog_color; }

    void SetFogScattering(float coefficient) {
        m_fog_scattering = ChClamp(coefficient, 0.f, 1.f);
        background_changed = true;
    }
    /// Set fog density from the distance at which visibility falls to ~1/256.
    void SetFogScatteringFromDistance(float distance) {
        distance = ChClamp(distance, 1e-3f, 1e16f);
        m_fog_scattering = std::log(256.0f) / distance;
        background_changed = true;
    }
    float GetFogScattering() const { return m_fog_scattering; }

    void SetSceneEpsilon(float e) { m_scene_epsilon = e; }
    float GetSceneEpsilon() const { return m_scene_epsilon; }

    bool GetLightsChanged() const { return lights_changed; }
    void ResetLightsChanged() { lights_changed = false; }
    bool GetBackgroundChanged() const { return background_changed; }
    void ResetBackgroundChanged() { background_changed = false; }

    // =========================================================================
    // Metal-only extensions (no chrono::sensor::ChOptixScene counterpart)
    //
    // On the OptiX backend these physical-camera effects are properties of
    // ChPhysCameraSensor (aperture number, focal length, gain/noise params) and are
    // applied by ChFilterPhysCameraDefocusBlur / ChFilterPhysCameraVignetting, not by
    // the scene. The Metal backend does not implement ChPhysCameraSensor yet, so it
    // exposes them here as scene-level knobs. Simulation code that uses these is
    // Metal-specific until that sensor is ported.
    // =========================================================================
    void SetExposure(float e) { m_exposure = e; }
    void SetVignette(float v) { m_vignette = v; }
    void SetSensorNoise(float sigma) { m_noise_sigma = sigma; }
    void SetDepthOfField(float aperture_radius, float focal_dist) {
        m_aperture_r = aperture_radius;
        m_focal_dist = focal_dist;
    }
    float GetExposure() const { return m_exposure; }
    float GetVignette() const { return m_vignette; }
    float GetSensorNoise() const { return m_noise_sigma; }
    float GetApertureRadius() const { return m_aperture_r; }
    float GetFocalDist() const { return m_focal_dist; }

    /// Environment-map radiance scale set by AddEnvironmentLight (renderer-internal).
    float GetEnvIntensity() const { return m_env_intensity; }

    const MetalRenderScene& GetRenderScene() const { return m_render_scene; }

    /// True when the geometry set changed and acceleration structures must be rebuilt.
    bool StructureDirty() const { return m_structure_dirty; }
    void ClearStructureDirty() { m_structure_dirty = false; }

    ChSystem* GetSystem() const { return m_system; }

  private:
    // ---- Light construction -------------------------------------------------
    // Each Make*Light packs the public (backend-neutral) parameters into a MetalSceneLight.
    // Add*Light appends the result, Modify*Light overwrites an existing slot with it, so the
    // two paths cannot drift apart.

    static MetalSceneLight MakePointLight(ChVector3f pos, ChColor color, float max_range, bool const_color) {
        MetalSceneLight L{pos, max_range, color, 0};
        L.const_color = const_color;
        return L;
    }

    static MetalSceneLight MakeDirectionalLight(ChColor color, float elevation, float azimuth) {
        ChVector3f light_dir(std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
        // The shader negates `pos` to recover the direction to the light, so store the
        // travel direction here. Net result matches the OptiX light_dir.
        return MetalSceneLight{-light_dir, 0.f, color, 1};
    }

    static MetalSceneLight MakeSpotLight(ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float angle_falloff_start, float angle_range, bool const_color) {
        MetalSceneLight L{pos, max_range, color, 2};
        L.dir = light_dir.GetNormalized();
        L.cosOuter = angle_range;
        L.cosInner = (angle_falloff_start < angle_range - 1e-6f) ? (1.f / (angle_range - angle_falloff_start)) : -1.f;  // no angular falloff -> hard cutoff
        L.const_color = const_color;
        return L;
    }

    static MetalSceneLight MakeRectangleLight(ChVector3f pos, ChColor color, float max_range, ChVector3f length_vec, ChVector3f width_vec, bool const_color) {
        MetalSceneLight L{pos, max_range, color, 4};
        L.dir = length_vec;  // edge-1
        L.cosOuter = width_vec.x();
        L.cosInner = width_vec.y();
        L.p0 = width_vec.z();  // edge-2
        L.const_color = const_color;
        return L;
    }

    static MetalSceneLight MakeDiskLight(ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float radius, bool const_color) {
        MetalSceneLight L{pos, max_range, color, 3};
        L.dir = light_dir.GetNormalized();
        L.p0 = radius;
        L.const_color = const_color;
        return L;
    }

    /// Append a light and return its ID (its index), as ChOptixScene's Add*Light do.
    unsigned int Append(const MetalSceneLight& L) {
        m_lights.push_back(L);
        lights_changed = true;
        return static_cast<unsigned int>(m_lights.size() - 1);
    }

    /// Overwrite the light at `id`. Out-of-range IDs are ignored, as ChOptixScene's Modify*Light do.
    void Replace(unsigned int id, const MetalSceneLight& L) {
        if (id < m_lights.size()) {
            m_lights[id] = L;
            lights_changed = true;
        }
    }

    ChSystem* m_system = nullptr;
    std::unique_ptr<ChMetalSceneBuilder> m_builder;
    MetalRenderScene m_render_scene;
    bool m_built = false;
    bool m_structure_dirty = false;

    std::vector<MetalSceneLight> m_lights;
    ChVector3f m_ambient{0.30f, 0.30f, 0.35f};
    Background m_background;  // defaults to SOLID_COLOR black, as ChOptixScene's does
    float m_env_intensity = 1.f;
    ChVector3f m_fog_color{0.6f, 0.7f, 0.8f};
    float m_fog_scattering = 0.f;
    float m_scene_epsilon = 1e-3f;
    bool lights_changed = true;
    bool background_changed = true;

    // Metal-only physical-camera state (see note above)
    float m_exposure = 1.f, m_vignette = 0.f, m_noise_sigma = 0.f, m_aperture_r = 0.f, m_focal_dist = 10.f;
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
