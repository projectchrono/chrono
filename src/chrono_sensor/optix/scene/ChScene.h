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
// =============================================================================

#ifndef ChScene_H
#define ChScene_H

#include <deque>
#include <mutex>
#include <vector>

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

// #include <optix.h>
// #include <optix_world.h>
// #include <optixu/optixu_math_namespace.h>
#include <cuda_runtime.h>

#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChColor.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"
#include "chrono_sensor/optix/ChOptixUtils.h"
#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h"

#ifdef USE_SENSOR_NVDB
  #include <openvdb/openvdb.h>
#endif


namespace chrono {
namespace sensor {

/// @addtogroup sensor_scene
/// @{

/// Information about the background of the scene. Determines the color, gradient, or image that is spherically mapped
/// to camera ray misses
struct Background {
    BackgroundMode mode;            ///< 0=solid zenith color, 1=gradient, 2=environment map
    ChVector3f color_zenith;   ///< color at zenith
    ChVector3f color_horizon;  ///< color at horizon (for gradient)
    std::string env_tex;            ///< full path name of the texture
};

/// Scene class used for camera renderings. Includes environment colors, lights, etc
class CH_SENSOR_API ChScene {
  public:
    /// Class constructor
    ChScene();
    /// Class destructor
    ~ChScene();

    /// @brief Add a point light that emits light in all directions.
    /// @param pos The world position of the point light
    /// @param color [W/sr/m^2] or [lumen/sr/m^2], color radiance of the light
    /// @param max_range [m], range at which the light intensity falls to 1% of its maximum color intensity.
    /// If set to -1, follows inverse square law.
    /// @return the index of the light that has been added
    unsigned int AddPointLight(ChVector3f pos, ChColor color, float max_range, bool const_color = true);

    /// @brief Function for modifying an existing point light in the scene
    /// @param light_ID the index of the point light to be modified
    /// @param point_light the new point light
    void ModifyPointLight(unsigned int light_ID, const ChOptixLight& point_light);

    /// @brief Add a directional light that emits light in a particular direction.
    /// @param color [W/m^2] or [lumen/m^2], color irradiance of the light
    /// @param elevation [rad], elevation angle of the directional light comes from
    /// @param azimuth [rad], azimuth angle of the directional light comes from
    /// @return the index of the light that has been added
    unsigned int AddDirectionalLight(ChColor color, float elevation, float azimuth);

    /// @brief Function for modifying an existing directional light in the scene
    /// @param light_ID the index of the directional light to be modified
    /// @param directional_light the new directional light
    void ModifyDirectionalLight(unsigned int light_ID, const ChOptixLight& directional_light);


    /// @brief Add a spot light that emits light in a particular direction.
    /// @param pos the world position of the spot light
    /// @param color color radiance of the light
    /// @param max_range range at which the light intensity falls to 1% of its maximum color intensity.
    /// If set to -1, follows inverse square law.
    /// @param light_dir the direction in which the spotlight points (no need to be normalized)
    /// @param angle_falloff_start [m], angle at which the spotlight starts to linearly fall off
    /// @param angle_range [rad], angle range of the spotlight falling off to zero.
    /// @param const_color whether to use constant color (no attenuation with distance)
    /// @return the index of the light that has been added
    unsigned int AddSpotLight(
      ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float angle_falloff_start, float angle_range, bool const_color = true
    );

    /// @brief Function for modifying an existing spot light in the scene
    /// @param light_ID the index of the spot light to be modified
    /// @param spot_light the new spot light
    void ModifySpotLight(unsigned int light_ID, const ChOptixLight& spot_light);
    
    /// @brief Add a rectangle light that emits light forward from a rectangle area.
    /// @param pos [m], the world position of the rectangle light
    /// @param color [W/m^2] or [lumen/m^2], color radiance of the light
    /// @param max_range [m], range at which the light intensity falls to 1% of its maximum color intensity.
    /// If set to -1, follows inverse square law.
    /// @param length_vec [m], one edge vector of the rectangle light
    /// @param width_vec [m], the other edge vector of the rectangle light perpendicular to `length_vec`. Light direction is: length_vec x width_vec.
    /// @param const_color whether to use constant color (no attenuation with distance)
    /// @return the index of the added light
    unsigned int AddRectangleLight(
      ChVector3f pos, ChColor color, float max_range, ChVector3f length_vec, ChVector3f width_vec, bool const_color = true
    );

    /// @brief Function for modifying an existing rectangle light in the scene
    /// @param light_ID the index of the rectangle light to be modified
    /// @param rectangle_light the new rectangle light
    void ModifyRectangleLight(unsigned int light_ID, const ChOptixLight& rectangle_light);

    /// @brief Add a disk light that emits light forward from a circular area.
    /// @param pos [m], the world position of the rectangle light
    /// @param color [W/m^2] or [lumen/m^2], color radiance of the light
    /// @param max_range [m], range at which the light intensity falls to 1% of its maximum color intensity.
    /// If set to -1, follows inverse square law.
    /// @param light_dir the direction in which the disk light points (no need to be normalized)
    /// @param radius [m], radius of the disk light
    /// @param const_color whether to use constant color (no attenuation with distance)
    /// @return the index of the added light
    unsigned int AddDiskLight(
      ChVector3f pos, ChColor color, float max_range, ChVector3f light_dir, float radius, bool const_color = true
    );

    /// @brief Function for modifying an existing disk light in the scene
    /// @param light_ID the index of the disk light to be modified
    /// @param disk_light the new disk light
    void ModifyDiskLight(unsigned int light_ID, const ChOptixLight& disk_light);
    
    /// @brief Add the environment light that emits light from all directions based on an environment map.
    /// @param env_tex_path the full path of the environment map texture.
    /// @param intensity_scale a scale factor for the intensity of the environment light. Default value is 1.0 (no scaling).
    /// @return the index of the light that has been added
    unsigned int AddEnvironmentLight(std::string env_tex_path, float intensity_scale = 1.f);

    /// Function for gaining access to the vector of lights which can be used to modify lighting dynamically.
    /// @return A vector of lights in the scene currently
    std::vector<ChOptixLight> GetLights() {return m_lights;}

    /// Function for gaining access to the background. Can be used to dynamically change the background color, or
    /// texture
    /// @return m_background the background used for rendering
    Background GetBackground() { return m_background; }

    /// Function for gaining access to the background. Can be used to dynamically change the background color, or
    /// texture
    /// @param b a new background for the scene
    void SetBackground(Background b);

    /// Function for setting the ambient light color
    /// @param color the color+intensity of ambient light
    void SetAmbientLight(ChVector3f color) { m_ambient_light = color; }

    /// Function for setting the ambient light color
    /// @return the ambient light in the scene
    ChVector3f GetAmbientLight() { return m_ambient_light; }

    /// Function for resetting the lights changed variable
    void ResetLightsChanged() { lights_changed = false; }

    /// Function for getting the lights changed variable
    bool GetLightsChanged() { return lights_changed; }

    /// Function for resetting the background changed variable
    void ResetBackgroundChanged() { background_changed = false; }
    /// Function for getting the background changed variable
    bool GetBackgroundChanged() { return background_changed; }

    /// Function to set the fog color
    void SetFogColor(ChVector3f color);

    /// Function to get the fog color
    ChVector3f GetFogColor() { return m_fog_color; }

    /// Function to set the fog scattering coefficient
    void SetFogScattering(float coefficient);

    /// Function to set the fog scattering coefficient from max visible distance
    void SetFogScatteringFromDistance(float distance);

    /// Function to get the fog scattering coefficient
    float GetFogScattering() { return m_fog_scattering; }

    /// Allows setting the scene epsilon used for traversal checks
    /// @param e the epsilon value
    void SetSceneEpsilon(float e);

    /// Accessor to the scene epsilon value
    /// @return the scene epsilon
    float GetSceneEpsilon() { return m_scene_epsilon; }

    void AddSprite(std::shared_ptr<ChBody> sprite);
    void SetSprites(std::vector<std::shared_ptr<ChBody>> sprites) {m_sprites = sprites;}

    std::shared_ptr<ChBody> GetSprite(int i) { return m_sprites[i]; }
    std::vector<std::shared_ptr<ChBody>> GetSprites() { return m_sprites; }

    #ifdef USE_SENSOR_NVDB
    /// @brief  Allows passing in Chrono::FSI SPH markers to the scene, to be used for rendering SPH simulations. Note: Must also add a ChNVDBVolume body to the scene as well.
    /// @param fsi_points_d pointer to the FSI markers in host memory
    void SetFSIParticles(float* fsi_points) { m_fsi_points = fsi_points; }

    /// @brief  Sets the number of FSI markers to be rendered
    /// @param n the number of FSI markers
    void SetFSINumFSIParticles(int n) { m_num_fsi_points = n; }

    /// @brief Returns a host pointer to the Chrono::FSI, SPH markers being rendered
    /// @return float* to Chrono::FSI, SPH markers in the scene
    float* GetFSIParticles() { return m_fsi_points; }

    /// @brief Returns the number of Chrono::FSI, SPH markers in the scene
    /// @return the number of Chrono::FSI, SPH markers in the scene (int)
    int GetNumFSIParticles() { return m_num_fsi_points;}
    #endif

    /// Function to change the origin offset if necessary
    /// @param sensor_pos the position of the sensor
    /// @param force whether to force updating even if threshold is not met
    void UpdateOriginOffset(ChVector3f sensor_pos, bool force = false);

    bool GetOriginChanged() { return m_origin_changed; }

    void ResetOriginChanged() { m_origin_changed = false; }

    /// Access function for the origin offset
    /// @returns the origin offset
    ChVector3f GetOriginOffset() { return m_origin_offset; }

    /// Set the threshold for moving the origin
    /// @param threshold the threshold outside of which to move the scene origin
    void SetOriginOffsetThreshold(float threshold) { m_dynamic_origin_threshold = threshold; }

    /// Enable dynamic moving of the scene origin
    /// @param enable whether to enable to the moving origin
    void EnableDynamicOrigin(bool enable) { m_dynamic_origin_offset = enable; }

    // void SetGVDBVolume(gvdb::VolumeGVDB* gvdb) { m_gvdb = gvdb; }
    // nvdb::VolumeGVDB* GetGVDBVolume() { return m_gvdb; }

    // void SetGVDBChan(int chan) { m_gvdb_chan = chan; }
    // int GetGVDBChan() { return m_gvdb_chan; }

  private:
    std::vector<ChOptixLight> m_lights;     //< list of all lights in the scene
    Background m_background;                ///< The background object
    ChVector3f m_ambient_light;        ///< ambient light color used in the scene

    bool lights_changed;      ///< for detecting if lights changed
    bool background_changed;  ///< for detecting if background changed

    float m_scene_epsilon;             ///< smallest value used for checking traversal hits
    bool m_dynamic_origin_offset;      ///< whether to dynamically change the scene origin when sensors are outside of
                                       ///< threshold
    float m_dynamic_origin_threshold;  ///< threshold to prompt moving the origin
    ChVector3f m_origin_offset;   ///< scene origin offset from Chrono
    bool m_origin_changed;             ///< whether the origin changed since last reset

    float m_fog_scattering;       ///< scattering coefficient of fog in the scene
    ChVector3f m_fog_color;  ///< color of the fog in the scene

    std::vector<std::shared_ptr<ChBody>> m_sprites;  ///< list of sprites in the scene
    // nvdb::VolumeGVDB* m_gvdb;  // GVDB volume of the scene
    // int m_gvdb_chan;           // GVDB render channel

    #ifdef USE_SENSOR_NVDB
      float* m_fsi_points = nullptr; // Pointer to FSI particle positions in host
      int m_num_fsi_points = 0; // Number of FSI particles
    #endif
};

/// @} sensor_scene

}  // namespace sensor
}  // namespace chrono

#endif
