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

#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChColor.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_scene
/// @{

/// Information about the background of the scene. Determines the color, gradient, or image that is spherically mapped
/// to camera ray misses
struct Background {
    BackgroundMode mode;            ///< 0=solid zenith color, 1=gradient, 2=environment map
    ChVector<float> color_zenith;   ///< color at zenith
    ChVector<float> color_horizon;  ///< color at horizon (for gradient)
    std::string env_tex;            ///< full path name of the texture
};

/// Scene class used for camera renderings. Includes environment colors, lights, etc
class CH_SENSOR_API ChScene {
  public:
    /// Class constructor
    ChScene();
    /// Class destructor
    ~ChScene();

    /// Add a point light that emits light in all directions.
    /// @param pos The global position of the light source
    /// @param color The golor of the light source
    /// @param max_range the range at which the light intensity is equal to 1% of its maximum intensity
    /// @return the index of the light that has been added
    unsigned int AddPointLight(ChVector<float> pos, ChColor color, float max_range);

    /// Add a point light that emits light in all directions.
    /// @param p A point light the will be added directly
    /// @return the index of the light that has been added
    unsigned int AddPointLight(const PointLight& p);

    /// Function for gaining access to the vector of point lights and can be used to modify lighting dynamically.
    /// @return m_pointlights A vector of point lights in the scene currently
    std::vector<PointLight> GetPointLights() { return m_pointlights; }

    /// Function for gaining access to the vector of point lights and can be used to modify lighting dynamically.
    /// @param id the index of the point light to be modified
    /// @param p the new point light that will replace the values at the given index
    void ModifyPointLight(unsigned int id, PointLight p);

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
    void SetAmbientLight(ChVector<float> color) { m_ambient_light = color; }

    /// Function for setting the ambient light color
    /// @return the ambient light in the scene
    ChVector<float> GetAmbientLight() { return m_ambient_light; }

    /// Function for resetting the lights changed variable
    void ResetLightsChanged() { lights_changed = false; }
    /// Function for getting the lights changed variable
    bool GetLightsChanged() { return lights_changed; }

    /// Function for resetting the background changed variable
    void ResetBackgroundChanged() { background_changed = false; }
    /// Function for getting the background changed variable
    bool GetBackgroundChanged() { return background_changed; }

    /// Function to set the fog color
    void SetFogColor(ChVector<float> color);

    /// Function to get the fog color
    ChVector<float> GetFogColor() { return m_fog_color; }

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

    /// Function to change the origin offset if necessary
    /// @param sensor_pos the position of the sensor
    /// @param force whether to force updating even if threshold is not met
    void UpdateOriginOffset(ChVector<float> sensor_pos, bool force = false);

    bool GetOriginChanged() { return m_origin_changed; }

    void ResetOriginChanged() { m_origin_changed = false; }

    /// Access function for the origin offset
    /// @returns the origin offset
    ChVector<float> GetOriginOffset() { return m_origin_offset; }

    /// Set the threshold for moving the origin
    /// @param threshold the threshold outside of which to move the scene origin
    void SetOriginOffsetThreshold(float threshold) { m_dynamic_origin_threshold = threshold; }

    /// Enable dynamic moving of the scene origin
    /// @param enable whether to enable to the moving origin
    void EnableDynamicOrigin(bool enable) { m_dynamic_origin_offset = enable; }

  private:
    std::vector<PointLight> m_pointlights;  //< list of point lights in the scene
    Background m_background;                ///< The background object
    ChVector<float> m_ambient_light;        ///< ambient light color used in the scene

    bool lights_changed;      ///< for detecting if lights changed
    bool background_changed;  ///< for detecting if background changed

    float m_scene_epsilon;             ///< smallest value used for checking traversal hits
    bool m_dynamic_origin_offset;      ///< whether to dynamically change the scene origin when sensors are outside of
                                       ///< threshold
    float m_dynamic_origin_threshold;  ///< threshold to prompt moving the origin
    ChVector<float> m_origin_offset;   ///< scene origin offset from Chrono
    bool m_origin_changed;             ///< whether the origin changed since last reset

    float m_fog_scattering;       ///< scattering coefficient of fog in the scene
    ChVector<float> m_fog_color;  ///< color of the fog in the scene
};

/// @} sensor_scene

}  // namespace sensor
}  // namespace chrono

#endif
