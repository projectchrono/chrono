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
 #define NOMINMAX
#endif

#include <optix.h>
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>

#include "chrono/physics/ChBody.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/scene/lights.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_scene
/// @{

struct Background {
    ChVector<float> color;  ///< background color
    bool has_texture;       ///< sets whether a texture should be used for background instead of color
    std::string env_tex;    ///< full path name of the texture
    bool has_changed;       ///< set if the background was changed since last scene update
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
    /// @param the range at which the light intensity is equal to 1% of its maximum intensity
    void AddPointLight(ChVector<float> pos, ChVector<float> color, float max_range);

    /// Function for gaining access to the vector of point lights and can be used to modify lighting dynamically.
    /// @return m_pointlights A reference to the vector of point lights
    std::vector<PointLight>& GetPointLights() { return m_pointlights; }

    /// Function for gaining access to the background. Can be used to dynamically change the background color, or
    /// texture
    /// @return m_background A reference to the scene background used for rendering
    Background& GetBackground() { return m_background; }

  private:
    std::vector<PointLight> m_pointlights;  //< list of point lights in the scene
    Background m_background;                ///< The background object
};

/// @} sensor_scene

}  // namespace sensor
}  // namespace chrono

#endif
