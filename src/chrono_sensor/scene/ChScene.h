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

#include <optix.h>
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>

#include "chrono/physics/ChBody.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/scene/ChWeather.h"
#include "chrono_sensor/scene/lights.h"

namespace chrono {
namespace sensor {

// struct TimeDate {
//     int time;
//     int date;
// };

// struct SceneBody {
//     float pos[7];       // x,y,z,e0,e1,e2,e3,e4
//     float vel[6];       // x_dt, y_dt, z_dt, angx_dt, angy_dt, angz_dt
//     ChBody chronobody;  // pointer to corresponding chrono body
//     // optix::Transform optixtransform;  // pointer to corresponding optix transform
// };

struct Background {
    ChVector<float> color;
    bool has_texture;
    std::string env_tex;
    bool has_changed;
};

struct Keyframe {
    // float sim_time;     // simulation time of the frame
    // SceneBody* bodies;  // list of body positions

    // Snow snow;                // state of the snow
    // Rain rain;                // state of the rain
    // Fog fog;                  // state of the fog
    // Wind wind;                // state of the wind
    // Temperature temperature;  // state of the temperature
    // SunSky sunsky;            // state of the sky

    std::vector<PointLight> pointlights;  // list of point lights for that keyframe
};

class CH_SENSOR_API ChScene {
  public:
    ChScene();
    ~ChScene();

    void AddPointLight(ChVector<float> pos, ChVector<float> color, float max_range);
    std::vector<PointLight>& GetPointLights() { return m_pointlights; }

    // void SetDateTime();
    // void GetDateTime();

    // void SetRain(Rain rain) { m_rain = rain; }
    // Rain GetRain() { return m_rain; }
    // void SetSnow(Snow snow) { m_snow = snow; }
    // Snow GetSnow() { return m_snow; }
    // void SetFog(Fog fog) { m_fog = fog; }
    // Fog GetFog() { return m_fog; }

    Background& GetBackground() { return m_background; }

    bool SetMaxKeyframes(int num_frames);
    int GetMaxKeyframes() { return max_keyframes; }

    void PackFrame(ChSystem* pSystem);        // saves the keyframe with the current states of the scene
    std::deque<Keyframe> GetKeyframesCopy();  // return a copy of the keyframes (is or will be threadsafe)

  private:
    // TimeDate m_timedate;
    // SunSky m_sunsky;
    // Temperature m_temperature;
    // Rain m_rain;
    // Snow m_snow;
    // Wind m_wind;
    // Fog m_fog;

    int max_keyframes = 10;
    Keyframe m_current_frame;
    std::deque<Keyframe> m_keyframes;
    std::mutex keyframe_mutex;

    std::vector<PointLight> m_pointlights;  // list of point lights for that keyframe
    Background m_background;
};

}  // namespace sensor
}  // namespace chrono

#endif
