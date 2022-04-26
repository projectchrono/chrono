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

#include "chrono_sensor/optix/scene/ChScene.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChScene::ChScene() {
    m_background.mode = BackgroundMode::GRADIENT;
    m_background.color_zenith = {0.4f, 0.5f, 0.6f};
    m_background.color_horizon = {0.7f, 0.8f, 0.9f};
    m_background.env_tex = "";

    m_ambient_light = ChVector<float>({.2f, .2f, .2f});
    m_pointlights = std::vector<PointLight>();
    lights_changed = true;
    background_changed = true;

    m_scene_epsilon = 1e-3f;
}

CH_SENSOR_API ChScene::~ChScene() {}

CH_SENSOR_API unsigned int ChScene::AddPointLight(ChVector<float> pos, ChColor color, float max_range) {
    PointLight p;
    p.pos = {pos.x(), pos.y(), pos.z()};
    p.color = {color.R, color.G, color.B};
    p.max_range = max_range;
    m_pointlights.push_back(p);
    lights_changed = true;
    return static_cast<unsigned int>(m_pointlights.size() - 1);
}

CH_SENSOR_API unsigned int ChScene::AddPointLight(PointLight p) {
    m_pointlights.push_back(p);
    lights_changed = true;
    return static_cast<unsigned int>(m_pointlights.size() - 1);
}

CH_SENSOR_API void ChScene::ModifyPointLight(unsigned int id, PointLight p) {
    if (id <= m_pointlights.size()) {
        m_pointlights[id] = p;
        lights_changed = true;
    }
}

CH_SENSOR_API void ChScene::SetBackground(Background b) {
    m_background = b;
    background_changed = true;
}

CH_SENSOR_API void ChScene::SetSceneEpsilon(float e) {
    m_scene_epsilon = e;
    background_changed = true;
}

}  // namespace sensor
}  // namespace chrono
