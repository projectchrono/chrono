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

#include "chrono_sensor/scene/ChScene.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChScene::ChScene() {
    m_background.color = {0.5f, 0.6f, 0.7f};
    m_background.has_texture = false;
    m_pointlights = std::vector<PointLight>();
}

CH_SENSOR_API ChScene::~ChScene() {}

void ChScene::AddPointLight(ChVector<float> pos, ChVector<float> color, float max_range) {
    PointLight p;
    p.pos = optix::make_float3(pos.x(), pos.y(), pos.z());
    p.color = optix::make_float3(color.x(), color.y(), color.z());
    p.max_range = max_range;
    m_pointlights.push_back(p);
}

}  // namespace sensor
}  // namespace chrono
