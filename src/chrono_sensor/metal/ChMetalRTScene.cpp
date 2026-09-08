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

#include "chrono_sensor/metal/ChMetalRTScene.h"
#include "chrono_sensor/metal/ChMetalSceneBuilder.h"

namespace chrono {
namespace sensor {

ChMetalRTScene::ChMetalRTScene() {}
ChMetalRTScene::~ChMetalRTScene() {}

void ChMetalRTScene::SyncFromSystem(ChSystem* sys) {
    if (!sys)
        return;
    if (!m_builder || sys != m_system) {
        m_builder = std::make_unique<ChMetalSceneBuilder>(sys);
        m_system = sys;
        m_built = false;
    }
    if (!m_built || m_builder->TopologyChanged()) {
        m_builder->Build(m_render_scene);
        m_built = true;
        m_structure_dirty = true;
    } else {
        m_builder->Refresh(m_render_scene);
    }
}

}  // namespace sensor
}  // namespace chrono
