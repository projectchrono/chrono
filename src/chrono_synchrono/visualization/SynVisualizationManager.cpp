// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Jay Taves
// =============================================================================
//
// Class that manages a mix of Irrlicht and Sensor visualizations at the same
// time. This class abstracts away the need to separately update and maintain
// Irrlicht and Sensor visualizations.
//
// =============================================================================

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

namespace chrono {
namespace synchrono {

void SynVisualizationManager::Update(double step) {
    for (std::shared_ptr<SynVisualization> vis : m_vis_list)
        vis->Update(step);
}

void SynVisualizationManager::AddVisualization(std::shared_ptr<SynVisualization> vis) {
    vis->Initialize();
    m_vis_list.push_back(vis);
}

}  // namespace synchrono
}  // namespace chrono
