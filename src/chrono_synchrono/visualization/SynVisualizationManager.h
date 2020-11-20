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

#ifndef SYN_VISUALIZATION_MANAGER_H
#define SYN_VISUALIZATION_MANAGER_H

#include "chrono_synchrono/visualization/SynVisualization.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_visualization
/// @{

/// Abstracts away the need to separately update and maintain Irrlicht and Sensor visualizations
class SYN_API SynVisualizationManager {
  public:
    /// Constructs a visualization manager
    SynVisualizationManager() {}

    /// Destructor
    ~SynVisualizationManager() {}

    /// @brief Tells its visualizations to perform any needed updates
    void Update(double step);

    /// @brief Initializes this visualization and adds it to the manager's list
    void AddVisualization(std::shared_ptr<SynVisualization> vis);

    /// Get visualization list
    SynVisualizationList GetVisualizationList() { return m_vis_list; }

  protected:
    SynVisualizationList m_vis_list;  ///< vector of handles to visualizations
};

/// @} synchrono_visualization

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_VIS_MANAGER_H
