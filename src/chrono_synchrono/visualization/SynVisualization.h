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
// Base class for visualizations that a SynVisualizationManager is able to
// manage. Visualizations (Irrlicht and Sensor) must provide functions for
// updating their view of the world (Update) and for performing any setup steps
// (Initialize)
//
// =============================================================================

#ifndef SYN_VISUALIZATION_H
#define SYN_VISUALIZATION_H

#include "chrono_synchrono/SynApi.h"

#include <vector>
#include <memory>

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_visualization
/// @{

/// Base class for a visualization wrapper (either Irrlicht or Sensor currently)
class SynVisualization {
  public:
    enum VisualizationType { SENSOR, IRRLICHT };

    SynVisualization(VisualizationType type) : m_type(type), m_should_initialize(true) {}
    ~SynVisualization() {}

    /// Whether or not visualizers need initialization
    bool ShouldInitialize() { return m_should_initialize; }

    /// @brief Advance the state of this visualizer.
    virtual void Update(double step) = 0;

    /// @brief Initialize this visualizer.
    virtual void Initialize() = 0;

    VisualizationType GetType() { return m_type; }

  protected:
    VisualizationType m_type;

    bool m_should_initialize;
};

typedef std::vector<std::shared_ptr<SynVisualization>> SynVisualizationList;

/// @} synchrono_visualization

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_VIS_H
