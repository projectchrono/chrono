// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================
//
// Base class for industrial robotics kinematics.
//
// =============================================================================

#ifndef INDUSTRIAL_ROBOT_KINEMATICS_H
#define INDUSTRIAL_ROBOT_KINEMATICS_H

#include "chrono_models/ChApiModels.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChRotation.h"
#include "chrono/utils/ChUtils.h"

namespace chrono {
namespace industrial {

/// @addtogroup robot_models_industrial
/// @{

class CH_MODELS_API IndustrialKinematics {
  public:
    /// Default constructor.
    IndustrialKinematics();

    /// Virtual destructor.
    virtual ~IndustrialKinematics() {}

    /// Get number of robot joints.
    int GetNumJoints() const { return m_num_joints; }

    /// Toggle verbosity.
    void SetVerbose(bool verbose) { m_verbose = verbose; }

  protected:
    unsigned int m_num_joints = 0;
    bool m_verbose = false;
};

/// @} robot_models_industrial

}  // end namespace industrial
}  // end namespace chrono

#endif  // end INDUSTRIAL_ROBOT_KINEMATICS_H