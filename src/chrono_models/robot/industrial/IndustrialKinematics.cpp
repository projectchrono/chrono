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

#include "IndustrialKinematics.h"

namespace chrono {
namespace industrial {

IndustrialKinematics::IndustrialKinematics() : m_num_joints(0) {}

}  // end namespace industrial
}  // end namespace chrono