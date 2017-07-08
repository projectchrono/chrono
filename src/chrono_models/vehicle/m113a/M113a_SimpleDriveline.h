// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// A simplified M113 driveline.
//
// =============================================================================

#ifndef M113a_SIMPLE_DRIVELINE_H
#define M113a_SIMPLE_DRIVELINE_H

#include "chrono_vehicle/tracked_vehicle/driveline/ChSimpleTrackDriveline.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

class CH_MODELS_API M113a_SimpleDriveline : public ChSimpleTrackDriveline {
  public:
    M113a_SimpleDriveline();
    ~M113a_SimpleDriveline() {}

    /// Return the torque bias ratio for the differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetDifferentialMaxBias() const override { return m_diff_maxBias; }

  private:
    static const double m_diff_maxBias;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
