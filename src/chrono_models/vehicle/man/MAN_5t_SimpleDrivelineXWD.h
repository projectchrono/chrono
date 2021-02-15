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
// MAN 5t simple driveline model.
//
// =============================================================================

#ifndef MAN5T_SIMPLEDRIVELINE_XWD_H
#define MAN5T_SIMPLEDRIVELINE_XWD_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDrivelineXWD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

/// Simple MAN 5t driveline subsystem (purely kinematic).
class CH_MODELS_API MAN_5t_SimpleDrivelineXWD : public ChSimpleDrivelineXWD {
  public:
    MAN_5t_SimpleDrivelineXWD(const std::string& name);

    ~MAN_5t_SimpleDrivelineXWD() {}

    virtual double GetDifferentialMaxBias() const override { return m_diff_bias; }

  private:
    static const double m_diff_bias;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
