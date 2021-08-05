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
// mrole simple driveline model.
//
// =============================================================================

#ifndef MROLE_SIMPLEDRIVELINE_XWD_H
#define MROLE_SIMPLEDRIVELINE_XWD_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDrivelineXWD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// Simple MAN 5t driveline subsystem (purely kinematic).
class CH_MODELS_API mrole_SimpleDrivelineXWD : public ChSimpleDrivelineXWD {
  public:
    mrole_SimpleDrivelineXWD(const std::string& name);

    ~mrole_SimpleDrivelineXWD() {}

    virtual double GetDifferentialMaxBias() const override { return m_diff_bias; }

  private:
    static const double m_diff_bias;
};

/// @} vehicle_models_mrole

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
