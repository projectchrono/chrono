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
// Base class for the mrole vehicle models
//
// =============================================================================

#ifndef MROLE_VEHICLE_H
#define MROLE_VEHICLE_H

#include <vector>
#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Base class for a mrole vehicle.
class CH_MODELS_API mrole_Vehicle : public ChWheeledVehicle {
  public:
    virtual ~mrole_Vehicle() {}

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 3.378; }
    virtual double GetMinTurningRadius() const override { return 7.62; }
    virtual double GetMaxSteeringAngle() const override { return 30.23 * CH_C_DEG_TO_RAD; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
        m_omega = omega;
    }

  protected:
    mrole_Vehicle(const std::string& name, ChContactMethod contactMethod, DrivelineTypeWV driveType)
        : ChWheeledVehicle(name, contactMethod), m_driveType(driveType), m_omega({0, 0, 0, 0}) {}

    mrole_Vehicle(const std::string& name, ChSystem* system, DrivelineTypeWV driveType)
        : ChWheeledVehicle(name, system), m_driveType(driveType), m_omega({0, 0, 0, 0}) {}

    DrivelineTypeWV m_driveType;
    std::vector<double> m_omega;
};

/// @} vehicle_models_hmmwv

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
