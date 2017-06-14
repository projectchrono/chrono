// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for the HMMWV vehicle models
//
// =============================================================================

#ifndef HMMWV_VEHICLE_H
#define HMMWV_VEHICLE_H

#include <vector>
#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV_Vehicle : public ChWheeledVehicle {
  public:
    virtual ~HMMWV_Vehicle() {}

    virtual int GetNumberAxles() const override { return 2; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
        m_omega = omega;
    }

  protected:
    HMMWV_Vehicle(const std::string& name, ChMaterialSurface::ContactMethod contactMethod, DrivelineType driveType)
        : ChWheeledVehicle(name, contactMethod), m_driveType(driveType), m_omega({0, 0, 0, 0}) {}

    HMMWV_Vehicle(const std::string& name, ChSystem* system, DrivelineType driveType)
        : ChWheeledVehicle(name, system), m_driveType(driveType), m_omega({0, 0, 0, 0}) {}

    DrivelineType m_driveType;
    std::vector<double> m_omega;
};

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
