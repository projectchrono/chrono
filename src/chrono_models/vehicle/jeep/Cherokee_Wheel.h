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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Class for modeling a wheel for Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef CHRONO_CHEROKEE_WHEEL_H
#define CHRONO_CHEROKEE_WHEEL_H

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// Jeep Cherokee wheel (can be used on any axle, left or right).
class CH_MODELS_API Cherokee_Wheel : public ChWheel {
  public:
    Cherokee_Wheel(const std::string& name);
    ~Cherokee_Wheel() {}

    virtual double GetWheelMass() const override { return m_mass; }
    virtual const ChVector3d& GetWheelInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

    virtual void Construct(std::shared_ptr<ChChassis> chassis,  ///< chassis vehicle (may be null)
                           std::shared_ptr<ChSpindle> spindle,  ///< associated suspension spindle
                           VehicleSide side,                    ///< wheel mounted on left/right side
                           double offset                        ///< offset from associated spindle center
                           ) override;

  protected:
    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector3d m_inertia;
};

/// @} vehicle_models_cherokee

}  // end namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_WHEEL_H
