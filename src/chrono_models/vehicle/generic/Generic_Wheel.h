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
// Generic wheel subsystem
//
// =============================================================================

#ifndef GENERIC_WHEEL_H
#define GENERIC_WHEEL_H

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Wheel subsystem for the generic vehicle.
class CH_MODELS_API Generic_Wheel : public ChWheel {
  public:
    Generic_Wheel(const std::string& name);
    ~Generic_Wheel() {}

    virtual double GetWheelMass() const override { return m_mass; }
    virtual const ChVector<>& GetWheelInertia() const override { return m_inertia; }

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< chassis vehicle (may be null)
                            std::shared_ptr<ChBody> spindle,     ///< associated suspension spindle body
                            VehicleSide side,                    ///< wheel mounted on left/right side
                            double offset = 0                    ///< offset from associated spindle center
                            ) override;

  private:
    static const double m_mass;
    static const ChVector<> m_inertia;
    static const double m_radius;
    static const double m_width;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
