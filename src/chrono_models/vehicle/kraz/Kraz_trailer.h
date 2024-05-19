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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Trailer for the Kraz tractor-trailer vehicle model.
//
// =============================================================================

#ifndef KRAZ_TRAILER_H
#define KRAZ_TRAILER_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// @addtogroup vehicle_models_kraz
/// @{

/// Kraz trailer system.
class CH_MODELS_API Kraz_trailer : public ChWheeledTrailer {
  public:
    Kraz_trailer(ChSystem* system, CollisionType chassis_collision_type = CollisionType::NONE);

    ~Kraz_trailer() {}

    virtual unsigned int GetNumberAxles() const override { return 3; }

    double GetSpringForce(int axle, VehicleSide side) const;
    double GetSpringLength(int axle, VehicleSide side) const;
    double GetSpringDeformation(int axle, VehicleSide side) const;

    double GetShockForce(int axle, VehicleSide side) const;
    double GetShockLength(int axle, VehicleSide side) const;
    double GetShockVelocity(int axle, VehicleSide side) const;

    virtual void Initialize(std::shared_ptr<ChChassis> frontChassis) override;
};

/// @} vehicle_models_kraz

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
