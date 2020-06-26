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
// Base class for a sub-chassis system for wheeled vehicles.
//
// =============================================================================

#ifndef CH_SUBCHASSIS_H
#define CH_SUBCHASSIS_H

#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_subchassis
/// @{

/// Base class for a sub-chassis system for wheeled vehicles.
class CH_VEHICLE_API ChSubchassis : public ChPart {
  public:
    ChSubchassis(const std::string& name  ///< [in] name of the subsystem
    );

    virtual ~ChSubchassis() {}

    /// Get the location of the subchassis relative to the chassis reference frame.
    /// The subchassis reference frame is always aligned with the chassis reference frame.
    const ChVector<>& GetLocation() const { return m_location; }

    /// Get a handle to the beam body on the specified side.
    std::shared_ptr<ChBody> GetBeam(VehicleSide side) const { return m_beam[side]; }

    /// Get the total mass of the subchassis subsystem.
    virtual double GetMass() const = 0;

    /// Get the current global COM location of the subchassis subsystem.
    virtual ChVector<> GetCOMPos() const = 0;

    /// Initialize this subchassis subsystem.
    /// The subchassis is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the subchassis
    /// reference frame is always aligned with the chassis reference frame.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            ) = 0;

  protected:
    ChVector<> m_location;              ///< location relative to chassis
    std::shared_ptr<ChBody> m_beam[2];  ///< handles to beam bodies
};

/// Vector of handles to subchassis subsystems.
typedef std::vector<std::shared_ptr<ChSubchassis>> ChSubchassisList;

/// @} vehicle_wheeled_subchassis

}  // end namespace vehicle
}  // end namespace chrono

#endif
