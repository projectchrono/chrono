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
// Base class for an anti-roll bar subsystem
//
// =============================================================================

#ifndef CH_ANTIROLLBAR_H
#define CH_ANTIROLLBAR_H

#include <string>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_antirollbar
/// @{

/// Base class for an anti-roll bar subsystem.
class CH_VEHICLE_API ChAntirollBar : public ChPart {
  public:
    ChAntirollBar(const std::string& name  ///< [in] name of the subsystem
                  );

    virtual ~ChAntirollBar() {}

    /// Initialize this anti-roll bar subsystem.
    /// The anti-roll bar subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always aligned with the chassis reference frame.
    /// Finally, susp_body_left and susp_body_right are handles to the suspension
    /// bodies to which the anti-roll bar's droplinks are to be attached.
    virtual void Initialize(
        std::shared_ptr<ChBodyAuxRef> chassis,   ///< [in] handle to the chassis body
        const ChVector<>& location,              ///< [in] location relative to the chassis frame
        std::shared_ptr<ChBody> susp_body_left,  ///< [in] susp body to which left droplink is connected
        std::shared_ptr<ChBody> susp_body_right  ///< [in] susp body to which right droplink is connected
        ) = 0;

    /// Get the total mass of the anti-roll bar subsystem.
    virtual double GetMass() const = 0;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() {}
};

/// Vector of handles to antirollbar subsystems.
typedef std::vector<std::shared_ptr<ChAntirollBar> > ChAntirollbarList;

/// @} vehicle_wheeled_antirollbar

}  // end namespace vehicle
}  // end namespace chrono

#endif
