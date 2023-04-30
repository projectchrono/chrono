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

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_antirollbar
/// @{

/// Base class for an anti-roll bar subsystem.
class CH_VEHICLE_API ChAntirollBar : public ChPart {
  public:
    virtual ~ChAntirollBar() {}

    /// Initialize this anti-roll bar subsystem.
    /// The anti-roll bar subsystem is initialized by attaching it to the specified chassis at the given location (with
    /// respect to and expressed in the reference frame of the chassis) and associating it with the specified suspension
    /// subsystem (assumed to be independent). It is assumed that the anti-roll bar reference frame is always aligned
    /// with the chassis reference frame.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,        ///< [in] handle to the chassis body
                            std::shared_ptr<ChSuspension> suspension,  ///< [in] associated suspension subsystem
                            const ChVector<>& location                 ///< [in] location relative to the chassis frame
    );

    /// Log current constraint violations.
    virtual void LogConstraintViolations() {}

  protected:
    /// Construct an anti-roll bar subsystem with given name.
    ChAntirollBar(const std::string& name);

    ChVector<> m_rel_loc;  ///< relative location of antiroll bar subsystem on chassis
};

/// Vector of handles to antirollbar subsystems.
typedef std::vector<std::shared_ptr<ChAntirollBar> > ChAntirollbarList;

/// @} vehicle_wheeled_antirollbar

}  // end namespace vehicle
}  // end namespace chrono

#endif
