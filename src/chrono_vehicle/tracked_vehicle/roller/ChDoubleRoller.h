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
// Base class for a double roller (template definition).
// A double roller is of type CENTRAL_PIN.
//
// =============================================================================

#ifndef CH_DOUBLE_ROLLER_H
#define CH_DOUBLE_ROLLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChRoller.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_roller
/// @{

/// Base class for a double roller (template definition).
class CH_VEHICLE_API ChDoubleRoller : public ChRoller {
  public:
    ChDoubleRoller(const std::string& name  ///< [in] name of the subsystem
                   );

    virtual ~ChDoubleRoller() {}

    /// Return the type of track shoe consistent with this roller.
    virtual GuidePinType GetType() const final override { return GuidePinType::CENTRAL_PIN; }

    /// Initialize this roller subsystem.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            ) override;

    /// Add visualization assets for the roller subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the roller subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    /// Return the total width of the roller.
    virtual double GetWidth() const = 0;
    /// Return the gap width.
    virtual double GetGap() const = 0;
};

/// @} vehicle_tracked_roller

}  // end namespace vehicle
}  // end namespace chrono

#endif
