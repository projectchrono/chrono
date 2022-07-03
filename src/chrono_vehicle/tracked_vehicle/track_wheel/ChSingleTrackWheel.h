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
// Base class for a single track wheel (template definition).
// A single track wheel is of type LATERAL_PIN.
//
// =============================================================================

#ifndef CH_SINGLE_TRACK_WHEEL_H
#define CH_SINGLE_TRACK_WHEEL_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackWheel.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Base class for a single track wheel (template definition).
class CH_VEHICLE_API ChSingleTrackWheel : public ChTrackWheel {
  public:
    ChSingleTrackWheel(const std::string& name  ///< [in] name of the subsystem
    );

    virtual ~ChSingleTrackWheel() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SingleTrackWheel"; }

    /// Return the type of track shoe consistent with this track wheel.
    virtual GuidePinType GetType() const final override { return GuidePinType::LATERAL_PIN; }

    /// Initialize this track wheel subsystem.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis subsystem
                            std::shared_ptr<ChBody> carrier,     ///< [in] carrier body
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
                            ) override;

    /// Add visualization assets for the track-wheel subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the track-wheel subsystem.
    virtual void RemoveVisualizationAssets() override final;
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
