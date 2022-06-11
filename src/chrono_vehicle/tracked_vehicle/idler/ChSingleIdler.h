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
// Base class for a single idler (template definition).
// A single idler is of type LATERAL_PIN.
//
// =============================================================================

#ifndef CH_SINGLE_IDLER_H
#define CH_SINGLE_IDLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_idler
/// @{

/// Base class for a single idler (template definition).
class CH_VEHICLE_API ChSingleIdler : public ChIdler {
  public:
    ChSingleIdler(const std::string& name  ///< [in] name of the subsystem
                  );

    virtual ~ChSingleIdler() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SingleIdler"; }

    /// Return the type of track shoe consistent with this idler.
    virtual GuidePinType GetType() const final override { return GuidePinType::LATERAL_PIN; }

    /// Initialize this idler subsystem.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
                            ) override;

    /// Add visualization assets for the idler subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the idler subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    /// Return the width of the idler wheel.
    virtual double GetWheelWidth() const = 0;
};

/// @} vehicle_tracked_idler

}  // end namespace vehicle
}  // end namespace chrono

#endif
