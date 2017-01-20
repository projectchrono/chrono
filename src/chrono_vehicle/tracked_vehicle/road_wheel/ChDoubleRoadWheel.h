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
// Base class for a double road wheel (template definition).
// A double road wheel is of type CENTRAL_PIN.
//
// =============================================================================

#ifndef CH_DOUBLE_ROAD_WHEEL_H
#define CH_DOUBLE_ROAD_WHEEL_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChRoadWheel.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Base class for a double road wheel (template definition).
class CH_VEHICLE_API ChDoubleRoadWheel : public ChRoadWheel {
  public:
    ChDoubleRoadWheel(const std::string& name  ///< [in] name of the subsystem
                      );

    virtual ~ChDoubleRoadWheel() {}

    /// Return the type of track shoe consistent with this road wheel.
    virtual GuidePinType GetType() const final override { return GuidePinType::CENTRAL_PIN; }

    /// Initialize this road wheel subsystem.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            std::shared_ptr<ChBody> carrier,        ///< [in] handle to the carrier body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            ) override;

    /// Add visualization assets for the road-wheel subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the road-wheel subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    /// Return the total width of the road wheel.
    virtual double GetWheelWidth() const = 0;
    /// Return the gap width.
    virtual double GetWheelGap() const = 0;
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
