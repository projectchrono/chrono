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
// Base class for all vehicle subsystems.
//
// =============================================================================

#ifndef CH_PART_H
#define CH_PART_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Base class for a terrain system.
class CH_VEHICLE_API ChPart {
  public:
    /// Construct a vehicle subsystem with the specified name.
    ChPart(const std::string& name  ///< [in] name of the subsystem
           );

    virtual ~ChPart() {}

    /// Get the name identifier for this track shoe subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this track shoe subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Set the visualization mode for this subsystem.
    void SetVisualizationType(VisualizationType vis);

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) {}

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() {}

  protected:
    std::string m_name;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
