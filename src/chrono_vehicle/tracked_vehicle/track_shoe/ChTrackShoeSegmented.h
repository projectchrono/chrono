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
// Base class for segmented track shoes.
//
// =============================================================================

#ifndef CH_TRACK_SHOE_SEGMENTED_H
#define CH_TRACK_SHOE_SEGMENTED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for segmented track shoes.
/// These are track shoes modeled with one or more rigid bodies connected through joints and/or bushings.
class CH_VEHICLE_API ChTrackShoeSegmented : public ChTrackShoe {
  public:
    virtual ~ChTrackShoeSegmented() {}

    /// Get the contact material for the track shoe part interacting with the sprocket.
    std::shared_ptr<ChMaterialSurface> GetSprocketContactMaterial() const { return m_shoe_sprk_material; }

  protected:
    ChTrackShoeSegmented(const std::string& name);

    /// Create the contact materials for the shoe, consistent with the specified contact method. A derived class must
    /// set m_shoe_sprk_material (used for contact with the sprocket) and m_shoe_materials which must include one or more
    /// contact materials for the collision shapes of the shoe itself (for contact with the wheels, idler, and ground).
    virtual void CreateContactMaterials(ChContactMethod contact_method) = 0;

    /// Activate or deactivate the RSDA elements used to model track bending stiffness.
    virtual void EnableTrackBendingStiffness(bool val) = 0;

    /// Add visualization assets for the track shoe subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the track shoe subsystem.
    virtual void RemoveVisualizationAssets() override;

    ChVehicleGeometry m_geometry;                             ///< collection of visualization and collision shapes
    std::shared_ptr<ChMaterialSurface> m_shoe_sprk_material;  ///< contact material for shoe shape contacting sprocket

    friend class ChTrackAssemblySegmented;
};

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
