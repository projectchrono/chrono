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
// Template for a rigid-body chassis vehicle subsystem.
//
// =============================================================================

#ifndef CH_RIGID_CHASSIS_H
#define CH_RIGID_CHASSIS_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Template for a rigid-body chassis vehicle subsystem.
class CH_VEHICLE_API ChRigidChassis : public ChChassis {
  public:
    /// Construct a vehicle subsystem with the specified name.
    ChRigidChassis(const std::string& name,  ///< [in] name of the subsystem
                   bool fixed = false        ///< [in] is the chassis body fixed to ground?
                   );

    virtual ~ChRigidChassis() {}

    /// Specifies whether or not visualization primitives were specified.
    bool HasPrimitives() const { return m_has_primitives; }

    /// Specifies whether or not a visualization mesh was specified.
    bool HasMesh() const { return m_has_mesh; }

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    bool m_has_primitives;
    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
