// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for a force element tire model
//
// =============================================================================

#ifndef CH_FORCEELEMENT_TIRE_H
#define CH_FORCEELEMENT_TIRE_H

#include "chrono/utils/ChUtils.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Base class for a force element (handling) tire model.
class CH_VEHICLE_API ChForceElementTire : public ChTire {
  public:
    /// Tire contact information.
    /// A ChForceElementTire performs its own collision detection to evaluate the normal tire load.
    /// This structure stores information on the current tire-terrain contact state.
    struct ContactData {
        ContactData() : in_contact(false), normal_force(0), depth(0) {}
        bool in_contact;      ///< true if tire in contact with terrain
        ChCoordsys<> frame;   ///< contact frame (x: long, y: lat, z: normal)
        ChVector3d vel;       ///< relative velocity expressed in contact frame
        double normal_force;  ///< magnitude of normal contact force
        double depth;         ///< penetration depth
    };

    virtual ~ChForceElementTire() {}

    /// Report the tire force and moment.
    /// The return application point, force, and moment are assumed to be expressed in the global reference frame.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    /// Report the tire forces expressed in the tire frame.
    /// The tire frame has its origin in the contact patch, the X axis in the tire heading direction and the Z axis in
    /// the terrain normal at the contact point.
    /// If the tire is not in contact, the tire frame is not set and the function returns zero force and moment.
    virtual TerrainForce ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const override;

    /// Report current tire-terrain contact information.
    /// If the tire is not in contact, all information is set to zero.
    const ContactData& ReportTireContactData() const { return m_data; }

    /// Enable/disable information terminal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Checkpoint the state of this tire to the given checkpint file.
    virtual void ExportCheckpoint(ChCheckpoint::Format format, const std::string& filename) const override;

    /// Initialize this tire from the given checkpoint file.
    virtual void ImportCheckpoint(ChCheckpoint::Format format, const std::string& filename) override;

  protected:
    /// Construct a tire with the specified name.
    ChForceElementTire(const std::string& name);

    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const = 0;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const = 0;

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the vehicle system.  Typically, the vehicle
    /// subsystem will pass the tire force to the appropriate suspension subsystem which applies it as an external force
    /// one the wheel body.
    virtual TerrainForce GetTireForce() const override;

    /// Get visualization width.
    virtual double GetVisualizationWidth() const = 0;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get current internal dynamics ODE states (if any).
    virtual void GetInternalStates(ChVector2d& states) const {}

    /// Set the internal dynamics ODE states (if any).
    virtual void SetInternalStates(const ChVector2d& states) {}

    ContactData m_data;          ///< tire-terrain collision information
    TerrainForce m_tireforce;    ///< tire forces (in tire contact frame)
    ChFunctionInterp m_areaDep;  ///< lookup table for estimation of penetration depth from intersection area

    std::shared_ptr<ChVisualShape> m_cyl_shape;  ///< visualization cylinder asset

    bool m_verbose;  ///< verbose output

  private:
    virtual void InitializeInertiaProperties() override final;
    virtual void UpdateInertiaProperties() override final;

    virtual double GetAddedMass() const override final;
    virtual ChVector3d GetAddedInertia() const override final;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
