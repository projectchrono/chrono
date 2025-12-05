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
// Template for a balancer subchassis system.
//
// =============================================================================

#ifndef CH_BALANCER_H
#define CH_BALANCER_H

#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/wheeled_vehicle/ChSubchassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_subchassis
/// @{

/// Template for a balancer subchassis system.
class CH_VEHICLE_API ChBalancer : public ChSubchassis {
  public:
    virtual ~ChBalancer();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Balancer"; }

    /// Construct the balancer subchassis subsystem.
    /// The subchassis is initialized by attaching it to the specified chassis at the specified location (with respect
    /// to and expressed in the reference frame of the chassis). It is assumed that the subchassis reference frame is
    /// always aligned with the chassis reference frame.
    virtual void Construct(std::shared_ptr<ChChassis> chassis,  ///< [in] chassis
                           const ChVector3d& location           ///< [in] location relative to the chassis frame
                           ) override;

    /// Add visualization assets for the balancer subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the balancer subsystem.
    virtual void RemoveVisualizationAssets() override;

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        BEAM,      ///< beam location
        REVOLUTE,  ///< location of revolute joint
        NUM_POINTS
    };

    ChBalancer(const std::string& name);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the subchassis reference frame.
    virtual const ChVector3d GetLocation(PointId which) = 0;

    /// Return the direction of the pin joint (default: Y axis).
    virtual const ChVector3d GetDirection() { return ChVector3d(0, 1, 0); }

    /// Get the balancer mass.
    virtual double GetBalancerBeamMass() const = 0;

    /// Get the inertia tensor of the chassis body.
    virtual const ChVector3d& GetBalancerBeamInertia() const = 0;

    /// Get the rear balancer min/max pitch.
    /// Used only if using a kinematic joint.
    virtual const double GetBalancerMaxPitch() const = 0;

    ///  Get balancer dimensions (for visualization).
    virtual const ChVector3d& GetBalancerBeamDimensions() const = 0;

    /// Return stiffness and damping data for the balancer bushing.
    /// Returning nullptr (default) results in using a kinematic revolute joint.
    virtual std::shared_ptr<ChJoint::BushingData> GetBushingData() const { return nullptr; }

    std::shared_ptr<ChJoint> m_balancer_joint[2];  ///< balancer pivot joints

  private:
    void InitializeSide(VehicleSide sid,
                        std::shared_ptr<ChChassis> chassis,
                        const std::vector<ChVector3d>& points,
                        const ChVector3d& dir);

    virtual void PopulateComponentList() override;

    // Hardpoint absolute locations
    std::vector<ChVector3d> m_pointsL;
    std::vector<ChVector3d> m_pointsR;

    // Pin directions
    ChVector3d m_dirL;
    ChVector3d m_dirR;
};

/// @} vehicle_wheeled_subchassis

}  // end namespace vehicle
}  // end namespace chrono

#endif
