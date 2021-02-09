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
    virtual ~ChBalancer() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Balancer"; }

    /// Initialize this subchassis subsystem.
    /// The subchassis is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the subchassis
    /// reference frame is always aligned with the chassis reference frame.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            ) override;

    /// Get the total mass of the subchassis subsystem.
    virtual double GetMass() const override;

    /// Get the current global COM location of the subchassis subsystem.
    virtual ChVector<> GetCOMPos() const override;

    /// Add visualization assets for the balancer subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the balancer subsystem.
    virtual void RemoveVisualizationAssets() override;

  protected:
    ChBalancer(const std::string& name);

    /// Identifiers for the various hardpoints.
    enum PointId {
        BEAM,      ///< beam location
        REVOLUTE,  ///< location of revolute joint
        NUM_POINTS
    };

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the subchassis reference frame.
    virtual const ChVector<> GetLocation(PointId which) = 0;

    /// Get the balancer mass.
    virtual double GetBalancerBeamMass() const = 0;

    /// Get the inertia tensor of the chassis body.
    virtual const ChVector<>& GetBalancerBeamInertia() const = 0;

    /// Get the rear balancer min/max pitch.
    virtual const double GetBalancerMaxPitch() const = 0;

    ///  Get balancer dimensions (for visualization).
    virtual const ChVector<>& GetBalancerBeamDimensions() const = 0;

    std::shared_ptr<ChLinkLockRevolute> m_balancer_joint[2];  ///< balancer pivot joints

private:
    // Hardpoint absolute locations
    std::vector<ChVector<>> m_pointsL;
    std::vector<ChVector<>> m_pointsR;
};

/// @} vehicle_wheeled_subchassis

}  // end namespace vehicle
}  // end namespace chrono

#endif
