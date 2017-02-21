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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Base class for a double-A arm suspension modeled with bodies and constraints.
// Derived from ChSuspension, but still an abstract base class.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#ifndef CH_DOUBLEWISHBONE_H
#define CH_DOUBLEWISHBONE_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a double-A arm suspension modeled with bodies and constraints.
/// Derived from ChSuspension, but still an abstract base class.
///
/// The suspension subsystem is modeled with respect to a right-handed frame,
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The suspension reference frame is assumed to be always aligned with that of
/// the vehicle.  When attached to a chassis, only an offset is provided.
///
/// All point locations are assumed to be given for the left half of the
/// supspension and will be mirrored (reflecting the y coordinates) to construct
/// the right side.
class CH_VEHICLE_API ChDoubleWishbone : public ChSuspension {
  public:
    ChDoubleWishbone(const std::string& name  ///< [in] name of the subsystem
                     );

    virtual ~ChDoubleWishbone() {}

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return true; }

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const final override { return true; }

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always aligned with the chassis reference frame.
    /// Finally, tierod_body is a handle to the body to which the suspension
    /// tierods are to be attached. For a steerable suspension, this will be the
    /// steering link of a suspension subsystem.  Otherwise, this is the chassis.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            std::shared_ptr<ChBody> tierod_body,    ///< [in] body to which tireods are connected
                            double left_ang_vel = 0,                ///< [in] initial angular velocity of left wheel
                            double right_ang_vel = 0                ///< [in] initial angular velocity of right wheel
                            ) override;

    /// Add visualization assets for the suspension subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the suspension subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the total mass of the suspension subsystem.
    virtual double GetMass() const override;

    /// Get the force in the spring element.
    double GetSpringForce(VehicleSide side) const { return m_spring[side]->Get_SpringReact(); }

    /// Get the current length of the spring element
    double GetSpringLength(VehicleSide side) const { return m_spring[side]->Get_SpringLength(); }

    /// Get the current deformation of the spring element.
    double GetSpringDeformation(VehicleSide side) const { return m_spring[side]->Get_SpringDeform(); }

    /// Get the force in the shock (damper) element.
    double GetShockForce(VehicleSide side) const { return m_shock[side]->Get_SpringReact(); }

    /// Get the current length of the shock (damper) element.
    double GetShockLength(VehicleSide side) const { return m_shock[side]->Get_SpringLength(); }

    /// Get the current deformation velocity of the shock (damper) element.
    double GetShockVelocity(VehicleSide side) const { return m_shock[side]->Get_SpringVelocity(); }

    /// Global coordinates, LCA ball joint position
    ChVector<> Get_LCA_sph_pos(VehicleSide side) { return m_sphericalLCA[side]->GetMarker2()->GetAbsCoord().pos; }

    /// Global coordinates, UCA ball joint position
    ChVector<> Get_UCA_sph_pos(VehicleSide side) { return m_sphericalUCA[side]->GetMarker2()->GetAbsCoord().pos; }

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

    /// Specify the left body for a possible antirollbar subsystem.
    /// Return a handle to the left Lower Control Arm.
    virtual std::shared_ptr<ChBody> GetLeftBody() const override { return m_LCA[0]; }

    /// Specify the right body for a possible antirollbar subsystem.
    /// Return a handle to the right Lower Control Arm.
    virtual std::shared_ptr<ChBody> GetRightBody() const override { return m_LCA[1]; }

    /// Log the locations of all hardpoints.
    /// The reported locations are expressed in the suspension reference frame.
    /// By default, these values are reported in SI units (meters), but can be
    /// optionally reported in inches.
    void LogHardpointLocations(const ChVector<>& ref, bool inches = false);

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        SPINDLE,   ///< spindle location
        UPRIGHT,   ///< upright location
        UCA_F,     ///< upper control arm, chassis front
        UCA_B,     ///< upper control arm, chassis back
        UCA_U,     ///< upper control arm, upright
        UCA_CM,    ///< upper control arm, center of mass
        LCA_F,     ///< lower control arm, chassis front
        LCA_B,     ///< lower control arm, chassis back
        LCA_U,     ///< lower control arm, upright
        LCA_CM,    ///< lower control arm, center of mass
        SHOCK_C,   ///< shock, chassis
        SHOCK_A,   ///< shock, lower control arm
        SPRING_C,  ///< spring, chassis
        SPRING_A,  ///< spring, lower control arm
        TIEROD_C,  ///< tierod, chassis
        TIEROD_U,  ///< tierod, upright
        NUM_POINTS
    };

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;

    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the mass of the upper control arm body.
    virtual double getUCAMass() const = 0;
    /// Return the mass of the lower control body.
    virtual double getLCAMass() const = 0;
    /// Return the mass of the upright body.
    virtual double getUprightMass() const = 0;

    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;
    /// Return the moments of inertia of the upper control arm body.
    virtual const ChVector<>& getUCAInertia() const = 0;
    /// Return the moments of inertia of the lower control arm body.
    virtual const ChVector<>& getLCAInertia() const = 0;
    /// Return the moments of inertia of the upright body.
    virtual const ChVector<>& getUprightInertia() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the radius of the upper control arm body (visualization only).
    virtual double getUCARadius() const = 0;
    /// Return the radius of the lower control arm body (visualization only).
    virtual double getLCARadius() const = 0;
    /// Return the radius of the upright body (visualization only).
    virtual double getUprightRadius() const = 0;

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const = 0;
    /// Return the callback function for spring force.
    virtual ChSpringForceCallback* getSpringForceCallback() const = 0;
    /// Return the callback function for shock force.
    virtual ChSpringForceCallback* getShockForceCallback() const = 0;

    std::shared_ptr<ChBody> m_upright[2];  ///< handles to the upright bodies (left/right)
    std::shared_ptr<ChBody> m_UCA[2];      ///< handles to the upper control arm bodies (left/right)
    std::shared_ptr<ChBody> m_LCA[2];      ///< handles to the lower control arm bodies (left/right)

    std::shared_ptr<ChLinkLockRevolute> m_revoluteUCA[2];    ///< handles to the chassis-UCA revolute joints (left/right)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalUCA[2];  ///< handles to the upright-UCA spherical joints (left/right)
    std::shared_ptr<ChLinkLockRevolute> m_revoluteLCA[2];    ///< handles to the chassis-LCA revolute joints (left/right)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalLCA[2];  ///< handles to the upright-LCA spherical joints (left/right)
    std::shared_ptr<ChLinkDistance> m_distTierod[2];         ///< handles to the tierod distance constraints (left/right)

    std::shared_ptr<ChLinkSpringCB> m_shock[2];   ///< handles to the spring links (left/right)
    std::shared_ptr<ChLinkSpringCB> m_spring[2];  ///< handles to the shock links (left/right)

  private:
    // Hardpoint absolute locations
    std::vector<ChVector<>> m_pointsL;
    std::vector<ChVector<>> m_pointsR;

    void InitializeSide(VehicleSide side,
                        std::shared_ptr<ChBodyAuxRef> chassis,
                        std::shared_ptr<ChBody> tierod_body,
                        const std::vector<ChVector<> >& points,
                        double ang_vel);

    static void AddVisualizationControlArm(std::shared_ptr<ChBody> arm,
                                           const ChVector<> pt_F,
                                           const ChVector<> pt_B,
                                           const ChVector<> pt_U,
                                           double radius);
    static void AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                        const ChVector<> pt_C,
                                        const ChVector<> pt_U,
                                        const ChVector<> pt_L,
                                        const ChVector<> pt_T,
                                        double radius);

    static const std::string m_pointNames[NUM_POINTS];
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
