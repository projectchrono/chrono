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
// Base class for a 3-link independent rear suspension (non-steerable).
// This suspension has a trailing arm and 2 additional links connecting the
// arm to the chassis.
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

#ifndef CH_THREELINK_IRS_H
#define CH_THREELINK_IRS_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a 3-link independent rear suspension (non-steerable).
/// This suspension has a trailing arm and 2 additional links connecting the
/// arm to the chassis.
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
class CH_VEHICLE_API ChThreeLinkIRS : public ChSuspension {
  public:
    ChThreeLinkIRS(const std::string& name  ///< [in] name of the subsystem
                   );

    virtual ~ChThreeLinkIRS() {}

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return false; }

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
    virtual void Initialize(
        std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
        const ChVector<>& location,             ///< [in] location relative to the chassis frame
        std::shared_ptr<ChBody> tierod_body,    ///< [in] body to which tireods are connected (ignored)
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

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

    /// Specify the left body for a possible antirollbar subsystem.
    /// Return a handle to the left trailing arm.
    virtual std::shared_ptr<ChBody> GetLeftBody() const override { return m_arm[0]; }

    /// Specify the right body for a possible antirollbar subsystem.
    /// Return a handle to the right trailing arm.
    virtual std::shared_ptr<ChBody> GetRightBody() const override { return m_arm[1]; }

    /// Log the locations of all hardpoints.
    /// The reported locations are expressed in the suspension reference frame.
    /// By default, these values are reported in SI units (meters), but can be
    /// optionally reported in inches.
    void LogHardpointLocations(const ChVector<>& ref, bool inches = false);

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        SPINDLE,   ///< spindle location (center of mass)
        TA_CM,     ///< trailing arm, center of mass
        UL_CM,     ///< upper link, center of mass
        LL_CM,     ///< lower link, center of mass
        TA_C,      ///< trailing arm, connection to chassis
        TA_S,      ///< trailing arm, connection to spindle
        UL_C,      ///< upper link, connection to chassis
        UL_A,      ///< upper link, connection to arm
        LL_C,      ///< lower link, connection to chassis
        LL_A,      ///< lower link, connection to arm
        SHOCK_C,   ///< shock, chassis
        SHOCK_A,   ///< shock, trailing arm
        SPRING_C,  ///< spring, chassis
        SPRING_A,  ///< spring, trailing arm
        NUM_POINTS
    };

    /// Identifiers for various joint directions.
    enum DirectionId {
        UNIV_AXIS_UPPER,  ///< universal joint (upper link - chassis)
        UNIV_AXIS_LOWER,  ///< universal joint (lower link - chassis)
        NUM_DIRS
    };

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;

    /// Return the vector of the specified direction.
    virtual const ChVector<> getDirection(DirectionId which) = 0;

    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the mass of the trailing arm body.
    virtual double getArmMass() const = 0;
    /// Return the mass of the upper link.
    virtual double getUpperLinkMass() const = 0;
    /// Return the mass of the lower link.
    virtual double getLowerLinkMass() const = 0;

    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;

    /// Return the moments of inertia of the trailing arm body.
    /// The x direction is aligned with the arm.
    virtual const ChVector<>& getArmInertia() const = 0;

    /// Return the moments of inertia of the upper link.
    /// The x direction is aligned with the link.
    virtual const ChVector<>& getUpperLinkInertia() const = 0;

    /// Return the moments of inertia of the lower link.
    /// The x direction is aligned with the link.
    virtual const ChVector<>& getLowerLinkInertia() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the radius of the trailing arm body (visualization only).
    virtual double getArmRadius() const = 0;

    /// Return the radius of the upper link body (visualization only).
    virtual double getUpperLinkRadius() const = 0;

    /// Return the radius of the lower link body (visualization only).
    virtual double getLowerLinkRadius() const = 0;

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const = 0;
    /// Return the callback function for spring force.
    virtual ChSpringForceCallback* getSpringForceCallback() const = 0;
    /// Return the callback function for shock force.
    virtual ChSpringForceCallback* getShockForceCallback() const = 0;

    std::shared_ptr<ChBody> m_arm[2];    ///< handles to the trailing arm bodies (left/right)
    std::shared_ptr<ChBody> m_upper[2];  ///< handles to the upper links (left/right)
    std::shared_ptr<ChBody> m_lower[2];  ///< handles to the lower links (left/right)

    std::shared_ptr<ChLinkLockSpherical> m_sphericalArm[2];    ///< chassis-arm spherical joints (left/right)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalUpper[2];  ///< upper-arm spherical joints (left/right)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalLower[2];  ///< lower-arm spherical joints (left/right)
    std::shared_ptr<ChLinkUniversal> m_universalUpper[2];      ///< upper-chassis universal joints (left/right)
    std::shared_ptr<ChLinkUniversal> m_universalLower[2];      ///< lower-chassis universal joints (left/right)

    std::shared_ptr<ChLinkSpringCB> m_shock[2];   ///< handles to the spring links (left/right)
    std::shared_ptr<ChLinkSpringCB> m_spring[2];  ///< handles to the shock links (left/right)

  private:
    std::vector<ChVector<>> m_pointsL;  ///< hardpoint locations (left)
    std::vector<ChVector<>> m_pointsR;  ///< hardpoint locations (right)

    std::vector<ChVector<>> m_dirsL;  ///< joint directions (left)
    std::vector<ChVector<>> m_dirsR;  ///< joint directions (right)

    void InitializeSide(VehicleSide side,
                        std::shared_ptr<ChBodyAuxRef> chassis,
                        const std::vector<ChVector<>>& points,
                        const std::vector<ChVector<>>& dirs,
                        double ang_vel);

    static void AddVisualizationArm(std::shared_ptr<ChBody> body,
                                    const ChVector<>& pt_C,
                                    const ChVector<>& pt_S,
                                    const ChVector<>& pt_CM,
                                    const ChVector<>& pt_U,
                                    const ChVector<>& pt_L,
                                    double radius);

    static void AddVisualizationLink(std::shared_ptr<ChBody> body,
                                     const ChVector<>& pt_1,
                                     const ChVector<>& pt_2,
                                     const ChVector<>& pt_CM,
                                     double radius);

    static const std::string m_pointNames[NUM_POINTS];
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
