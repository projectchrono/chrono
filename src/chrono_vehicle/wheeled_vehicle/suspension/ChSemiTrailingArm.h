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
// Base class for a semi-trailing arm suspension (non-steerable).
// Derived from ChSuspension, but still an abstract base class.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// suspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#ifndef CH_SEMITRAILINGARM_H
#define CH_SEMITRAILINGARM_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a semi-trailing arm suspension (non-steerable).
/// Derived from ChSuspension, but still an abstract base class.
///
/// The suspension subsystem is modeled with respect to a right-handed frame,
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The suspension reference frame is assumed to be always aligned with that of
/// the vehicle.  When attached to a chassis, only an offset is provided.
///
/// All point locations are assumed to be given for the left half of the
/// suspension and will be mirrored (reflecting the y coordinates) to construct
/// the right side.
class CH_VEHICLE_API ChSemiTrailingArm : public ChSuspension {
  public:
    ChSemiTrailingArm(const std::string& name  ///< [in] name of the subsystem
    );

    virtual ~ChSemiTrailingArm();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SemiTrailingArm"; }

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return false; }

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const final override { return true; }

    /// Construct this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified chassis and (if provided) to the
    /// specified subchassis, at the specified location (with respect to and expressed in the reference frame of the
    /// chassis). It is assumed that the suspension reference frame is always aligned with the chassis reference frame.
    /// Since this suspension is non-steerable, the steering subsystem is always ignored.
    virtual void Construct(
        std::shared_ptr<ChChassis> chassis,        ///< [in] associated chassis subsystem
        std::shared_ptr<ChSubchassis> subchassis,  ///< [in] associated subchassis subsystem (may be null)
        std::shared_ptr<ChSteering> steering,      ///< [in] associated steering subsystem (may be null)
        const ChVector3d& location,                ///< [in] location relative to the chassis frame
        double left_ang_vel,                       ///< [in] initial angular velocity of left wheel
        double right_ang_vel                       ///< [in] initial angular velocity of right wheel
        ) override;

    /// Add visualization assets for the suspension subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the suspension subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the wheel track for the suspension subsystem.
    virtual double GetTrack() override;

    /// Get a handle to the specified spring element.
    std::shared_ptr<ChLinkTSDA> GetSpring(VehicleSide side) const { return m_spring[side]; }

    /// Get a handle to the specified shock (damper) element.
    std::shared_ptr<ChLinkTSDA> GetShock(VehicleSide side) const { return m_shock[side]; }

    /// Return current suspension TSDA force information on the specified side.
    virtual std::vector<ForceTSDA> ReportSuspensionForce(VehicleSide side) const override;

    /// Get the force in the spring element.
    double GetSpringForce(VehicleSide side) const { return m_spring[side]->GetForce(); }

    /// Get the current length of the spring element
    double GetSpringLength(VehicleSide side) const { return m_spring[side]->GetLength(); }

    /// Get the current deformation of the spring element.
    double GetSpringDeformation(VehicleSide side) const { return m_spring[side]->GetDeformation(); }

    /// Get the force in the shock (damper) element.
    double GetShockForce(VehicleSide side) const { return m_shock[side]->GetForce(); }

    /// Get the current length of the shock (damper) element.
    double GetShockLength(VehicleSide side) const { return m_shock[side]->GetLength(); }

    /// Get the current deformation velocity of the shock (damper) element.
    double GetShockVelocity(VehicleSide side) const { return m_shock[side]->GetVelocity(); }

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

    /// Specify the suspension body on the specified side to attach a possible antirollbar subsystem.
    /// Return the corresponding trailing arm.
    virtual std::shared_ptr<ChBody> GetAntirollBody(VehicleSide side) const override { return m_arm[side]; }

    /// Log the locations of all hardpoints.
    /// The reported locations are expressed in the suspension reference frame.
    /// By default, these values are reported in SI units (meters), but can be
    /// optionally reported in inches.
    void LogHardpointLocations(const ChVector3d& ref, bool inches = false);

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        SPINDLE,   ///< spindle location (center of mass)
        TA_CM,     ///< trailing arm, center of mass
        TA_O,      ///< trailing arm, chassis connection outter
        TA_I,      ///< trailing arm, chassis connection inner
        TA_S,      ///< trailing arm, connection to spindle
        SHOCK_C,   ///< shock, chassis
        SHOCK_A,   ///< shock, lower control arm
        SPRING_C,  ///< spring, chassis
        SPRING_A,  ///< spring, lower control arm
        NUM_POINTS
    };

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector3d getLocation(PointId which) = 0;

    /// Return the camber angle, in radians (default: 0).
    virtual double getCamberAngle() const { return 0; }

    /// Return the toe angle, in radians (default: 0).
    /// A positive value indicates toe-in, a negative value indicates toe-out.
    virtual double getToeAngle() const { return 0; }

    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the mass of the trailing arm body.
    virtual double getArmMass() const = 0;

    /// Return the moments of inertia of the spindle body.
    virtual const ChVector3d& getSpindleInertia() const = 0;
    /// Return the moments of inertia of the trailing arm body.
    virtual const ChVector3d& getArmInertia() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the radius of the trailing arm body (visualization only).
    virtual double getArmRadius() const = 0;

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const = 0;
    /// Return the free (rest) length of the shock element.
    virtual double getShockRestLength() const { return 0; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const = 0;
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const = 0;

    /// Return stiffness and damping data for the arm bushing.
    /// Returning nullptr (default) results in using a kinematic revolute joint.
    virtual std::shared_ptr<ChJoint::BushingData> getCABushingData() const { return nullptr; }

    std::shared_ptr<ChBody> m_arm[2];  ///< trailing arm bodies (left/right)

    std::shared_ptr<ChJoint> m_revoluteArm[2];  ///< chassis-arm revolute joints (left/right)

    std::shared_ptr<ChLinkTSDA> m_shock[2];   ///< spring links (left/right)
    std::shared_ptr<ChLinkTSDA> m_spring[2];  ///< shock links (left/right)

  private:
    // Hardpoint absolute locations
    std::vector<ChVector3d> m_pointsL;
    std::vector<ChVector3d> m_pointsR;

    void InitializeSide(VehicleSide side,
                        std::shared_ptr<ChChassis> chassis,
                        const std::vector<ChVector3d>& points,
                        double ang_vel);

    static void AddVisualizationArm(std::shared_ptr<ChBody> arm,
                                    const ChVector3d pt_AC_O,
                                    const ChVector3d pt_AC_I,
                                    const ChVector3d pt_AS,
                                    const ChVector3d pt_S,
                                    double radius);

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    static const std::string m_pointNames[NUM_POINTS];
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
