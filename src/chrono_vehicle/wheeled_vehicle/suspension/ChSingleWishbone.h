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
// Base class for a single-A arm suspension modeled with bodies and constraints.
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

#ifndef CH_SINGLEWISHBONE_H
#define CH_SINGLEWISHBONE_H

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
/// suspension and will be mirrored (reflecting the y coordinates) to construct
/// the right side.
class CH_VEHICLE_API ChSingleWishbone : public ChSuspension {
  public:
    virtual ~ChSingleWishbone() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SingleWishbone"; }

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return true; }

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const final override { return true; }

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always aligned with the chassis reference frame.
    /// 'tierod_body' is a handle to the body to which the suspension tierods
    /// are to be attached. For a steered suspension, this will be the steering
    /// (central) link of a suspension subsystem.  Otherwise, this is the chassis.
    /// If this suspension is steered, 'steering_index' indicates the index of the
    /// associated steering mechanism in the vehicle's list (-1 for a non-steered suspension).
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            std::shared_ptr<ChBody> tierod_body,    ///< [in] body to which tireods are connected
                            int steering_index,                     ///< [in] index of the associated steering mechanism
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

    /// Get the current global COM location of the suspension subsystem.
    virtual ChVector<> GetCOMPos() const override;

    /// Get the wheel track for the suspension subsystem.
    virtual double GetTrack() override;
 
    /// Get a handle to the specified shock (damper) element.
    std::shared_ptr<ChLinkTSDA> GetShock(VehicleSide side) const { return m_shock[side]; }

    /// Return current suspension forces (spring and shock) on the specified side.
    virtual ChSuspension::Force ReportSuspensionForce(VehicleSide side) const override;

    /// Get the force in the shock (damper) element.
    double GetShockForce(VehicleSide side) const { return m_shock[side]->GetForce(); }

    /// Get the current length of the shock (damper) element.
    double GetShockLength(VehicleSide side) const { return m_shock[side]->GetLength(); }

    /// Get the current deformation velocity of the shock (damper) element.
    double GetShockVelocity(VehicleSide side) const { return m_shock[side]->GetVelocity(); }

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

    /// Specify the left body for a possible antirollbar subsystem.
    /// Return a handle to the left Control Arm.
    virtual std::shared_ptr<ChBody> GetLeftBody() const override { return m_control_arm[0]; }

    /// Specify the right body for a possible antirollbar subsystem.
    /// Return a handle to the right Control Arm.
    virtual std::shared_ptr<ChBody> GetRightBody() const override { return m_control_arm[1]; }

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
        CA_C,      ///< control arm, chassis
        CA_U,      ///< control arm, upright
        CA_CM,     ///< control arm, center of mass
        STRUT_C,   ///< strut, chassis
        STRUT_A,   ///< strut, control arm
        TIEROD_C,  ///< tierod, chassis
        TIEROD_U,  ///< tierod, upright
        NUM_POINTS
    };

    /// Protected constructor.
    ChSingleWishbone(const std::string& name);

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;

    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the mass of the upper control arm body.
    virtual double getCAMass() const = 0;
    /// Return the mass of the upright body.
    virtual double getUprightMass() const = 0;

    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;

    /// Return the moments of inertia of the control arm body.
    virtual const ChVector<>& getCAInertiaMoments() const = 0;
    /// Return the products of inertia of the control arm body.
    virtual const ChVector<>& getCAInertiaProducts() const = 0;

    /// Return the moments of inertia of the upright body.
    virtual const ChVector<>& getUprightInertiaMoments() const = 0;
    /// Return the products of inertia of the upright body.
    virtual const ChVector<>& getUprightInertiaProducts() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the radius of the control arm body (visualization only).
    virtual double getCARadius() const = 0;
    /// Return the radius of the upright body (visualization only).
    virtual double getUprightRadius() const = 0;

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const = 0;
    /// Return the functor object for spring-damper force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const = 0;

    std::shared_ptr<ChBody> m_upright[2];      ///< handles to the upright bodies (left/right)
    std::shared_ptr<ChBody> m_control_arm[2];  ///< handles to the control arm bodies (left/right)

    std::shared_ptr<ChLinkLockRevolute> m_revoluteCA[2];  ///< handles to the chassis-CA revolute joints (left/right)
    std::shared_ptr<ChLinkLockRevolute> m_revoluteUA[2];  ///< handles to the upright-CA revolute joints (left/right)
    std::shared_ptr<ChLinkDistance> m_distTierod[2];      ///< handles to the tierod distance constraints (left/right)

    std::shared_ptr<ChLinkTSDA> m_shock[2];   ///< handles to the spring-damper (left/right)

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
                                           const ChVector<> pt_C,
                                           const ChVector<> pt_U,
                                           double radius);
    static void AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                        const ChVector<> pt_U,
                                        const ChVector<> pt_S,
                                        const ChVector<> pt_A,
                                        const ChVector<> pt_T,
                                        double radius);

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    static const std::string m_pointNames[NUM_POINTS];
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
