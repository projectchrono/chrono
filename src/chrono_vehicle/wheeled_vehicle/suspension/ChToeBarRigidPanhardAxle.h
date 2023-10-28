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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Base class for a steerable solid Panhard axle suspension.
//
// This class is meant for modelling a very simple steerable solid Panhard
// axle. The guiding function is modelled by a ChLinkLockRevolutePrismatic joint
// which allows vertical movement and tilting of the axle tube but no elasticity.
//
// This axle subsystem works with the ChRotaryArm steering subsystem.
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

#ifndef CH_TOEBAR_RIGID_PANHARD_AXLE_H
#define CH_TOEBAR_RIGID_PANHARD_AXLE_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a steerable solid Panhard axle suspension.
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
class CH_VEHICLE_API ChToeBarRigidPanhardAxle : public ChSuspension {
  public:
    ChToeBarRigidPanhardAxle(const std::string& name  ///< [in] name of the subsystem
    );

    virtual ~ChToeBarRigidPanhardAxle();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ToeBarRigidPanhardAxle"; }

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return true; }

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const final override { return false; }

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified chassis and (if provided) to the
    /// specified subchassis, at the specified location (with respect to and expressed in the reference frame of the
    /// chassis). It is assumed that the suspension reference frame is always aligned with the chassis reference frame.
    /// If a steering subsystem is provided, the suspension tierods are to be attached to the steering's central link
    /// body (steered suspension); otherwise they are to be attached to the chassis (non-steered suspension).
    virtual void Initialize(
        std::shared_ptr<ChChassis> chassis,        ///< [in] associated chassis subsystem
        std::shared_ptr<ChSubchassis> subchassis,  ///< [in] associated subchassis subsystem (may be null)
        std::shared_ptr<ChSteering> steering,      ///< [in] associated steering subsystem (may be null)
        const ChVector<>& location,                ///< [in] location relative to the chassis frame
        double left_ang_vel = 0,                   ///< [in] initial angular velocity of left wheel
        double right_ang_vel = 0                   ///< [in] initial angular velocity of right wheel
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

    /// Get the current turning angle of the kingpin joint
    double GetKingpinAngleLeft() { return m_revoluteKingpin[0]->GetRelAngle(); }
    double GetKingpinAngleRight() { return m_revoluteKingpin[1]->GetRelAngle(); }

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

    void LogHardpointLocations(const ChVector<>& ref, bool inches = false);

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        SHOCK_A,      ///< shock, axle
        SHOCK_C,      ///< shock, chassis
        KNUCKLE_L,    ///< lower knuckle point
        KNUCKLE_U,    ///< upper knuckle point
        KNUCKLE_DRL,  ///< knuckle to draglink
        SPRING_A,     ///< spring, axle
        SPRING_C,     ///< spring, chassis
        TIEROD_K,     ///< tierod, knuckle
        SPINDLE,      ///< spindle location
        KNUCKLE_CM,   ///< knuckle, center of mass
        DRAGLINK_C,   ///< draglink, chassis
        PANHARD_A,    ///< panhard rod, axle
        PANHARD_C,    ///< panhard rod, chassis
        ANTIROLL_A,   ///< connector antirollbar on axle tube
        ANTIROLL_C,   ///< connector antirollbar on chassis
        NUM_POINTS
    };

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;

    /// Return the camber angle, in radians (default: 0).
    virtual double getCamberAngle() const { return 0; }

    /// Return the toe angle, in radians (default: 0).
    /// A positive value indicates toe-in, a negative value indicates toe-out.
    virtual double getToeAngle() const { return 0; }

    /// Return the center of mass of the axle tube.
    virtual const ChVector<> getAxleTubeCOM() const = 0;

    /// Return the mass of the axle tube body.
    virtual double getAxleTubeMass() const = 0;
    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the mass of the knuckle body.
    virtual double getKnuckleMass() const = 0;
    /// Return the mass of the tierod body.
    virtual double getTierodMass() const = 0;
    /// Return the mass of the draglink body.
    virtual double getDraglinkMass() const = 0;
    /// Return the mass of the SRB body.
    virtual double getARBMass() const = 0;

    /// Return the radius of the axle tube body (visualization only).
    virtual double getAxleTubeRadius() const = 0;
    /// Return the radius of the knuckle body (visualization only).
    virtual double getKnuckleRadius() const = 0;
    /// Return the radius of the tierod body (visualization only).
    virtual double getTierodRadius() const = 0;
    /// Return the radius of the draglink body (visualization only).
    virtual double getDraglinkRadius() const = 0;
    /// Return the radius of the ARB body (visualization only).
    virtual double getARBRadius() const = 0;

    /// Return the moments of inertia of the axle tube body.
    virtual const ChVector<>& getAxleTubeInertia() const = 0;
    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;
    /// Return the moments of inertia of the knuckle body.
    virtual const ChVector<>& getKnuckleInertia() const = 0;
    /// Return the moments of inertia of the tierod body.
    virtual const ChVector<>& getTierodInertia() const = 0;
    /// Return the moments of inertia of the draglink body.
    virtual const ChVector<>& getDraglinkInertia() const = 0;
    /// Return the moments of inertia of the ARB body.
    virtual const ChVector<>& getARBInertia() const = 0;

    /// Return the striffness of the ARB system.
    virtual double getARBStiffness() const = 0;

    /// Return the striffness of the ARB system.
    virtual double getARBDamping() const = 0;

    /// Return the mass of the Panhard tube body.
    virtual double getPanhardRodMass() const = 0;

    /// Return the radius of the panhard rod body (visualization only).
    virtual double getPanhardRodRadius() const = 0;

    /// Return the moments of inertia of the Panhard rod body.
    virtual const ChVector<>& getPanhardRodInertia() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const = 0;
    /// Return the free (rest) length of the shock element.
    virtual double getShockRestLength() const { return 0; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const = 0;
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const = 0;

    /// Returns toplology flag for knuckle/draglink connection
    virtual bool isLeftKnuckleActuated() { return true; }

    std::shared_ptr<ChBody> m_axleTubeBody;    ///< handles to the axle tube body
    std::shared_ptr<ChBody> m_tierodBody;      ///< handles to the tierod body
    std::shared_ptr<ChBody> m_draglinkBody;    ///< handles to the draglink body
    std::shared_ptr<ChBody> m_knuckleBody[2];  ///< handles to the knuckle bodies (L/R)
    std::shared_ptr<ChBody> m_panhardRodBody;  ///< handles to the panhard rod body
    std::shared_ptr<ChBody> m_arbBody[2];      ///< handles to the antiroll bar bodies

    std::shared_ptr<ChLinkLockPlanePlane> m_axleTubeGuide;     ///< allows translation Y,Z and rotation X
    std::shared_ptr<ChLinkLockSpherical> m_sphericalTierod;    ///< knuckle-tierod spherical joint (left)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalDraglink;  ///< draglink-chassis spherical joint (left)
    std::shared_ptr<ChLinkUniversal> m_universalDraglink;      ///< draglink-bellCrank universal joint (left)
    std::shared_ptr<ChLinkUniversal> m_universalTierod;        ///< knuckle-tierod universal joint (right)
    std::shared_ptr<ChLinkLockRevolute> m_revoluteKingpin[2];  ///< knuckle-axle tube revolute joints (L/R)
    std::shared_ptr<ChVehicleJoint> m_sphPanhardAxle;
    std::shared_ptr<ChVehicleJoint> m_sphPanhardChassis;

    std::shared_ptr<ChVehicleJoint> m_revARBChassis;
    std::shared_ptr<ChLinkLockRevolute> m_revARBLeftRight;
    std::shared_ptr<ChVehicleJoint> m_slideARB[2];

    std::shared_ptr<ChLinkTSDA> m_shock[2];   ///< handles to the spring links (L/R)
    std::shared_ptr<ChLinkTSDA> m_spring[2];  ///< handles to the shock links (L/R)

  private:
    // Hardpoint absolute locations
    std::vector<ChVector<>> m_pointsL;
    std::vector<ChVector<>> m_pointsR;

    // Points for axle tube visualization
    ChVector<> m_axleOuterL;
    ChVector<> m_axleOuterR;

    // Points for tierod visualization
    ChVector<> m_tierodOuterL;
    ChVector<> m_tierodOuterR;

    // Points for Panhard rod visualization
    ChVector<> m_panrodOuterA;
    ChVector<> m_panrodOuterC;

    // Points for antiroll bar visualization
    ChVector<> m_ptARBAxle[2];
    ChVector<> m_ptARBChassis[2];
    ChVector<> m_ptARBCenter;

    // Left or right knuckle is actuated by draglink?
    bool m_left_knuckle_steers;

    void InitializeSide(VehicleSide side,
                        std::shared_ptr<ChChassis> chassis,
                        const std::vector<ChVector<>>& points,
                        double ang_vel);

    static void AddVisualizationLink(std::shared_ptr<ChBody> body,
                                     const ChVector<> pt_1,
                                     const ChVector<> pt_2,
                                     double radius,
                                     const ChColor& color);

    static void AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                        const ChVector<> pt_U,
                                        const ChVector<> pt_L,
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
