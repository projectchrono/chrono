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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Base class for a Hendrickson PRIMAXX EX suspension.
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

#ifndef CH_HENDRICKSON_PRIMAXX_H
#define CH_HENDRICKSON_PRIMAXX_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a Hendrickson PRIMAXX EX suspension.
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
class CH_VEHICLE_API ChHendricksonPRIMAXX : public ChSuspension {
  public:
    virtual ~ChHendricksonPRIMAXX();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "HendricksonPRIMAXX"; }

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

    /// Return current suspension forces (spring and shock) on the specified side.
    /// For this particular suspension, the return struct contains information about the LB force element in its
    /// "spring" members and information about the AH force element in its "shock" members.
    virtual ChSuspension::Force ReportSuspensionForce(VehicleSide side) const override;

    /// There could be a spring (coil or air) and damper element between chassis and lower beam
    /// and a second spring and damper element between chassis and housing

    /// Spring (coil or air) and damper element between chassis and lower beam (LB)
    /// Get the force in the air spring (coil or spring) and a damper element

    /// Get the force in the spring-damper element.
    double GetShockLBForce(VehicleSide side) const { return m_shockLB[side]->GetForce(); }

    /// Get the current length of the spring-damper element.
    double GetShockLBLength(VehicleSide side) const { return m_shockLB[side]->GetLength(); }

    /// Get the current deformation velocity of the spring-damper element.
    double GetShockLBVelocity(VehicleSide side) const { return m_shockLB[side]->GetVelocity(); }

    /// Spring (coil or air) and damper element between chassis and axle housing (AH)
    /// Get the force in the air spring (coil or spring) and a damper element

    /// Get the force in the spring-damper element.
    double GetShockAHForce(VehicleSide side) const { return m_shockAH[side]->GetForce(); }

    /// Get the current length of the spring-damper element.
    double GetShockAHLength(VehicleSide side) const { return m_shockAH[side]->GetLength(); }

    /// Get the current deformation velocity of the spring-damper element.
    double GetShockAHVelocity(VehicleSide side) const { return m_shockAH[side]->GetVelocity(); }

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

    /// Log the locations of all hardpoints.
    /// The reported locations are expressed in the suspension reference frame.
    /// By default, these values are reported in SI units (meters), but can be
    /// optionally reported in inches.
    void LogHardpointLocations(const ChVector<>& ref, bool inches = false);

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        SPINDLE,       ///< spindle location
        KNUCKLE_L,     ///< lower knuckle point
        KNUCKLE_U,     ///< upper knuckle point
        TIEROD_C,      ///< tierod, chassis
        TIEROD_K,      ///< tierod, knuckle
        TORQUEROD_C,   ///< torquerod, chassis
        TORQUEROD_AH,  ///< torquerod, axle housing (AH)
        LOWERBEAM_C,   ///< lowerbeam, chassis
        LOWERBEAM_AH,  ///< lowerbeam, axle housing (AH)
        LOWERBEAM_TB,  ///< lowerbeam, transverse beam
        SHOCKAH_C,     ///< shock at axle housing (AH), chasis
        SHOCKAH_AH,    ///< shock at axle housing (AH), axle housing
        SHOCKLB_C,     ///< shock at lower beam (LB), chasis
        SHOCKLB_LB,    ///< shock at lower beam (LB), lower beam
        KNUCKLE_CM,    ///< knuckle, center of mass
        TORQUEROD_CM,  ///< torquerod, center of mass
        LOWERBEAM_CM,  ///< lowerbeam, center of mass
        NUM_POINTS
    };

    /// Identifiers for the various vectors.
    enum DirectionId {
        UNIV_AXIS_LOWERBEAM_BEAM,     ///< universal joint (lowerbeam, beam side)
        UNIV_AXIS_LOWERBEAM_CHASSIS,  ///< universal joint (lowerbeam, chassis side)
        UNIV_AXIS_TORQUEROD_ROD,      ///< universal joint (torquerod, rod side)
        UNIV_AXIS_TORQUEROD_CHASSIS,  ///< universal joint (torquerod, chasis side)
        NUM_DIRS
    };

    ChHendricksonPRIMAXX(const std::string& name  ///< [in] name of the subsystem
    );

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Indicate whether or not tirod bodies are modelled (default: false).
    /// If false, tierods are modelled using distance constraints.
    /// If true, rigid tierod bodies are created (in which case a derived class must provide the mass and inertia) and
    /// connected either with kinematic joints or bushings (depending on whether or not bushing data is defined).
    virtual bool UseTierodBodies() const { return false; }

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;

    ///// Return the vector of the specified direction.
    virtual const ChVector<> getDirection(DirectionId which) = 0;

    /// Return the center of mass of the axle tube.
    virtual const ChVector<> getAxlehousingCOM() const = 0;

    /// Return the center of mass of the transverse beam.
    virtual const ChVector<> getTransversebeamCOM() const = 0;

    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the mass of the knuckle body.
    virtual double getKnuckleMass() const = 0;
    /// Return the mass of the torque rod body.
    virtual double getTorquerodMass() const = 0;
    /// Return the mass of the lower beam body.
    virtual double getLowerbeamMass() const = 0;
    /// Return the mass of the transverse beam body.
    virtual double getTransversebeamMass() const = 0;
    /// Return the mass of the axle housing body.
    virtual double getAxlehousingMass() const = 0;
    /// Return the mass of the tierod body.
    virtual double getTierodMass() const { return 0; }

    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;
    /// Return the moments of inertia of the knuckle body.
    virtual const ChVector<>& getKnuckleInertia() const = 0;
    /// Return the moments of inertia of the torque rod body.
    virtual const ChVector<>& getTorquerodInertia() const = 0;
    /// Return the moments of inertia of the lower beam body.
    virtual const ChVector<>& getLowerbeamInertia() const = 0;
    /// Return the moments of inertia of the transverse beam body.
    virtual const ChVector<>& getTransversebeamInertia() const = 0;
    /// Return the moments of inertia of the axle housing body.
    virtual const ChVector<>& getAxlehousingInertia() const = 0;
    /// Return the moments of inertia of the tierod body.
    virtual const ChVector<> getTierodInertia() const { return ChVector<>(0); }

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the radius of the knuckle body (visualization only).
    virtual double getKnuckleRadius() const = 0;
    /// Return the radius of the torque rod body (visualization only).
    virtual double getTorquerodRadius() const = 0;
    /// Return the radius of the lower beam body (visualization only).
    virtual double getLowerbeamRadius() const = 0;
    /// Return the radius of the transverse beam body (visualization only).
    virtual double getTransversebeamRadius() const = 0;
    /// Return the radius of the axle housing body (visualization only).
    virtual double getAxlehousingRadius() const = 0;
    /// Return the radius of the tierod body (visualization only).
    virtual double getTierodRadius() const { return 0; }

    // Lower beam spring and damper

    /// Return the free (rest) length of the spring element.
    virtual double getShockLBRestLength() const = 0;
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockLBForceCallback() const = 0;

    // Axle housing spring and damper

    /// Return the free (rest) length of the spring element.
    virtual double getShockAHRestLength() const = 0;
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockAHForceCallback() const = 0;

    /// Return stiffness and damping data for the tierod bushings.
    /// Used only if tierod bodies are defined (see UseTierodBody).
    /// Returning nullptr (default) results in using kinematic joints (spherical + universal).
    virtual std::shared_ptr<ChVehicleBushingData> getTierodBushingData() const { return nullptr; }

    std::shared_ptr<ChBody> m_knuckle[2];      ///< the knuckle bodies (left/right)
    std::shared_ptr<ChBody> m_torquerod[2];    ///< torquerod bodies (left/right)
    std::shared_ptr<ChBody> m_lowerbeam[2];    ///< lowerbeam bodies (left/right)
    std::shared_ptr<ChBody> m_transversebeam;  ///< transversebeam body
    std::shared_ptr<ChBody> m_axlehousing;     ///< axlehousing body
    std::shared_ptr<ChBody> m_tierod[2];       ///< tierod bodies, if used (left/right)

    std::shared_ptr<ChLinkLockRevolute> m_revoluteKingpin[2];      ///< knuckle-axle housing joints (left/right)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalTorquerod[2];  ///< torquerod-axle housing joints (left/right)
    std::shared_ptr<ChLinkLockRevolute> m_revoluteTorquerod[2];    ///< torquerod-chasis joints (left/right)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalLowerbeam[2];  ///< lowerbeam-axle housing joints (left/right)
    std::shared_ptr<ChLinkLockRevolute> m_revoluteLowerbeam[2];    ///< lowerbeam chasis joints (left/right)
    std::shared_ptr<ChLinkLockSpherical> m_sphericalTB[2];         ///< transversebeam-lower beam joints (left/right)

    std::shared_ptr<ChLinkDistance> m_distTierod[2];       ///< tierod distance constraints (left/right)
    std::shared_ptr<ChVehicleJoint> m_sphericalTierod[2];  ///< tierod-upright spherical joints (left/right)
    std::shared_ptr<ChVehicleJoint> m_universalTierod[2];  ///< tierod-chassis universal joints (left/right)

    std::shared_ptr<ChLinkTSDA> m_shockLB[2];  ///< spring links (left/right)
    std::shared_ptr<ChLinkTSDA> m_shockAH[2];  ///< spring links (left/right)

  private:
    // Hardpoint absolute locations and directions
    std::vector<ChVector<>> m_pointsL;
    std::vector<ChVector<>> m_pointsR;

    std::vector<ChVector<>> m_dirsL;
    std::vector<ChVector<>> m_dirsR;

    // Points for link visualization
    ChVector<> m_outerL;
    ChVector<> m_outerR;

    void InitializeSide(VehicleSide side,
                        std::shared_ptr<ChChassis> chassis,
                        std::shared_ptr<ChBody> tierod_body,
                        const std::vector<ChVector<>>& points,
                        const std::vector<ChVector<>>& dirs,
                        double ang_vel);

    static void AddVisualizationLink(std::shared_ptr<ChBody> body,
                                     const ChVector<> pt_1,
                                     const ChVector<> pt_2,
                                     double radius,
                                     const ChColor& color);
    static void AddVisualizationLowerBeam(std::shared_ptr<ChBody> body,
                                          const ChVector<> pt_C,
                                          const ChVector<> pt_AH,
                                          const ChVector<> pt_TB,
                                          double radius,
                                          const ChColor& color);
    static void AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                        const ChVector<> pt_U,
                                        const ChVector<> pt_L,
                                        const ChVector<> pt_T,
                                        double radius);
    static void AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
                                       const ChVector<> pt_C,
                                       const ChVector<> pt_U,
                                       double radius);

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    static const std::string m_pointNames[NUM_POINTS];
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
