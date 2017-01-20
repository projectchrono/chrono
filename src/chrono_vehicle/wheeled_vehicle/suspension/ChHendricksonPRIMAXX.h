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
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#ifndef CH_HENDRICKSON_PRIMAXX_H
#define CH_HENDRICKSON_PRIMAXX_H

#include <vector>

#include "chrono/assets/ChColorAsset.h"

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
/// supspension and will be mirrored (reflecting the y coordinates) to construct
/// the right side.
class CH_VEHICLE_API ChHendricksonPRIMAXX : public ChSuspension {
  public:
    ChHendricksonPRIMAXX(const std::string& name  ///< [in] name of the subsystem
                         );

    virtual ~ChHendricksonPRIMAXX() {}

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return true; }

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const final override { return false; }

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

    /// There could be a spring (coil or air) and damper element between chassis and lower beam
    /// and a second spring and damper element between chassis and housing

    /// Spring (coil or air) and damper element between chassis and lower beam (LB)
    /// Get the force in the air spring (coil or spring) and a damper element

    /// Get the force in the spring-damper element.
    double GetShockLBForce(VehicleSide side) const { return m_shockLB[side]->Get_SpringReact(); }

    /// Get the current length of the spring-damper element.
    double GetShockLBLength(VehicleSide side) const { return m_shockLB[side]->Get_SpringLength(); }

    /// Get the current deformation velocity of the spring-damper element.
    double GetShockLBVelocity(VehicleSide side) const { return m_shockLB[side]->Get_SpringVelocity(); }

    /// Spring (coil or air) and damper element between chassis and axle housing (AH)
    /// Get the force in the air spring (coil or spring) and a damper element

    /// Get the force in the spring-damper element.
    double GetShockAHForce(VehicleSide side) const { return m_shockAH[side]->Get_SpringReact(); }

    /// Get the current length of the spring-damper element.
    double GetShockAHLength(VehicleSide side) const { return m_shockAH[side]->Get_SpringLength(); }

    /// Get the current deformation velocity of the spring-damper element.
    double GetShockAHVelocity(VehicleSide side) const { return m_shockAH[side]->Get_SpringVelocity(); }

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
        SPINDLE,            ///< spindle location
        KNUCKLE_L,          ///< lower knuckle point
        KNUCKLE_U,          ///< upper knuckle point
        TIEROD_C,           ///< tierod, chassis
        TIEROD_K,           ///< tierod, knuckle
        TORQUEROD_C,        ///< torquerod, chassis
        TORQUEROD_AH,       ///< torquerod, axle housing (AH)
        LOWERBEAM_C,        ///< lowerbeam, chassis
        LOWERBEAM_AH,       ///< lowerbeam, axle housing (AH)
        LOWERBEAM_TB,       ///< lowerbeam, transverse beam
        SHOCKAH_C,          ///< shock at axle housing (AH), chasis
        SHOCKAH_AH,         ///< shock at axle housing (AH), axle housing
        SHOCKLB_C,          ///< shock at lower beam (LB), chasis
        SHOCKLB_LB,         ///< shock at lower beam (LB), lower beam
        KNUCKLE_CM,         ///< knuckle, center of mass
        TORQUEROD_CM,       ///< torquerod, center of mass
        LOWERBEAM_CM,       ///< lowerbeam, center of mass
        TRANSVERSEBEAM_CM,  ///< transverse beam, center of mass
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
    /// Return the mass of the axlehousing body.
    virtual double getAxlehousingMass() const = 0;

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
    /// Return the moments of inertia of the axlehousing body.
    virtual const ChVector<>& getAxlehousingInertia() const = 0;

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
    /// Return the radius of the axlehousing body (visualization only).
    virtual double getAxlehousingRadius() const = 0;

    // Lower beam spring and damper

    /// Return the free (rest) length of the spring element.
    virtual double getShockLBRestLength() const = 0;
    /// Return the callback function for shock force.
    virtual ChSpringForceCallback* getShockLBForceCallback() const = 0;

    // Axlehousing spring and damper

    /// Return the free (rest) length of the spring element.
    virtual double getShockAHRestLength() const = 0;
    /// Return the callback function for shock force.
    virtual ChSpringForceCallback* getShockAHForceCallback() const = 0;

    std::shared_ptr<ChBody> m_knuckle[2];      ///< handles to the knuckle bodies (left/right)
    std::shared_ptr<ChBody> m_torquerod[2];    ///< handles to torquerod bodies (left/right)
    std::shared_ptr<ChBody> m_lowerbeam[2];    ///< handles to lowerbeam bodies (left/right)
    std::shared_ptr<ChBody> m_transversebeam;  ///< handles to transversebeam body
    std::shared_ptr<ChBody> m_axlehousing;     ///< handles to axlehousing body

    std::shared_ptr<ChLinkLockRevolute>
        m_revoluteKingpin[2];  ///< handles to the knuckle-axle housing revolute joints (left/right)
    std::shared_ptr<ChLinkLockSpherical>
        m_sphericalTorquerod[2];  ///< handles to the torque rod-axle housing spherical joints (left/right)

    std::shared_ptr<ChLinkLockRevolute>
      m_revoluteTorquerod[2];  ///< handles to the torquerod chasis revolute joints (left/right)

    std::shared_ptr<ChLinkLockSpherical>
        m_sphericalLowerbeam[2];  ///< handles to the lower beam-axle housing spherical joints (left/right)

    std::shared_ptr<ChLinkLockRevolute>
      m_revoluteLowerbeam[2];  ///< handles to the lowerbeam chasis revolute joints (left/right)

    std::shared_ptr<ChLinkLockSpherical>
        m_sphericalTB[2];  ///< handles to the transversebeam-lower beam spherical joints (left/right)
    std::shared_ptr<ChLinkDistance> m_distTierod[2];  ///< handles to the tierod distance constraints (left/right)

    std::shared_ptr<ChLinkSpringCB> m_shockLB[2];  ///< handles to the spring links (left/right)
    std::shared_ptr<ChLinkSpringCB> m_shockAH[2];  ///< handles to the spring links (left/right)

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
                        std::shared_ptr<ChBodyAuxRef> chassis,
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

    static const std::string m_pointNames[NUM_POINTS];
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
