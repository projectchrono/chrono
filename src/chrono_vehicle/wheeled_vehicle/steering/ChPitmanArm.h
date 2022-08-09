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
// Base class for a Pitman Arm steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// The steering subsystem is modeled with respect to a right-handed frame with
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
//
// When attached to a chassis, both an offset and a rotation (as a quaternion)
// are provided.
//
// =============================================================================

#ifndef CH_PITMANARM_H
#define CH_PITMANARM_H

#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_steering
/// @{

/// Base class for a Pitman Arm steering subsystem.
///
/// The steering subsystem is modeled with respect to a right-handed frame with
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
///
/// When attached to a chassis, both an offset and a rotation (as a quaternion)
/// are provided.
class CH_VEHICLE_API ChPitmanArm : public ChSteering {
  public:
    virtual ~ChPitmanArm();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "PitmanArm"; }

    /// Initialize this steering subsystem.
    /// The steering subsystem is initialized by attaching it to the specified chassis at the specified location (with
    /// respect to and expressed in the reference frame of the chassis) and with specified orientation (with respect to
    /// the chassis reference frame).
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis subsystem
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation       ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Add visualization assets for the steering subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the steering subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Update the state of this steering subsystem at the current time.
    /// The steering subsystem is provided the current steering driver input (a value between -1 and +1).  Positive
    /// steering input indicates steering to the left. This function is called during the vehicle update.
    virtual void Synchronize(double time,                           ///< [in] current time
                             const DriverInputs& driver_inputs  ///< [in] current driver inputs
                             ) override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        STEERINGLINK,  ///< steering link location (com)
        PITMANARM,     ///< Pitman arm location (com)
        REV,           ///< location of joint between Pitman arm and chassis
        UNIV,          ///< location of joint between Pitman arm and steering link
        REVSPH_R,      ///< location of revolute joint for the idler arm
        REVSPH_S,      ///< location of spherical joint for the idler arm
        TIEROD_PA,     ///< tierod connection point (Pitman arm side)
        TIEROD_IA,     ///< tierod connection point (idler arm side)
        NUM_POINTS
    };

    /// Identifiers for the various direction unit vectors.
    enum DirectionId {
        REV_AXIS,        ///< revolute joint
        UNIV_AXIS_ARM,   ///< universal joint (Pitman arm side)
        UNIV_AXIS_LINK,  ///< universal joint (steering link side)
        REVSPH_AXIS,     ///< revolute joint for idler arm
        NUM_DIRS
    };

    /// Protected constructor.
    ChPitmanArm(const std::string& name,            ///< [in] name of the subsystem
                bool vehicle_frame_inertia = false  ///< [in] inertia specified in vehicle-aligned centroidal frames?
                );

    /// Indicate whether or not inertia matrices are specified with respect to a
    /// vehicle-aligned centroidal frame (flag=true) or with respect to the body
    /// centroidal frame (flag=false).  Note that this function must be called
    /// before Initialize().
    void SetVehicleFrameInertiaFlag(bool val) { m_vehicle_frame_inertia = val; }

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;
    /// Return the unit vector for the specified direction.
    /// The returned vector must be expressed in the suspension reference frame.
    virtual const ChVector<> getDirection(DirectionId which) = 0;

    /// Return the mass of the steering link body.
    virtual double getSteeringLinkMass() const = 0;
    /// Return the mass of the Pitman arm body.
    virtual double getPitmanArmMass() const = 0;

    /// Return the moments of inertia of the steering link body.
    virtual const ChVector<>& getSteeringLinkInertiaMoments() const = 0;
    /// Return the products of inertia of the steering link body.
    virtual const ChVector<>& getSteeringLinkInertiaProducts() const = 0;

    /// Return the moments of inertia of the Pitman arm body.
    virtual const ChVector<>& getPitmanArmInertiaMoments() const = 0;
    /// Return the products of inertia of the Pitman arm body.
    virtual const ChVector<>& getPitmanArmInertiaProducts() const = 0;

    /// Return the radius of the steering link body (visualization only).
    virtual double getSteeringLinkRadius() const = 0;
    /// Return the radius of the Pitman arm body (visualization only).
    virtual double getPitmanArmRadius() const = 0;

    /// Return the maximum rotation angle of the revolute joint.
    virtual double getMaxAngle() const = 0;

    std::shared_ptr<ChBody> m_arm;  ///< handle to the Pitman arm body

    std::shared_ptr<ChLinkMotorRotationAngle> m_revolute;  ///< handle to the chassis-arm revolute joint
    std::shared_ptr<ChLinkRevoluteSpherical> m_revsph;     ///< handle to the revolute-spherical joint (idler arm)
    std::shared_ptr<ChLinkUniversal> m_universal;          ///< handle to the arm-link universal joint

  private:
    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    // Flag indicating that the inertia matrices for the upright and control arms
    // are provided in vehicle-aligned centroidal frames
    bool m_vehicle_frame_inertia;

    // Points for link visualization
    ChVector<> m_pP;
    ChVector<> m_pI;
    ChVector<> m_pTP;
    ChVector<> m_pTI;

    // Points for arm visualization
    ChVector<> m_pC;
    ChVector<> m_pL;
};

/// @} vehicle_wheeled_steering

}  // end namespace vehicle
}  // end namespace chrono

#endif
