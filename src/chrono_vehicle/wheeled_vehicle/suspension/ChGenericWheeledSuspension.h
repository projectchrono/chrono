// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Base class for a generic wheeled vehicle suspension. Except for the spindle
// bodies and the associated revolute joints, the topology of such a suspension
// is completely arbitrary and left to derived classes.
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

#ifndef CH_GENERIC_WHEELED_SUSPENSION_H
#define CH_GENERIC_WHEELED_SUSPENSION_H

#include <vector>
#include <unordered_map>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a generic wheeled vehicle suspension. Except for the spindle
/// bodies and the associated revolute joints, the topology of such a suspension
/// is completely arbitrary and left to derived classes.
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
class CH_VEHICLE_API ChGenericWheeledSuspension : public ChSuspension {
  public:
    /// Identification of a body in the suspension subsystem.
    struct BodyIdentifier {
        BodyIdentifier(const std::string& part_name,
                       int side = -1,
                       bool is_chassis = false,
                       bool is_subchassis = false,
                       bool is_steering = false);
        BodyIdentifier();
        std::string name;  ///< name of part
        int side;          ///< side of part (if not the same)
        bool chassis;      ///< true if this is a vehicle chassis body
        bool subchassis;   ///< true if this is a vehicle subchassis body
        bool steering;     ///< true if this is a vehicle steering link
    };

    /// Identification of the vehicle chassis body.
    struct ChassisIdentifier : public BodyIdentifier {
        ChassisIdentifier();
    };

    /// Identification of a vehicle subchassis body.
    struct SubchassisIdentifier : public BodyIdentifier {
        SubchassisIdentifier();
    };

    /// Identification of a vehicle steering link body.
    struct SteeringIdentifier : public BodyIdentifier {
        SteeringIdentifier();
    };

    virtual ~ChGenericWheeledSuspension();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "GenericWheeledSuspension"; }

    /// Add a body definition to the suspension subsystem.
    void DefineBody(
        const std::string& name,             ///< body name
        bool mirrored,                       ///< true if mirrored on right side
        const ChVector<>& pos,               ///< body COM position (in subsystem reference frame)
        const ChQuaternion<>& rot,           ///< body frame orientation (in subsystem reference frame)
        double mass,                         ///< body mass
        const ChVector<>& inertia_moments,   ///< body moments of inertia (relative to COG frame)
        const ChVector<>& inertia_products,  ///< body products of inertia (relative to COG frame)
        std::shared_ptr<ChVehicleGeometry> geometry = nullptr  ///< optional collision and visualization geometry
    );

    /// Add a joint definition to the suspension subsystem.
    /// If bushing data is provided, creates a bushing. Otherwise, creates a kinematic joint.
    void DefineJoint(const std::string& name,    ///< joint name
                     bool mirrored,              ///< true if mirrored on right side
                     ChVehicleJoint::Type type,  ///< joint type
                     BodyIdentifier body1,       ///< first connected body
                     BodyIdentifier body2,       ///< second connected body
                     const ChVector<>& pos,      ///< joint position (in subsystem reference frame)
                     const ChQuaternion<>& rot,  ///< joint frame orientation (in subsystem reference frame)
                     std::shared_ptr<ChVehicleBushingData> bdata = nullptr  ///< optional bushing data
    );

    /// Add a distance constraint definition to the suspension subsystem.
    void DefineDistanceConstraint(const std::string& name,   ///< constraint name
                                  bool mirrored,             ///< true if mirrored on right side
                                  BodyIdentifier body1,      ///< first connected body
                                  BodyIdentifier body2,      ///< second connected body
                                  const ChVector<>& point1,  ///< point on body1 (in subsystem reference frame)
                                  const ChVector<>& point2   ///< point on body2 (in subsystem reference frame)
    );

    /// Add a TSDA to model a suspension spring or shock.
    void DefineTSDA(const std::string& name,                          ///< TSDA name
                    bool mirrored,                                    ///< true if mirrored on right side
                    BodyIdentifier body1,                             ///< first connected body
                    BodyIdentifier body2,                             ///< second connected body
                    const ChVector<>& point1,                         ///< point on body1 (in subsystem reference frame)
                    const ChVector<>& point2,                         ///< point on body2 (in subsystem reference frame)
                    double rest_length,                               ///< rest (free) length
                    std::shared_ptr<ChLinkTSDA::ForceFunctor> force,  ///< functor for TSDA force evaluation
                    std::shared_ptr<ChTSDAGeometry> geometry = nullptr  ///< optional visualization geometry
    );

    /// Add an RSDA to model a suspension spring or shock.
    void DefineRSDA(const std::string& name,                           ///< RSDA name
                    bool mirrored,                                     ///< true if mirrored on right side
                    BodyIdentifier body1,                              ///< first connected body
                    BodyIdentifier body2,                              ///< second connected body
                    const ChVector<>& pos,                             ///< RSDA position (in subsystem reference frame)
                    const ChVector<>& axis,                            ///< axis of action for the RSDA
                    double rest_angle,                                 ///< rest (free) angle
                    std::shared_ptr<ChLinkRSDA::TorqueFunctor> torque  ///< functor for RSDA torque evaluation
    );

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified chassis and (if provided) to the
    /// specified subchassis, at the specified location (with respect to and expressed in the reference frame of the
    /// chassis). It is assumed that the suspension reference frame is always aligned with the chassis reference frame.
    virtual void Initialize(
        std::shared_ptr<ChChassis> chassis,        ///< associated chassis subsystem
        std::shared_ptr<ChSubchassis> subchassis,  ///< associated subchassis subsystem (may be null)
        std::shared_ptr<ChSteering> steering,      ///< associated steering subsystem (may be null)
        const ChVector<>& location,                ///< location relative to the chassis frame
        double left_ang_vel = 0,                   ///< initial angular velocity of left wheel
        double right_ang_vel = 0                   ///< initial angular velocity of right wheel
        ) override;

    /// Add visualization assets for the suspension subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the suspension subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the wheel track for the suspension subsystem.
    virtual double GetTrack() override;

    /// Return current suspension TSDA force information on the specified side.
    virtual std::vector<ForceTSDA> ReportSuspensionForce(VehicleSide side) const override;

    /// Return current RSDA torque information on the specified side.
    virtual std::vector<ForceRSDA> ReportSuspensionTorque(VehicleSide side) const override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

    /// Specify the suspension body on the specified side to attach a possible antirollbar subsystem.
    /// Return the corresponding lower control arm.
    virtual std::shared_ptr<ChBody> GetAntirollBody(VehicleSide side) const override;

  protected:
    /// Protected constructor.
    ChGenericWheeledSuspension(const std::string& name);

    /// Return the camber angle, in radians (default: 0).
    virtual double getCamberAngle() const { return 0; }

    /// Return the toe angle, in radians (default: 0).
    /// A positive value indicates toe-in, a negative value indicates toe-out.
    virtual double getToeAngle() const { return 0; }

    /// Return the location of the spindle body.
    virtual ChVector<> getSpindlePos() const = 0;
    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;
    /// Specify the suspension body to which the spindle is attached.
    virtual BodyIdentifier getSpindleAttachmentBody() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Specify the suspension body to which an antiroll bar subsystem can attach.
    virtual BodyIdentifier getAntirollBody() const = 0;

  private:
    ChFrame<> m_X_SA;  ///< suspension to absolute transform

    /// Internal specification of a suspension body.
    struct Body {
        std::shared_ptr<ChBody> body;  ///< underlying Chrono body
        ChVector<> pos;                ///< body position (in subsystem frame)
        ChQuaternion<> rot;            ///< body rotation (in subsystem frame)
        double mass;                   ///< body mass
        ChVector<> inertia_moments;    ///< moments of inertia
        ChVector<> inertia_products;   ///< products of inertia
        ChVehicleGeometry geometry;    ///< (optional) visualization and collision geometry
    };

    /// Internal specification of a suspension joint.
    struct Joint {
        std::shared_ptr<ChVehicleJoint> joint;        ///< underlying Chrono vehicle joint
        ChVehicleJoint::Type type;                    ///< joint type
        BodyIdentifier body1;                         ///< identifier of 1st body
        BodyIdentifier body2;                         ///< identifier of 2nd body
        ChVector<> pos;                               ///< joint position in subsystem frame
        ChQuaternion<> rot;                           ///< joint orientation in subsystem frame
        std::shared_ptr<ChVehicleBushingData> bdata;  ///< bushing data
    };

    /// Internal specification of a distance constraint.
    struct DistanceConstraint {
        std::shared_ptr<ChLinkDistance> dist;  ///< underlying Chrono distance constraint
        BodyIdentifier body1;                  ///< identifier of 1st body
        BodyIdentifier body2;                  ///< identifier of 2nd body
        ChVector<> point1;                     ///< point on body1 (in subsystem frame)
        ChVector<> point2;                     ///< point on body2 (in subsystem frame)
    };

    /// Internal specification of a suspension TSDA.
    struct TSDA {
        std::shared_ptr<ChLinkTSDA> tsda;                 ///< underlying Chrono TSDA element
        BodyIdentifier body1;                             ///< identifier of 1st body
        BodyIdentifier body2;                             ///< identifier of 2nd body
        ChVector<> point1;                                ///< point on body1 (in subsystem frame)
        ChVector<> point2;                                ///< point on body2 (in subsystem frame)
        double rest_length;                               ///< TSDA rest (free) length
        std::shared_ptr<ChLinkTSDA::ForceFunctor> force;  ///< force functor
        ChTSDAGeometry geometry;                          ///< (optional) visualization geometry
    };

    /// Internal specification of a suspension RSDA.
    struct RSDA {
        std::shared_ptr<ChLinkRSDA> rsda;                   ///< underlying Chrono RSDA element
        BodyIdentifier body1;                               ///< identifier of 1st body
        BodyIdentifier body2;                               ///< identifier of 2nd body
        ChVector<> pos;                                     ///< RSDA position (in subsystem frame)
        ChVector<> axis;                                    ///< axis of action for the RSDA (in subsystem frame)
        double rest_angle;                                  ///< RSDA rest (free) angle
        std::shared_ptr<ChLinkRSDA::TorqueFunctor> torque;  ///< torque functor
    };

    /// Key for a suspension part.
    struct PartKey {
        bool operator==(const PartKey& rhs) const;
        std::string name;  ///< base name
        int side;          ///< vehicle side for mirrored parts, -1 otherwise
    };

    /// Hash function for a part key.
    struct PartKeyHash {
        std::size_t operator()(const PartKey& id) const;
    };

    std::unordered_map<PartKey, Body, PartKeyHash> m_bodies;               ///< suspension bodies
    std::unordered_map<PartKey, Joint, PartKeyHash> m_joints;              ///< suspension joints
    std::unordered_map<PartKey, TSDA, PartKeyHash> m_tsdas;                ///< suspension TSDA force elements
    std::unordered_map<PartKey, RSDA, PartKeyHash> m_rsdas;                ///< suspension RSDA force elements
    std::unordered_map<PartKey, DistanceConstraint, PartKeyHash> m_dists;  ///< suspension distance constraints

    /// Get the unique name of an item in this suspension subsystem.
    std::string Name(const PartKey& id) const;

    /// Express a point given in the suspension reference frame to the absolute coordinate frame.
    ChVector<> TransformPosition(const ChVector<>& pos_loc, int side) const;

    /// Express a direction vector given in the suspension reference frame to the absolute coordinate frame.
    ChVector<> TransformDirection(const ChVector<>& dir_loc, int side) const;

    /// Express a quaternion given in the suspension reference frame to the absolute coordinate frame.
    ChQuaternion<> TransformRotation(const ChQuaternion<>& rot_local, int side) const;

    /// Find a body from the given identification.
    std::shared_ptr<ChBody> FindBody(BodyIdentifier body, int side) const;

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;
    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;
    virtual void Output(ChVehicleOutput& database) const override;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
