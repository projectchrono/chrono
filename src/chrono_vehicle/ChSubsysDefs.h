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
// Various utility classes for vehicle subsystems.
//
// =============================================================================

#ifndef CH_SUBSYS_DEFS_H
#define CH_SUBSYS_DEFS_H

#include <cstdint>
#include <string>
#include <vector>

#include "chrono/utils/ChForceFunctors.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

// -----------------------------------------------------------------------------
// Utility classes and structures for data exchange
// -----------------------------------------------------------------------------

/// Enum for the side (left/right) of a vehicle.
enum VehicleSide {
    LEFT = 0,  ///< left side of vehicle is always 0
    RIGHT = 1  ///< right side of vehicle is always 1
};

/// Enum for wheel location on spindle.
enum WheelLocation {
    SINGLE = 0,  ///< wheel on one side of a single-wheel axle
    INNER = 1,   ///< inner wheel on one side of a double-wheel axle
    OUTER = 2    ///< outer wheel on one side of a double-wheel axle
};

/// Structure to communicate a full body state.
struct BodyState {
    ChVector3d pos;      ///< global position
    ChQuaternion<> rot;  ///< orientation with respect to global frame
    ChVector3d lin_vel;  ///< linear velocity, expressed in the global frame
    ChVector3d ang_vel;  ///< angular velocity, expressed in the global frame
};

/// Vector of body state structures
typedef std::vector<BodyState> BodyStates;

/// Structure to communicate a full wheel body state.
/// In addition to the quantities communicated for a generic body, the wheel
/// state also includes the wheel angular speed about its axis of rotation.
struct WheelState {
    ChVector3d pos;      ///< global position
    ChQuaternion<> rot;  ///< orientation with respect to global frame
    ChVector3d lin_vel;  ///< linear velocity, expressed in the global frame
    ChVector3d ang_vel;  ///< angular velocity, expressed in the global frame
    double omega;        ///< wheel angular speed about its rotation axis
};

/// Vector of wheel state structures
typedef std::vector<WheelState> WheelStates;

/// Structure to communicate a set of generalized terrain contact forces (tire or track shoe).
struct TerrainForce {
    TerrainForce() : force(VNULL), point(VNULL), moment(VNULL) {}
    ChVector3d force;   ///< force vector, epxressed in the global frame
    ChVector3d point;   ///< global location of the force application point
    ChVector3d moment;  ///< moment vector, expressed in the global frame
};

/// Vector of terrain contact force structures.
typedef std::vector<TerrainForce> TerrainForces;

/// Driver (vehicle control) inputs.
struct DriverInputs {
    double m_steering;  ///< steering input [-1, +1]
    double m_throttle;  ///< throttle input [0, 1]
    double m_braking;   ///< braking input [0, 1]
    double m_clutch;    ///< clutch input [0, 1]
};

// -----------------------------------------------------------------------------

/// Utilities to encode and decode vehicle object tags.
/// An object tag (int) encodes the vehicle tag (uint16_t) and the tag of the containing part/subsystem type (uint16_t).
/// These are assigned to the ChObj objects (bodies, links, etc) that compose a given part/subsystem.
namespace VehicleObjTag {

/// Generate a vehicle tag and a subsystem tag into an object tag.
CH_VEHICLE_API int Generate(uint16_t vehicle_tag, uint16_t part_tag);

/// Extract the vehicle tag from a body tag.
CH_VEHICLE_API uint16_t ExtractVehicleTag(int tag);

/// Extract the subsystem (part) tag from a body tag.
CH_VEHICLE_API uint16_t ExtractPartTag(int tag);

}  // end namespace VehicleObjTag

/// Tags for specific parts of a wheeled vehicle.
enum VehiclePartTag : uint16_t {
    CHASSIS = 0xDD00,
    CHASSIS_REAR = 0xDD01,

    SUBCHASSIS = 0xEE00,
    SUSPENSION = 0xEE01,
    STEERING = 0xEE02,
    ANTIROLLBAR = 0xEE03,
    WHEEL = 0xEE04,
    TIRE = 0xEE05,
    
    SPROCKET = 0xFF00,
    IDLER = 0xFF01,
    TRACK_WHEEL = 0xFF02,
    SHOE = 0xFF03,
    TRACK_SUSPENSION = 0xFF04
};

// -----------------------------------------------------------------------------
// Enums and flags for wheeled and tracked vehicles
// -----------------------------------------------------------------------------

/// Enum for available tire models.
enum class TireModelType {
    RIGID,        ///< rigid tire (cylindrical)
    RIGID_MESH,   ///< rigid tire (mesh)
    FIALA,        ///< Fiala tire
    ANCF,         ///< ANCF shell element-based tire
    ANCF_LUMPED,  ///< ANCF shell element-based tire with single layers
    REISSNER,     ///< Reissner 6-field shell element-based tire
    FEA,          ///< FEA co-rotational tire
    PAC89,        ///< Pacejka 89 (magic formula) tire, version 1989
    TMEASY,       ///< Tire Model Made Easy tire (G. Rill)
    PAC02,        ///< Pacejka 02 (magic formula) tire, version 2002 or later
    TMSIMPLE      ///< Tire Model Simple (W. Hirschberg)
};

/// Enum for available engine model templates.
enum class EngineModelType {
    SHAFTS,      ///< engine model based on ChShaft elements
    SIMPLE_MAP,  ///< simple model based on engine maps
    SIMPLE       ///< simple engine model (similar to a DC motor)
};

/// Enum for available transmission model templates.
enum class TransmissionModelType {
    AUTOMATIC_SHAFTS,      ///< automatic transmission model based of ChShaft elements
    AUTOMATIC_SIMPLE_MAP,  ///< automatic transmission model based on TC maps
    AUTOMATIC_SIMPLE_CVT,  ///< automatic transmission model based on CVT design
    MANUAL_SHAFTS          ///< manual transmission model based on ChShaft elements
};

/// Enum for available wheeled-vehicle suspension model templates.
enum class SuspensionTypeWV {
    DEDION_AXLE,                      ///< DeDion Axle
    DOUBLE_WISHBONE,                  ///< double wishbone
    DOUBLE_WISHBONE_REDUCED,          ///< simplified double wishbone (constraint-based)
    HENDRICKSON_PRIMAXX,              ///< Hendrickson PRIMAXX (walking beam)
    LEAF_SPRING_AXLE,                 ///< leaf-spring solid axle
    SAE_LEAF_SPRING_AXLE,             ///< leaf-spring solid axle with kinematic leaf-spring model
    MACPHERSON_STRUT,                 ///< MacPherson strut
    MULTI_LINK,                       ///< multi-link
    RIGID_PINNED,                     ///< pinned rigid beam
    RIGID_SUSPENSION,                 ///< rigid suspension
    SEMI_TRAILING_ARM,                ///< semi trailing arm
    SOLID_AXLE,                       ///< solid axle
    SOLID_THREE_LINK_AXLE,            ///< rigid suspension + 3 guiding links
    SOLID_BELLCRANK_THREE_LINK_AXLE,  ///< rigid suspension + 3 guiding links + bellcrank steering mechanism
    THREE_LINK_IRS,                   ///< three-link independent rear suspension
    TOE_BAR_LEAF_SPRING_AXLE,         ///< steerable leaf-spring solid axle
    SAE_TOE_BAR_LEAF_SPRING_AXLE,     ///< steerable leaf-spring solid axle with kinematic leaf-spring model
    PUSHPIPE_AXLE,                    ///< solid axle with pushpipe and panhard rod
    TOEBAR_DEDION_AXLE,               ///< steerable DeDion Axle
    TOEBAR_PUSHPIPE_AXLE              ///< steerable solid axle with pushpipe and panhard rod
};

/// Enum for available brake model templates.
enum class BrakeType {
    SHAFTS,  ///< brake model using a clutch between two shafts
    SIMPLE   ///< brake model using a simple speed-dependent torque
};

/// Enum for available wheeled-vehicle steering model templates.
enum class SteeringTypeWV {
    PITMAN_ARM,         ///< Pitman arm (input to revolute joint)
    PITMAN_ARM_SHAFTS,  ///< Pitman arm with compliant column (input to steering wheel)
    RACK_PINION         ///< rack-pinion (input to pinion)
};

/// Enum for wheeled-vehicle driveline types.
enum class DrivelineTypeWV {
    FWD,        ///< front-wheel drive
    RWD,        ///< rear-wheel drive
    AWD,        ///< all-wheel drive (4x4)
    AWD6,       ///< all-wheel drive (6x4 / 6x6 locked)
    AWD8,       ///< all-wheel drive (8x8)
    SIMPLE,     ///< simple kinematic driveline
    SIMPLE_XWD  ///< simple kinematic driveline for more than 2 axles
};

/// Enum for tracked-vehicle driveline types.
enum class DrivelineTypeTV {
    BDS,    ///< braked differential steering
    SIMPLE  ///< simple kinematic driveline
};

/// Enum for track shoe types.
enum class TrackShoeType {
    SINGLE_PIN,    ///< single-pin track shoe and sprocket
    DOUBLE_PIN,    ///< double-pin track shoe and sprocket
    BAND_BUSHING,  ///< rigid tooth-rigid web continuous band track shoe and sprocket
    BAND_ANCF      ///< rigid tooth-ANCF web continuous band track shoe and sprocket
};

/// Topology of the double-pin track shoe.
/// The "full" double-pin track shoe mechanism uses separate bodies for the left and right connector bodies.  The
/// "reduced" model uses a single connector body. The mass and inertia of the composite connector body in the reduced
/// model are calculated based on the provided values for an individual connector body.  Furthermore, the collision
/// geometry is the same, meaning both models of a double-pin track shoe can interact with the same type of sprocket.
enum class DoublePinTrackShoeType {
    TWO_CONNECTORS,  ///< two connector bodies
    ONE_CONNECTOR    ///< one connector body
};

/// Enum for guide pin (track shoe/roadwheel/idler).
enum class GuidePinType {
    CENTRAL_PIN,  ///< track shoes with central guiding pin and double wheels
    LATERAL_PIN   ///< track shoes with lateral guiding pins and single wheels
};

/// Enumerations for track collision flags.
namespace TrackedCollisionFlag {
// Note: we cannot use strongly typed enums since these are used as integers
enum Enum {
    NONE = 0,
    CHASSIS = 1 << 0,
    SPROCKET_LEFT = 1 << 1,
    SPROCKET_RIGHT = 1 << 2,
    IDLER_LEFT = 1 << 3,
    IDLER_RIGHT = 1 << 4,
    WHEELS_LEFT = 1 << 5,
    WHEELS_RIGHT = 1 << 6,
    SHOES_LEFT = 1 << 7,
    SHOES_RIGHT = 1 << 8,
    ROLLERS_LEFT = 1 << 9,
    ROLLERS_RIGHT = 1 << 10,
    ALL = 0xFFFF
};
}  // namespace TrackedCollisionFlag

/// Enumerations for vehicle collision families.
namespace VehicleCollisionFamily {
// Note: we cannot use strongly typed enums, since these are passed as integers
enum Enum {
    CHASSIS_FAMILY = 0,  ///< chassis collision family

    TIRE_FAMILY = 1,   ///< collision family for tire systems
    WHEEL_FAMILY = 2,  ///< collision family for wheel systems

    IDLER_FAMILY = 3,        ///< collision family for idler subsystems
    TRACK_WHEEL_FAMILY = 4,  ///< collision family for road-wheel assemblies
    ROLLER_FAMILY = 5,       ///< collision family for roller subsystems
    SHOE_FAMILY = 6          ///< collision family for track shoe subsystems
};
}  // namespace VehicleCollisionFamily

/// Flags for output (log/debug).
/// These flags can be bit-wise ORed and used as a mask.
enum OutputInformation {
    OUT_SPRINGS = 1 << 0,      ///< suspension spring information
    OUT_SHOCKS = 1 << 1,       ///< suspension shock information
    OUT_CONSTRAINTS = 1 << 2,  ///< constraint violation information
    OUT_TESTRIG = 1 << 3       ///< test-rig specific information
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
