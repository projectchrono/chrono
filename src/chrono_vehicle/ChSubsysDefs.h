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

#include <string>
#include <vector>

#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkTSDA.h"

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
    ChVector<> pos;      ///< global position
    ChQuaternion<> rot;  ///< orientation with respect to global frame
    ChVector<> lin_vel;  ///< linear velocity, expressed in the global frame
    ChVector<> ang_vel;  ///< angular velocity, expressed in the global frame
};

/// Vector of body state structures
typedef std::vector<BodyState> BodyStates;

/// Structure to communicate a full wheel body state.
/// In addition to the quantities communicated for a generic body, the wheel
/// state also includes the wheel angular speed about its axis of rotation.
struct WheelState {
    ChVector<> pos;      ///< global position
    ChQuaternion<> rot;  ///< orientation with respect to global frame
    ChVector<> lin_vel;  ///< linear velocity, expressed in the global frame
    ChVector<> ang_vel;  ///< angular velocity, expressed in the global frame
    double omega;        ///< wheel angular speed about its rotation axis
};

/// Vector of wheel state structures
typedef std::vector<WheelState> WheelStates;

/// Structure to communicate a set of generalized terrain contact forces (tire or track shoe).
struct TerrainForce {
    TerrainForce() : force(VNULL), point(VNULL), moment(VNULL) {}
    ChVector<> force;   ///< force vector, epxressed in the global frame
    ChVector<> point;   ///< global location of the force application point
    ChVector<> moment;  ///< moment vector, expressed in the global frame
};

/// Vector of terrain conatct force structures.
typedef std::vector<TerrainForce> TerrainForces;

/// Driver (vehicle control) inputs.
struct DriverInputs {
    double m_steering;  ///< steering input [-1, +1]
    double m_throttle;  ///< throttle input [0, 1]
    double m_braking;   ///< braking input [0, 1]
    double m_clutch;    ///< clutch input [0, 1]
};

// -----------------------------------------------------------------------------
// Utility functor classes for force elements
// -----------------------------------------------------------------------------

/// Base class for linear and nonlinear translational spring forces.
class CH_VEHICLE_API SpringForce : public ChLinkTSDA::ForceFunctor {
  public:
    SpringForce(double preload);
    void enable_stops(double min_length, double max_length);
    void set_stops(const std::vector<std::pair<double, double>>& data_bump,
                   const std::vector<std::pair<double, double>>& data_rebound);
    void set_stops(double bump_coefficient, double rebound_coefficient);
    double evaluate_stops(double length);
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  protected:
    double m_P;  ///< pre-tension

    bool m_stops;
    double m_min_length;
    double m_max_length;
    ChFunction_Recorder m_bump;
    ChFunction_Recorder m_rebound;
};

/// Utility class for specifying a linear translational spring force with pre-tension.
/// F = P - K * (length - rest_length)
class CH_VEHICLE_API LinearSpringForce : public SpringForce {
  public:
    LinearSpringForce(double k, double preload = 0);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
};

/// Utility class for specifying a nonlinear translational spring force with pre-tension.
/// F = P - mapK(length - rest_length)
class CH_VEHICLE_API NonlinearSpringForce : public SpringForce {
  public:
    NonlinearSpringForce(double preload = 0);
    NonlinearSpringForce(const std::vector<std::pair<double, double>>& dataK, double preload = 0);
    void add_pointK(double x, double y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunction_Recorder m_mapK;
};

/// Utility class for specifying a linear translational damper force.
/// F = -C * vel
class CH_VEHICLE_API LinearDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    LinearDamperForce(double c, double preload = 0);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_c;
};

/// Utility class for specifying a nonlinear translational damper force.
/// F = -mapC(vel)
class CH_VEHICLE_API NonlinearDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    NonlinearDamperForce();
    NonlinearDamperForce(const std::vector<std::pair<double, double>>& dataC);
    void add_pointC(double x, double y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunction_Recorder m_mapC;
};

/// Utility class for specifying a degressive translational damper force.
class CH_VEHICLE_API DegressiveDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    /// Fallback to LinearDamperForce
    DegressiveDamperForce(double c_compression);

    /// Fallback to LinearDamperForce with different compression and expansion bins
    DegressiveDamperForce(double c_compression, double c_expansion);

    /// Different compression and expansion degressivity, same damper coefficient at origin
    DegressiveDamperForce(double c_compression, double degr_compression, double degr_expansion);

    /// Full parametrization
    DegressiveDamperForce(double c_compression, double degr_compression, double c_expansion, double degr_expansion);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_c_compression;
    double m_c_expansion;
    double m_degr_compression;
    double m_degr_expansion;
};

/// Utility class for specifying a linear translational spring-damper force with pre-tension.
/// F = P - K * (length - rest_length) - C * vel
class CH_VEHICLE_API LinearSpringDamperForce : public SpringForce {
  public:
    LinearSpringDamperForce(double k, double c, double preload = 0);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
    double m_c;
};

/// Utility class for specifying a nonlinear translational spring-damper force with pre-tension.
/// F = P - mapK(length - rest_length) - mapC(vel)
class CH_VEHICLE_API NonlinearSpringDamperForce : public SpringForce {
  public:
    NonlinearSpringDamperForce(double preload = 0);
    NonlinearSpringDamperForce(const std::vector<std::pair<double, double>>& dataK,
                               const std::vector<std::pair<double, double>>& dataC,
                               double preload = 0);
    void add_pointK(double x, double y);
    void add_pointC(double x, double y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunction_Recorder m_mapK;
    ChFunction_Recorder m_mapC;
};

/// Utility class for specifying a general nonlinear translational spring-damper force with pre-tension.
/// F = P - map(length - rest_length, vel)
class CH_VEHICLE_API MapSpringDamperForce : public SpringForce {
  public:
    MapSpringDamperForce(double preload = 0);
    MapSpringDamperForce(const std::vector<double>& defs,
                         const std::vector<double>& vels,
                         ChMatrixConstRef data,
                         double preload = 0);
    void set_deformations(const std::vector<double> defs);
    void add_pointC(double x, const std::vector<double>& y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

    void print_data();

  private:
    std::vector<double> m_defs;
    std::vector<double> m_vels;
    ChMatrixDynamic<double> m_data;

    std::pair<int, int> m_last;
};

// -----------------------------------------------------------------------------
// Utility functor classes for torque elements
// -----------------------------------------------------------------------------

/// Utility class for specifying a linear rotational spring torque.
class CH_VEHICLE_API LinearSpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearSpringTorque(double k, double preload = 0);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
    double m_P;
};

/// Utility class for specifying a nonlinear rotational spring torque.
class CH_VEHICLE_API NonlinearSpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    NonlinearSpringTorque(double preload = 0);
    NonlinearSpringTorque(const std::vector<std::pair<double, double>>& dataK, double preload = 0);
    void add_pointK(double x, double y);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunction_Recorder m_mapK;
    double m_P;
};

/// Utility class for specifying a linear rotational damper torque.
class CH_VEHICLE_API LinearDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearDamperTorque(double c);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_c;
};

/// Utility class for specifying a nonlinear rotational damper torque.
class CH_VEHICLE_API NonlinearDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    NonlinearDamperTorque();
    NonlinearDamperTorque(const std::vector<std::pair<double, double>>& dataC);
    void add_pointC(double x, double y);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunction_Recorder m_mapC;
};

/// Utility class for specifying a linear rotational spring-damper torque.
class CH_VEHICLE_API LinearSpringDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearSpringDamperTorque(double k, double c, double preload = 0);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
    double m_c;
    double m_P;
};

/// Utility class for specifying a nonlinear rotational spring-damper torque.
class CH_VEHICLE_API NonlinearSpringDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    NonlinearSpringDamperTorque(double preload = 0);
    NonlinearSpringDamperTorque(const std::vector<std::pair<double, double>>& dataK,
                                const std::vector<std::pair<double, double>>& dataC,
                                double preload = 0);
    void add_pointK(double x, double y);
    void add_pointC(double x, double y);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunction_Recorder m_mapK;
    ChFunction_Recorder m_mapC;
    double m_P;
};

// -----------------------------------------------------------------------------
// Enums and flags for wheeled and tracked vehicles
// -----------------------------------------------------------------------------

/// Enum for visualization types.
enum class VisualizationType {
    NONE,        ///< no visualization
    PRIMITIVES,  ///< use primitve shapes
    MESH         ///< use meshes
};

/// Enum for available tire models.
enum class TireModelType {
    RIGID,       ///< rigid tire (cylindrical)
    RIGID_MESH,  ///< rigid tire (mesh)
    FIALA,       ///< Fiala tire
    ANCF,        ///< ANCF shell element-based tire
    REISSNER,    ///< Reissner 6-field shell element-based tire
    FEA,         ///< FEA co-rotational tire
    PAC89,       ///< Pacejka 89 (magic formula) tire, version 1989
    TMEASY,      ///< Tire Model Made Easy tire (G. Rill)
    PAC02,       ///< Pacejka 02 (magic formula) tire, version 2002 or later
    TMSIMPLE     ///< Tire Model Simple (W. Hirschberg)
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
    MANUAL_SHAFTS          ///< manual transmission model based on ChShaft elements
};

/// Enum for available wheeled-vehicle suspension model templates.
enum class SuspensionTypeWV {
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

/// Enumerations for wheeled vehicle collision families.
namespace WheeledCollisionFamily {
// Note: we cannot use strongly typed enums, since these are passed as integers
enum Enum {
    CHASSIS = 0,  ///< chassis collision family
    TIRE = 1,     ///< collision family for tire systems
    WHEEL = 2     ///< collision family for wheel systems
};
}  // namespace WheeledCollisionFamily

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

/// Enumerations for tracked vehicle collision families.
namespace TrackedCollisionFamily {
// Note: we cannot use strongly typed enums, since these are passed as integers
enum Enum {
    CHASSIS = 0,  ///< chassis collision family
    IDLERS = 1,   ///< collision family for idler subsystems
    WHEELS = 2,   ///< collision family for road-wheel assemblies
    ROLLERS = 3,  ///< collision family for roller subsystems
    SHOES = 4     ///< collision family for track shoe subsystems
};
}  // namespace TrackedCollisionFamily

/// Flags for output (log/debug).
/// These flags can be bit-wise ORed and used as a mask.
enum OutputInformation {
    OUT_SPRINGS = 1 << 0,      ///< suspension spring information
    OUT_SHOCKS = 1 << 1,       ///< suspension shock information
    OUT_CONSTRAINTS = 1 << 2,  ///< constraint violation information
    OUT_TESTRIG = 1 << 3       ///< test-rig specific information
};

/// Identifiers for specific component bodies.
enum BodyID {
    CHASSIS_BODY = -99990,
    SPROCKET_BODY = -99991,
    IDLER_BODY = -99992,
    WHEEL_BODY = -99993,
    ROLER_BODY = -99994,
    SHOE_BODY = -99995
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
