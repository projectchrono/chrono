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

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/assets/ChColor.h"

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
};

// -----------------------------------------------------------------------------
// Utility functor classes for force elements
// -----------------------------------------------------------------------------

/// Utility class for specifying a linear translational spring force.
class LinearSpringForce : public ChLinkTSDA::ForceFunctor {
  public:
    LinearSpringForce(double k) : m_k(k) {}
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return -m_k * (length - rest_length);
    }

  private:
    double m_k;
};

/// Utility class for specifying a linear translational damper force.
class LinearDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    LinearDamperForce(double c) : m_c(c) {}
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return -m_c * vel;
    }

  private:
    double m_c;
};

/// Utility class for specifying a linear translational spring-damper force.
class LinearSpringDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    LinearSpringDamperForce(double k, double c) : m_k(k), m_c(c) {}
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return -m_k * (length - rest_length) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
};

/// Utility class for specifying a linear translational spring-damper force with pre-tension.
class LinearSpringDamperActuatorForce : public ChLinkTSDA::ForceFunctor {
  public:
    LinearSpringDamperActuatorForce(double k, double c, double f) : m_k(k), m_c(c), m_f(f) {}
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return m_f - m_k * (length - rest_length) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_f;
};

/// Utility class for specifying a map translational spring force.
class MapSpringForce : public ChLinkTSDA::ForceFunctor {
  public:
    MapSpringForce() {}
    MapSpringForce(const std::vector<std::pair<double, double>>& data) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return -m_map.Get_y(length - rest_length);
    }

  private:
    ChFunction_Recorder m_map;
};

/// Utility class for specifying a map translational spring force with bump and rebound stop.
class MapSpringBistopForce : public ChLinkTSDA::ForceFunctor {
  public:
    MapSpringBistopForce(double spring_min_length, double spring_max_length)
        : m_min_length(spring_min_length), m_max_length(spring_max_length) {
        setup_stop_maps();
    }
    MapSpringBistopForce(const std::vector<std::pair<double, double>>& data,
                         double spring_min_length,
                         double spring_max_length)
        : m_min_length(spring_min_length), m_max_length(spring_max_length) {
        setup_stop_maps();
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        double defl_bump = 0.0;
        double defl_rebound = 0.0;

        if (length < m_min_length) {
            defl_bump = m_min_length - length;
        }

        if (length > m_max_length) {
            defl_rebound = length - m_max_length;
        }

        return -m_map.Get_y(length - rest_length) + m_bump.Get_y(defl_bump) - m_rebound.Get_y(defl_rebound);
    }

  private:
    void setup_stop_maps() {
        m_bump.AddPoint(0.0, 0.0);
        m_bump.AddPoint(2.0e-3, 200.0);
        m_bump.AddPoint(4.0e-3, 400.0);
        m_bump.AddPoint(6.0e-3, 600.0);
        m_bump.AddPoint(8.0e-3, 800.0);
        m_bump.AddPoint(10.0e-3, 1000.0);
        m_bump.AddPoint(20.0e-3, 2500.0);
        m_bump.AddPoint(30.0e-3, 4500.0);
        m_bump.AddPoint(40.0e-3, 7500.0);
        m_bump.AddPoint(50.0e-3, 12500.0);
        m_bump.AddPoint(60.0e-3, 125000.0);

        m_rebound.AddPoint(0.0, 0.0);
        m_rebound.AddPoint(2.0e-3, 200.0);
        m_rebound.AddPoint(4.0e-3, 400.0);
        m_rebound.AddPoint(6.0e-3, 600.0);
        m_rebound.AddPoint(8.0e-3, 800.0);
        m_rebound.AddPoint(10.0e-3, 1000.0);
        m_rebound.AddPoint(20.0e-3, 2500.0);
        m_rebound.AddPoint(30.0e-3, 4500.0);
        m_rebound.AddPoint(40.0e-3, 7500.0);
        m_rebound.AddPoint(50.0e-3, 12500.0);
        m_rebound.AddPoint(60.0e-3, 125000.0);
    }
    ChFunction_Recorder m_map;
    ChFunction_Recorder m_bump;
    ChFunction_Recorder m_rebound;
    double m_min_length;
    double m_max_length;
};

/// Utility class for specifying a linear translational spring force with bump and rebound stop.
class LinearSpringBistopForce : public ChLinkTSDA::ForceFunctor {
  public:
    /// Use default bump stop and rebound stop maps
    LinearSpringBistopForce(double k, double min_length, double max_length)
        : m_k(k), m_min_length(min_length), m_max_length(max_length) {
        // From ADAMS/Car example
        m_bump.AddPoint(0.0, 0.0);
        m_bump.AddPoint(2.0e-3, 200.0);
        m_bump.AddPoint(4.0e-3, 400.0);
        m_bump.AddPoint(6.0e-3, 600.0);
        m_bump.AddPoint(8.0e-3, 800.0);
        m_bump.AddPoint(10.0e-3, 1000.0);
        m_bump.AddPoint(20.0e-3, 2500.0);
        m_bump.AddPoint(30.0e-3, 4500.0);
        m_bump.AddPoint(40.0e-3, 7500.0);
        m_bump.AddPoint(50.0e-3, 12500.0);
        m_bump.AddPoint(60.0e-3, 125000.0);

        m_rebound.AddPoint(0.0, 0.0);
        m_rebound.AddPoint(2.0e-3, 200.0);
        m_rebound.AddPoint(4.0e-3, 400.0);
        m_rebound.AddPoint(6.0e-3, 600.0);
        m_rebound.AddPoint(8.0e-3, 800.0);
        m_rebound.AddPoint(10.0e-3, 1000.0);
        m_rebound.AddPoint(20.0e-3, 2500.0);
        m_rebound.AddPoint(30.0e-3, 4500.0);
        m_rebound.AddPoint(40.0e-3, 7500.0);
        m_rebound.AddPoint(50.0e-3, 12500.0);
        m_rebound.AddPoint(60.0e-3, 125000.0);
    }

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        double force = 0;

        double defl_spring = rest_length - length;
        double defl_bump = 0.0;
        double defl_rebound = 0.0;

        if (length < m_min_length) {
            defl_bump = m_min_length - length;
        }

        if (length > m_max_length) {
            defl_rebound = length - m_max_length;
        }

        force = defl_spring * m_k + m_bump.Get_y(defl_bump) - m_rebound.Get_y(defl_rebound);

        return force;
    }

  private:
    double m_k;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
    ChFunction_Recorder m_rebound;
};

/// Utility class for specifying a degressive translational damper force.
class DegressiveDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    /// Fallback to LinearDamperForce
    DegressiveDamperForce(double c_compression)
        : m_c_compression(c_compression), m_c_expansion(c_compression), m_degr_compression(0), m_degr_expansion(0) {}

    /// Fallback to LinearDamperForce with different compression and expansion bins
    DegressiveDamperForce(double c_compression, double c_expansion)
        : m_c_compression(c_compression), m_c_expansion(c_expansion), m_degr_compression(0), m_degr_expansion(0) {}

    /// Different compression and expansion degressivity, same damper coefficient at origin
    DegressiveDamperForce(double c_compression, double degr_compression, double degr_expansion)
        : m_c_compression(c_compression),
          m_c_expansion(c_compression),
          m_degr_compression(degr_compression),
          m_degr_expansion(degr_expansion) {}

    /// Full parametrization
    DegressiveDamperForce(double c_compression, double degr_compression, double c_expansion, double degr_expansion)
        : m_c_compression(c_compression),
          m_c_expansion(c_expansion),
          m_degr_compression(degr_compression),
          m_degr_expansion(degr_expansion) {}

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        if (vel >= 0) {
            return -m_c_expansion * vel / (1.0 + m_degr_expansion * vel);
        } else {
            return -m_c_compression * vel / (1.0 - m_degr_compression * vel);
        }
    }

  private:
    double m_c_compression;
    double m_c_expansion;
    double m_degr_compression;
    double m_degr_expansion;
};

/// Utility class for specifying a map translational damper force.
class MapDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    MapDamperForce() {}
    MapDamperForce(const std::vector<std::pair<double, double>>& data) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return -m_map.Get_y(vel);
    }

  private:
    ChFunction_Recorder m_map;
};

/// Utility class for specifying a map translational spring-damper force with pre-tension.
class MapSpringDamperActuatorForce : public ChLinkTSDA::ForceFunctor {
  public:
    MapSpringDamperActuatorForce() {}
    MapSpringDamperActuatorForce(const std::vector<std::pair<double, double>>& dataK,
                                 const std::vector<std::pair<double, double>>& dataC,
                                 double f) {
        for (unsigned int i = 0; i < dataK.size(); ++i) {
            m_mapK.AddPoint(dataK[i].first, dataK[i].second);
        }
        for (unsigned int i = 0; i < dataC.size(); ++i) {
            m_mapC.AddPoint(dataC[i].first, dataC[i].second);
        }
    }
    void add_pointK(double x, double y) { m_mapK.AddPoint(x, y); }
    void add_pointC(double x, double y) { m_mapC.AddPoint(x, y); }
    void set_f(double f) { m_f = f; }
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return m_f - m_mapK.Get_y(length - rest_length) - m_mapC.Get_y(vel);
    }

  private:
    ChFunction_Recorder m_mapK;
    ChFunction_Recorder m_mapC;
    double m_f;
};

/// Utility class for specifying a linear rotational spring torque.
class LinearSpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearSpringTorque(double k, double rest_angle = 0) : m_k(k), m_rest_angle(rest_angle) {}
    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return -m_k * (angle - m_rest_angle);
    }

  private:
    double m_k;
    double m_rest_angle;
};

/// Utility class for specifying a linear rotational damper torque.
class LinearDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearDamperTorque(double c) : m_c(c) {}
    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return -m_c * vel;
    }

  private:
    double m_c;
};

/// Utility class for specifying a linear rotational spring-damper torque.
class LinearSpringDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearSpringDamperTorque(double k, double c, double rest_angle = 0) : m_k(k), m_c(c), m_rest_angle(rest_angle) {}
    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return -m_k * (angle - m_rest_angle) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_rest_angle;
};

/// Utility class for specifying a linear rotational spring-damper torque with pre-tension.
class LinearSpringDamperActuatorTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearSpringDamperActuatorTorque(double k, double c, double t, double rest_angle = 0)
        : m_k(k), m_c(c), m_t(t), m_rest_angle(rest_angle) {}
    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return m_t - m_k * (angle - m_rest_angle) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_t;
    double m_rest_angle;
};

/// Utility class for specifying a map rotational spring torque.
class MapSpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    MapSpringTorque() {}
    MapSpringTorque(const std::vector<std::pair<double, double>>& data, double rest_angle = 0)
        : m_rest_angle(rest_angle) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return -m_map.Get_y(angle - m_rest_angle);
    }

  private:
    ChFunction_Recorder m_map;
    double m_rest_angle;
};

/// Utility class for specifying a map rotational damper torque.
class MapDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    MapDamperTorque() {}
    MapDamperTorque(const std::vector<std::pair<double, double>>& data) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return -m_map.Get_y(vel);
    }

  private:
    ChFunction_Recorder m_map;
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
    PACEJKA,     ///< Pacejka (magic formula) tire
    LUGRE,       ///< Lugre frition model tire
    FIALA,       ///< Fiala tire
    ANCF,        ///< ANCF shell element-based tire
    REISSNER,    ///< Reissner 6-field shell element-based tire
    FEA,         ///< FEA co-rotational tire
    PAC89,       ///< Pacejka 89 (magic formula) tire
    TMEASY,      ///< Tire Model Made Easy tire (G. Rill)
    PAC02        ///< Pacejka 02 (magic formula) tire, redesign of PACEJKA
};

/// Enum for available powertrain model templates.
enum class PowertrainModelType {
    SHAFTS,      ///< powertrain based on ChShaft elements
    SIMPLE_MAP,  ///< simple powertrain model (based on engine-map)
    SIMPLE,      ///< simple powertrain model (similar to a DC motor)
    SIMPLE_CVT   ///< simple cvt powertrain model (like a DC motor / CVT gearbox)
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
    SOLID_BELLCRANK_THREE_LINK_AXLE,  ///< rigid suspension + 3 guiding linls + bellcrank steering mechanism
    THREE_LINK_IRS,                   ///< three-link independent rear suspension
    TOE_BAR_LEAF_SPRING_AXLE,         ///< steerable leaf-spring solid axle
    SAE_TOE_BAR_LEAF_SPRING_AXLE      ///< steerable leaf-spring solid axle with kinematic leaf-spring model
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
    TIRES = 1     ///< collision family for tire systems
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

// -----------------------------------------------------------------------------
// Class defining geometry (visualization and collision) and contact materials.
// -----------------------------------------------------------------------------

/// Utility class defining geometry (visualization and collision) and contact materials for a rigid vehicle body.
/// Holds vectors of primitive shapes (any one of which may be empty) and a list of contact materials.
/// Each shape defines its position and orientation relative to the parent body, geometric dimensions, and an index into
/// the list of contact materials.
class CH_VEHICLE_API ChVehicleGeometry {
  public:
    ChVehicleGeometry();

    /// Box shape for visualization and/or collision.
    struct BoxShape {
        BoxShape(const ChVector<>& pos, const ChQuaternion<>& rot, const ChVector<>& dims, int matID = -1)
            : m_pos(pos), m_rot(rot), m_dims(dims), m_matID(matID) {}
        ChVector<> m_pos;      ///< position relative to body
        ChQuaternion<> m_rot;  ///< orientation relative to body
        ChVector<> m_dims;     ///< box dimensions
        int m_matID;           ///< index in contact material list
    };

    /// Sphere shape for visualization and/or collision.
    struct SphereShape {
        SphereShape(const ChVector<>& pos, double radius, int matID = -1)
            : m_pos(pos), m_radius(radius), m_matID(matID) {}
        ChVector<> m_pos;  ///< position relative to body
        double m_radius;   ///< sphere radius
        int m_matID;       ///< index in contact material list
    };

    /// Cylinder shape for visualization and/or collision.
    struct CylinderShape {
        CylinderShape(const ChVector<>& pos, const ChQuaternion<>& rot, double radius, double length, int matID = -1)
            : m_pos(pos), m_rot(rot), m_radius(radius), m_length(length), m_matID(matID) {}
        ChVector<> m_pos;      ///< position relative to body
        ChQuaternion<> m_rot;  ///< orientation relative to body
        double m_radius;       ///< cylinder radius
        double m_length;       ///< cylinder length
        int m_matID;           ///< index in contact material list
    };

    struct LineShape {
        LineShape(const ChVector<>& pos, const ChQuaternion<>& rot, std::shared_ptr<geometry::ChLine> line)
            : m_pos(pos), m_rot(rot), m_line(line) {}
        ChVector<> m_pos;                              ///< position relative to body
        ChQuaternion<> m_rot;                          ///< orientation relative to body
        std::shared_ptr<geometry::ChLine> m_line;  ///< line data
    };

    /// Convex hulls shape for collision.
    struct ConvexHullsShape {
        ConvexHullsShape(const std::string& filename, int matID = -1) : m_filename(filename), m_matID(matID) {}
        std::string m_filename;  ///< name of Wavefront OBJ file
        int m_matID;             ///< index in contact material list
    };

    /// Tri-mesh shape for collision.
    struct TrimeshShape {
        TrimeshShape(const ChVector<>& pos, const std::string& filename, double radius, int matID = -1)
            : m_filename(filename), m_radius(radius), m_pos(pos), m_matID(matID) {}
        std::string m_filename;  ///< name of Wavefront OBJ file
        double m_radius;         ///< radius of sweeping sphere
        ChVector<> m_pos;        ///< position relative to body
        int m_matID;             ///< index in contact material list
    };

    bool m_has_collision;                                         ///< true if body has a collision model
    std::vector<std::shared_ptr<ChMaterialSurface>> m_materials;  ///< list of contact materials
    std::vector<BoxShape> m_coll_boxes;                           ///< list of collision boxes
    std::vector<SphereShape> m_coll_spheres;                      ///< list of collision spheres
    std::vector<CylinderShape> m_coll_cylinders;                  ///< list of collision cylinders
    std::vector<ConvexHullsShape> m_coll_hulls;                   ///< list of collision convex hulls
    std::vector<TrimeshShape> m_coll_meshes;                      ///< list of collision trimeshes

    bool m_has_primitives;                       ///< true if the body uses visualization primitives
    std::vector<BoxShape> m_vis_boxes;           ///< list of visualization boxes
    std::vector<SphereShape> m_vis_spheres;      ///< list of visualization spheres
    std::vector<CylinderShape> m_vis_cylinders;  ///< list of visualization cylinders
    std::vector<LineShape> m_vis_lines;          ///< list of visualization lines

    bool m_has_colors;          ///< true if primitive colors were provided
    ChColor m_color_boxes;      ///< visualization color
    ChColor m_color_spheres;    ///< visualization color
    ChColor m_color_cylinders;  ///< visualization color

    bool m_has_mesh;              ///< true if the body uses a visualization mesh
    std::string m_vis_mesh_file;  ///< name of Wavefront OBJ file with visualizaiton mesh

    /// Create visualization assets for the specified body.
    void AddVisualizationAssets(std::shared_ptr<ChBody> body, VisualizationType vis);

    /// Create collision shapes for the specified body.
    void AddCollisionShapes(std::shared_ptr<ChBody> body, int collision_family);
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
