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

#include <vector>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChLinkRotSpringCB.h"
#include "chrono/physics/ChLinkSpringCB.h"

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

/// Class to encode the ID of a vehicle wheel.
/// By convention, wheels are counted front to rear and left to right. In other
/// words, for a vehicle with 2 axles, the order is: front-left, front-right,
/// rear-left, rear-right.
class WheelID {
  public:
    WheelID(int id) : m_id(id), m_axle(id / 2), m_side(VehicleSide(id % 2)) {}
    WheelID(int axle, VehicleSide side) : m_id(2 * axle + side), m_axle(axle), m_side(side) {}

    /// Return the wheel ID.
    int id() const { return m_id; }

    /// Return the axle index for this wheel ID.
    /// Axles are counted from the front of the vehicle.
    int axle() const { return m_axle; }

    /// Return the side for this wheel ID.
    /// By convention, left is 0 and right is 1.
    VehicleSide side() const { return m_side; }

  private:
    int m_id;            ///< wheel ID
    int m_axle;          ///< axle index (counted from the front)
    VehicleSide m_side;  ///< vehicle side (LEFT: 0, RIGHT: 1)
};

/// Global constant wheel IDs for the common topology of a 2-axle vehicle.
CH_VEHICLE_API extern const WheelID FRONT_LEFT;
CH_VEHICLE_API extern const WheelID FRONT_RIGHT;
CH_VEHICLE_API extern const WheelID REAR_LEFT;
CH_VEHICLE_API extern const WheelID REAR_RIGHT;

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

// -----------------------------------------------------------------------------
// Utility functor classes for force elements
// -----------------------------------------------------------------------------

/// Utility class for specifying a linear translational spring force.
class LinearSpringForce : public ChLinkSpringCB::ForceFunctor {
  public:
    LinearSpringForce(double k) : m_k(k) {}
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        return -m_k * (length - rest_length);
    }

  private:
    double m_k;
};

/// Utility class for specifying a linear translational damper force.
class LinearDamperForce : public ChLinkSpringCB::ForceFunctor {
  public:
    LinearDamperForce(double c) : m_c(c) {}
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        return -m_c * vel;
    }

  private:
    double m_c;
};

/// Utility class for specifying a linear translational spring-damper force.
class LinearSpringDamperForce : public ChLinkSpringCB::ForceFunctor {
  public:
    LinearSpringDamperForce(double k, double c) : m_k(k), m_c(c) {}
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        return -m_k * (length - rest_length) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
};

/// Utility class for specifying a linear translational spring-damper force with pre-tension.
class LinearSpringDamperActuatorForce : public ChLinkSpringCB::ForceFunctor {
  public:
    LinearSpringDamperActuatorForce(double k, double c, double f) : m_k(k), m_c(c), m_f(f) {}
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        return m_f - m_k * (length - rest_length) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_f;
};

/// Utility class for specifying a map translational spring force.
class MapSpringForce : public ChLinkSpringCB::ForceFunctor {
  public:
    MapSpringForce() {}
    MapSpringForce(const std::vector<std::pair<double, double>>& data) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        return -m_map.Get_y(length - rest_length);
    }

  private:
    ChFunction_Recorder m_map;
};

/// Utility class for specifying a linear translational spring force with bump and rebound stop.
class LinearSpringBistopForce : public ChLinkSpringCB::ForceFunctor {
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

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
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
class DegressiveDamperForce : public ChLinkSpringCB::ForceFunctor {
  public:
    /// Fallback to LinearDamperForce
    DegressiveDamperForce(double c_compression)
        : m_c_compression(c_compression), m_degr_compression(0), m_c_expansion(c_compression), m_degr_expansion(0) {}

    /// Fallback to LinearDamperForce with different compression and expansion bins
    DegressiveDamperForce(double c_compression, double c_expansion)
        : m_c_compression(c_compression), m_degr_compression(0), m_c_expansion(c_expansion), m_degr_expansion(0) {}

    /// Different compression and expansion degressivity, same damper coefficient at origin
    DegressiveDamperForce(double c_compression, double degr_compression, double degr_expansion)
        : m_c_compression(c_compression),
          m_degr_compression(degr_compression),
          m_c_expansion(c_compression),
          m_degr_expansion(degr_expansion) {}

    /// Full parametrization
    DegressiveDamperForce(double c_compression, double degr_compression, double c_expansion, double degr_expansion)
        : m_c_compression(c_compression),
          m_degr_compression(degr_compression),
          m_c_expansion(c_expansion),
          m_degr_expansion(degr_expansion) {}

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        if (vel >= 0) {
            return -m_c_expansion * vel / (1.0 + m_degr_expansion * vel);
        } else {
            return -m_c_compression * vel / (1.0 + m_degr_compression * std::abs(vel));
        }
    }

  private:
    double m_c_compression;
    double m_c_expansion;
    double m_degr_compression;
    double m_degr_expansion;
};

/// Utility class for specifying a map translational damper force.
class MapDamperForce : public ChLinkSpringCB::ForceFunctor {
  public:
    MapDamperForce() {}
    MapDamperForce(const std::vector<std::pair<double, double>>& data) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        return -m_map.Get_y(vel);
    }

  private:
    ChFunction_Recorder m_map;
};

/// Utility class for specifying a map translational spring-damper force with pre-tension.
class MapSpringDamperActuatorForce : public ChLinkSpringCB::ForceFunctor {
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
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
        return m_f - m_mapK.Get_y(length - rest_length) - m_mapC.Get_y(vel);
    }

  private:
    ChFunction_Recorder m_mapK;
    ChFunction_Recorder m_mapC;
    double m_f;
};

/// Utility class for specifying a linear rotational spring torque.
class LinearSpringTorque : public ChLinkRotSpringCB::TorqueFunctor {
  public:
    LinearSpringTorque(double k, double rest_angle = 0) : m_k(k), m_rest_angle(rest_angle) {}
    virtual double operator()(double time, double angle, double vel, ChLinkRotSpringCB* link) override {
        return -m_k * (angle - m_rest_angle);
    }

  private:
    double m_k;
    double m_rest_angle;
};

/// Utility class for specifying a linear rotational damper torque.
class LinearDamperTorque : public ChLinkRotSpringCB::TorqueFunctor {
  public:
    LinearDamperTorque(double c) : m_c(c) {}
    virtual double operator()(double time, double angle, double vel, ChLinkRotSpringCB* link) override {
        return -m_c * vel;
    }

  private:
    double m_c;
};

/// Utility class for specifying a linear rotational spring-damper torque.
class LinearSpringDamperTorque : public ChLinkRotSpringCB::TorqueFunctor {
  public:
    LinearSpringDamperTorque(double k, double c, double rest_angle = 0) : m_k(k), m_c(c), m_rest_angle(rest_angle) {}
    virtual double operator()(double time, double angle, double vel, ChLinkRotSpringCB* link) override {
        return -m_k * (angle - m_rest_angle) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_rest_angle;
};

/// Utility class for specifying a linear rotational spring-damper torque with pre-tension.
class LinearSpringDamperActuatorTorque : public ChLinkRotSpringCB::TorqueFunctor {
  public:
    LinearSpringDamperActuatorTorque(double k, double c, double t, double rest_angle = 0)
        : m_k(k), m_c(c), m_t(t), m_rest_angle(rest_angle) {}
    virtual double operator()(double time, double angle, double vel, ChLinkRotSpringCB* link) override {
        return m_t - m_k * (angle - m_rest_angle) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_t;
    double m_rest_angle;
};

/// Utility class for specifying a map rotational spring torque.
class MapSpringTorque : public ChLinkRotSpringCB::TorqueFunctor {
  public:
    MapSpringTorque() {}
    MapSpringTorque(const std::vector<std::pair<double, double>>& data, double rest_angle = 0)
        : m_rest_angle(rest_angle) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double operator()(double time, double angle, double vel, ChLinkRotSpringCB* link) override {
        return -m_map.Get_y(angle - m_rest_angle);
    }

  private:
    ChFunction_Recorder m_map;
    double m_rest_angle;
};

/// Utility class for specifying a map rotational damper torque.
class MapDamperTorque : public ChLinkRotSpringCB::TorqueFunctor {
  public:
    MapDamperTorque() {}
    MapDamperTorque(const std::vector<std::pair<double, double>>& data) {
        for (unsigned int i = 0; i < data.size(); ++i) {
            m_map.AddPoint(data[i].first, data[i].second);
        }
    }
    void add_point(double x, double y) { m_map.AddPoint(x, y); }
    virtual double operator()(double time, double angle, double vel, ChLinkRotSpringCB* link) override {
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
    TMEASY       ///< Tire Model Made Easy tire (G. Rill)
};

/// Enum for available powertrain model templates.
enum class PowertrainModelType {
    SHAFTS,      ///< powertrain based on ChShaft elements
    SIMPLE_MAP,  ///< simple powertrain model (based on engine-map)
    SIMPLE,      ///< simple powertrain model (similar to a DC motor)
    SIMPLE_CVT   ///< simple cvt powertrain model (like a DC motor / CVT gearbox)
};

/// Enum for available wheeled-vehicle suspension model templates.
enum class SuspensionType {
    DOUBLE_WISHBONE,          ///< double wishbone
    DOUBLE_WISHBONE_REDUCED,  ///< simplified double wishbone (constraint-based)
    SOLID_AXLE,               ///< solid axle
    MULTI_LINK,               ///< multi-link
    HENDRICKSON_PRIMAXX,      ///< Hendrickson PRIMAXX (walking beam)
    MACPHERSON_STRUT,         ///< MacPherson strut
    SEMI_TRAILING_ARM,        ///< semi trailing arm
    THREE_LINK_IRS,           ///< three-link independent rear suspension
    RIGID_PINNED,             ///< pinned rigid beam
    RIGID_SUSPENSION          ///< rigid suspension
};

/// Enum for available wheeled-vehicle steering model templates.
enum class SteeringType {
    PITMAN_ARM,         ///< Pitman arm (input to revolute joint)
    PITMAN_ARM_SHAFTS,  ///< Pitman arm with compliant column (input to steering wheel)
    RACK_PINION         ///< rack-pinion (input to pinion)
};

/// Enum for drive types.
enum class DrivelineType {
    FWD,    ///< front-wheel drive
    RWD,    ///< rear-wheel drive
    AWD,    ///< all-wheel drive
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

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
