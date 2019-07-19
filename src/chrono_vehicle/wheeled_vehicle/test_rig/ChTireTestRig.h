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
// Implementation of a single-tire test rig.
// - Accepts an arbitrary Chrono::Vehicle tire object
//   (and associated ChWheel object)
// - Currently works with SCM or Rigid terrain
// - Allows variation of longitudinal speed, wheel angular speed, and wheel slip
//   angle as functions of time
// - Provides support for automatic selection of longitudinal and angular speeds
//   in order to enforce a specified longitudinal slip value
// - Allows specification of camber angle (kept constant through the simulation)
//
// =============================================================================

#ifndef CH_TIRE_TEST_RIG_H
#define CH_TIRE_TEST_RIG_H

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Definition of a single-tire test rig.
class CH_VEHICLE_API ChTireTestRig {
  public:
    //// Construct a tire test rig within the specified system.
    ChTireTestRig(std::shared_ptr<ChWheel> wheel,  ///< wheel subsystem
                  std::shared_ptr<ChTire> tire,    ///< tire subsystem
                  ChSystem* system                 ///< containing mechanical system
    );

    /// Set gravitational acceleration (default: 9.81 m/s2).
    void SetGravitationalAcceleration(double grav) { m_grav = grav; }

    /// Set desired normal load (default: 0 N).
    void SetNormalLoad(double load) { m_normal_load = load; }

    /// Set camber angle (default: 0 rad).
    void SetCamberAngle(double camber) { m_camber_angle = camber; }

    /// Specify rig carrier longitudinal speed as function of time (default: none).
    /// If a function is not specified, the carrier is not actuated.
    void SetLongSpeedFunction(std::shared_ptr<ChFunction> funct);

    /// Specify wheel angular speed as function of time (default: none).
    /// If a function is not specified, the wheel is not actuated.
    void SetAngSpeedFunction(std::shared_ptr<ChFunction> funct);

    /// Specify wheel slip angle as function of time (default: constant value 0 rad).
    void SetSlipAngleFunction(std::shared_ptr<ChFunction> funct) { m_sa_fun = funct; }

    /// Set collision type for tire-terrain interaction (default: SINGLE_POINT).
    void SetTireCollisionType(ChTire::CollisionType coll_type) { m_collision_type = coll_type; }

    /// Set the time step for advancing tire dynamics (default: 1e-3 s).
    void SetTireStepsize(double step) { m_tire_step = step; }

    /// Set visualization type for the tire (default: PRIMITIVES).
    void SetTireVisualizationType(VisualizationType vis) { m_tire_vis = vis; }

    /// Enable use of SCM terrain. Specify SCM soil parameters.
    void SetTerrainSCM(double Bekker_Kphi,         ///< Kphi, frictional modulus in Bekker model
                       double Bekker_Kc,           ///< Kc, cohesive modulus in Bekker model
                       double Bekker_n,            ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
                       double Mohr_cohesion,       ///< cohesion [Pa], for shear failure
                       double Mohr_friction,       ///< Friction angle [degrees], for shear failure
                       double Janosi_shear,        ///< shear parameter J [m], (usually a few mm or cm)
                       double terrain_length = 10  ///< length of terrain patch
    );

    /// Enable use of rigid terrain. Specify contact material properties.
    void SetTerrainRigid(double friction,            ///< coefficient of friction
                         double restitution,         ///< coefficient of restitution
                         double Young_modulus,       ///< contact material Young's modulus [Pa]
                         double terrain_length = 10  ///< length of terrain patch [m]
    );

    /// Enable use of granular terrain.
    /// The terrain subsystem consists of identical spherical particles initialized in layers.
    /// A moving-patch option is used, with the patch dimensions set based on the tire dimensions.
    void SetTerrainGranular(double radius,            ///< particle radius [m]
                            unsigned int num_layers,  ///< number of layers for initial particle creation
                            double density,           ///< particle material density [kg/m3]
                            double friction,          ///< inter-particle coefficient of friction
                            double cohesion,          ///< inter-particle cohesion pressure [Pa]
                            double Young_modulus      ///< particle contact material Young's modulus [Pa]
    );

    /// Set time delay before releasing the wheel (default: 0s).
    void SetTimeDelay(double delay) { m_time_delay = delay; }

    /// Initialize the rig system. This version uses all motion functions as specified by the user. It is the user's
    /// responsibility to set these up for a meaningful test.
    void Initialize();

    /// Initialize the rig system for a simulation with given longitudinal slip. This version overrides the motion
    /// functions for the carrier longitudinal slip and for the wheel angular speed to enfore the specified longitudinal
    /// slip value. A positive slip value indicates that the wheel is spinning. A negative slip value indicates that the
    /// wheel is sliding (skidding); in particular, s=-1 indicates sliding without rotation.
    void Initialize(double long_slip, double base_speed = 1);

    /// Get suggested collision settings.
    /// These values are meaningful only when using granular terrain and Chrono::Parallel.
    void GetSuggestedCollisionSettings(
        double& collision_envelope,    ///< suggested envelope based on particle radius
        ChVector<int>& collision_bins  ///< suggested number of bins for broad-pahse collision detection
    ) const;

    /// Advance system state by the specified time step.
    void Advance(double step);

    /// Get total rig mass.
    double GetTotalMass() const { return m_total_mass; }

    /// Get applied load on rig
    /// (to enforce specified normal load, taking into account the masses of all components).
    double GetAppliedLoad() const { return m_applied_load; }

    /// Get a handle to the underlying terrain subsystem.
    std::shared_ptr<ChTerrain> GetTerrain() const { return m_terrain; }

    /// Get current carrier body position.
    const ChVector<>& GetPos() const { return m_carrier_body->GetPos(); }

    /// Get the current tire forces
    TerrainForce GetTireForce() const;

  private:
    enum class TerrainType { SCM, RIGID, CRG, GRANULAR, NONE };

    struct TerrainParamsSCM {
        double Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
        double Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
        double Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        double Mohr_cohesion;  ///< Cohesion in, Pa, for shear failure
        double Mohr_friction;  ///< Friction angle (in degrees!), for shear failure
        double Janosi_shear;   ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
        double length;
        double width;
    };

    struct TerrainParamsRigid {
        float friction;       ///< coefficient of friction
        float Young_modulus;  ///< Young's modulus (Pa)
        float restitution;    ///< coefficient of restitution
        double length;
        double width;
    };

    struct TerrainParamsGranular {
        double radius;            ///< particle radius
        unsigned int num_layers;  ///< number of layers for initial particle creation
        double density;           ///< particle material density (kg/m3)
        double friction;          ///< inter-particle coefficient of friction
        double cohesion;          ///< inter-particle cohesion pressure (Pa)
        double Young_modulus;     ///< particle contact material Young's modulus (Pa)
        double length;
        double width;
    };

    void CreateMechanism();

    void CreateTerrain();
    void CreateTerrainSCM();
    void CreateTerrainRigid();
    void CreateTerrainGranular();

    ChSystem* m_system;  ///< pointer to the Chrono system

    std::shared_ptr<ChTerrain> m_terrain;    ///< handle to underlying terrain subsystem
    std::shared_ptr<ChWheel> m_wheel;        ///< handle to wheel subsystem
    std::shared_ptr<ChTire> m_tire;          ///< handle to tire subsystem
    VisualizationType m_tire_vis;            ///< visualization type for tire subsystem
    ChTire::CollisionType m_collision_type;  ///< tire-terrain collision method
    double m_tire_step;                      ///< step size for tire integration
    double m_camber_angle;                   ///< camber angle

    double m_grav;          ///< gravitational acceleration
    double m_normal_load;   ///< desired normal load
    double m_applied_load;  ///< applied load on chassis body
    double m_total_mass;    ///< total sprung mass
    double m_time_delay;    ///< time delay before applying external load 

    TerrainType m_terrain_type;               ///< terrain type
    TerrainParamsSCM m_params_SCM;            ///< SCM soil parameters
    TerrainParamsRigid m_params_rigid;        ///< rigid terrain contact material properties
    TerrainParamsGranular m_params_granular;  ///< granular terrain parameters

    double m_terrain_offset;  ///< Y coordinate of tire center
    double m_terrain_height;  ///< height coordinate for terrain subsystem

    std::shared_ptr<ChBody> m_ground_body;   ///< ground body
    std::shared_ptr<ChBody> m_carrier_body;  ///< rig carrier body
    std::shared_ptr<ChBody> m_chassis_body;  ///< "chassis" body which carries normal load
    std::shared_ptr<ChBody> m_slip_body;     ///< intermediate body for controlling slip angle
    std::shared_ptr<ChBody> m_spindle_body;  ///< wheel body

    bool m_ls_actuated;                    ///< is linear spped actuated?
    bool m_rs_actuated;                    ///< is angular speed actuated?
    std::shared_ptr<ChFunction> m_ls_fun;  ///< longitudinal speed function of time
    std::shared_ptr<ChFunction> m_rs_fun;  ///< angular speed function of time
    std::shared_ptr<ChFunction> m_sa_fun;  ///< slip angle function of time

    std::shared_ptr<ChLinkMotorLinearSpeed> m_lin_motor;    ///< carrier actuator
    std::shared_ptr<ChLinkMotorRotationSpeed> m_rot_motor;  ///< wheel actuator
    std::shared_ptr<ChLinkLockLock> m_slip_lock;            ///< slip angle actuator
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
