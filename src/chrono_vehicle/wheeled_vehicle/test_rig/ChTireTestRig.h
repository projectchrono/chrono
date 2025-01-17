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

#include "chrono/ChConfig.h"
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
    /// Tire test rig operation mode.
    enum class Mode {
        SUSPEND,  ///< suspended tire (locked spindle)
        DROP,     ///< tire drop test
        TEST      ///< normal tire test
    };

    /// Rigid terrain patch parameters.
    struct TerrainParamsRigid {
        float friction;       ///< coefficient of friction
        float Young_modulus;  ///< Young's modulus (Pa)
        float restitution;    ///< coefficient of restitution
        double length;        ///< patch length
        double width;         ///< patch width
    };

    /// SCM terrain patch parameters.
    struct TerrainParamsSCM {
        double Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
        double Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
        double Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        double Mohr_cohesion;  ///< Cohesion in, Pa, for shear failure
        double Mohr_friction;  ///< Friction angle (in degrees!), for shear failure
        double Janosi_shear;   ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
        double length;         ///< patch length
        double width;          ///< patch width
        double grid_spacing;   ///< SCM grid spacing
    };

    /// Granular terrain patch parameters.
    struct TerrainParamsGranular {
        double radius;            ///< particle radius
        unsigned int num_layers;  ///< number of layers for initial particle creation
        double density;           ///< particle material density (kg/m3)
        double friction;          ///< inter-particle coefficient of friction
        double cohesion;          ///< inter-particle cohesion pressure (Pa)
        double Young_modulus;     ///< particle contact material Young's modulus (Pa)
        double width;             ///< patch width
    };

    struct TerrainParamsCRM {
        double radius;    ///< particle radius
        double density;   ///< Terrain bulk density (kg/m3)
        double cohesion;  ///< Terrain artificial cohesion (Pa)
        double length;    ///< patch length
        double width;     ///< patch width
        double depth;     ///< patch depth
    };

    /// Construct a tire test rig within the specified system.
    ChTireTestRig(std::shared_ptr<ChWheel> wheel,  ///< wheel subsystem
                  std::shared_ptr<ChTire> tire,    ///< tire subsystem
                  ChSystem* system                 ///< containing mechanical system
    );

    ~ChTireTestRig();

    /// Set gravitational acceleration (default: 9.81 m/s2).
    void SetGravitationalAcceleration(double grav) { m_grav = grav; }

    /// Set desired normal load (default: 1000 N).
    void SetNormalLoad(double load) { m_normal_load = load; }

    /// Get the normal load.
    double GetNormalLoad(double load) const { return m_normal_load; }

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

    /// Specify a constant given longitudinal slip. This version overrides the motion functions for the carrier
    /// longitudinal slip and for the wheel angular speed to enfore the specified longitudinalslip value. A positive
    /// slip value indicates that the wheel is spinning. A negative slip value indicates that the wheel is sliding
    /// (skidding); in particular, s=-1 indicates sliding without rotation.
    void SetConstantLongitudinalSlip(double long_slip, double base_speed = 1);

    /// Set collision type for tire-terrain interaction (default: SINGLE_POINT).
    void SetTireCollisionType(ChTire::CollisionType coll_type);

    /// Set the time step for advancing tire dynamics (default: 1e-3 s).
    void SetTireStepsize(double step) { m_tire_step = step; }

    /// Set visualization type for the tire (default: PRIMITIVES).
    void SetTireVisualizationType(VisualizationType vis) { m_tire_vis = vis; }

    /// Enable use of rigid terrain.
    void SetTerrainRigid(const TerrainParamsRigid& params);

    /// Enable use of rigid terrain.
    /// Specify contact material properties and patch dimensions.
    void SetTerrainRigid(double friction,             ///< coefficient of friction
                         double restitution,          ///< coefficient of restitution
                         double Young_modulus,        ///< contact material Young's modulus [Pa]
                         double terrain_length = 10,  ///< length of terrain patch [m]
                         double terrain_width = 1     ///< width of terrain patch
    );

    /// Enable use of SCM terrain.
    void SetTerrainSCM(const TerrainParamsSCM& params);

    /// Enable use of SCM terrain.
    /// Specify SCM soil parameters and patch dimensions.
    void SetTerrainSCM(double Bekker_Kphi,           ///< Kphi, frictional modulus in Bekker model
                       double Bekker_Kc,             ///< Kc, cohesive modulus in Bekker model
                       double Bekker_n,              ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
                       double Mohr_cohesion,         ///< cohesion [Pa], for shear failure
                       double Mohr_friction,         ///< Friction angle [degrees], for shear failure
                       double Janosi_shear,          ///< shear parameter J [m], (usually a few mm or cm)
                       double grid_spacing = 0.125,  ///< SCM grid spacing
                       double terrain_length = 10,   ///< length of terrain patch
                       double terrain_width = 1      ///< width of terrain patch
    );

    /// Enable use of granular terrain.
    /// The terrain subsystem consists of identical spherical particles initialized in layers.
    /// A moving-patch option is used, with the patch length set based on the tire dimensions.
    void SetTerrainGranular(const TerrainParamsGranular& params);

    /// Enable use of granular terrain.
    /// Specify contact material properties.
    /// The terrain subsystem consists of identical spherical particles initialized in layers.
    /// A moving-patch option is used, with the patch length set based on the tire dimensions.
    void SetTerrainGranular(double radius,            ///< particle radius [m]
                            unsigned int num_layers,  ///< number of layers for initial particle creation
                            double density,           ///< particle material density [kg/m3]
                            double friction,          ///< inter-particle coefficient of friction
                            double cohesion,          ///< inter-particle cohesion pressure [Pa]
                            double Young_modulus,     ///< particle contact material Young's modulus [Pa]
                            double terrain_width = 1  ///< width of terrain patch
    );

    /// Enable use of CRM terrain.
    // The terrain subsystem is modelled through continuum with CRM.
    // Other material and SPH parameters are left to CRM defaults
    void SetTerrainCRM(const TerrainParamsCRM& params);

    // Enable use of CRM terrain.
    // The radius here is the radius of the SPH markers (SPH is the underlying solver for the continuum PDEs)
    void SetTerrainCRM(double radius,
                       double density,
                       double cohesion,
                       double terrain_length = 10,
                       double terrain_width = 1,
                       double terrain_depth = 0.2);

    /// Set time delay before applying motion functions (default: 0 s).
    void SetTimeDelay(double delay) { m_time_delay = delay; }

    /// Initialize the rig system.
    /// It is the user's responsibility to set the operation mode and motion functions in a consistent way.
    void Initialize(Mode mode);

    /// Get suggested collision settings.
    /// These values are meaningful only when using granular terrain and Chrono::Multicore.
    void GetSuggestedCollisionSettings(
        double& collision_envelope,  ///< suggested envelope based on particle radius
        ChVector3i& collision_bins   ///< suggested number of bins for broad-pahse collision detection
    ) const;

    /// Advance system state by the specified time step.
    void Advance(double step);

    /// Get total rig mass.
    double GetMass() const { return m_total_mass; }

    /// Get a handle to the underlying terrain subsystem.
    std::shared_ptr<ChTerrain> GetTerrain() const { return m_terrain; }

    /// Get current carrier body position.
    const ChVector3d& GetPos() const { return m_carrier_body->GetPos(); }

    /// Get the current tire forces
    TerrainForce ReportTireForce() const;

    /// Return current drawbar-pull value.
    /// This is the reaction force in the linear motor used to enforce the specified rig longitudinal speed.
    double GetDBP() const;

  private:
    enum class TerrainType { SCM, RIGID, CRG, GRANULAR, CRM, NONE };

    void CreateMechanism(Mode mode);

    void CreateTerrain();
    void CreateTerrainSCM();
    void CreateTerrainRigid();
    void CreateTerrainGranular();
    void CreateTerrainCRM();

    ChSystem* m_system;  ///< pointer to the Chrono system

    std::shared_ptr<ChTerrain> m_terrain;  ///< handle to underlying terrain subsystem
    std::shared_ptr<ChWheel> m_wheel;      ///< handle to wheel subsystem
    std::shared_ptr<ChTire> m_tire;        ///< handle to tire subsystem
    VisualizationType m_tire_vis;          ///< visualization type for tire subsystem
    double m_tire_step;                    ///< step size for tire integration
    double m_camber_angle;                 ///< camber angle

    double m_grav;         ///< gravitational acceleration
    double m_normal_load;  ///< desired normal load
    double m_total_mass;   ///< total sprung mass
    double m_time_delay;   ///< time delay before applying external load

    TerrainType m_terrain_type;               ///< terrain type
    TerrainParamsSCM m_params_SCM;            ///< SCM soil parameters
    TerrainParamsRigid m_params_rigid;        ///< rigid terrain contact material properties
    TerrainParamsGranular m_params_granular;  ///< granular terrain parameters
    TerrainParamsCRM m_params_crm;            ///< granular terrain parameters
    double m_terrain_offset;                  ///< Y coordinate of tire center
    double m_terrain_height;                  ///< height coordinate for terrain subsystem

    std::shared_ptr<ChBody> m_ground_body;   ///< ground body
    std::shared_ptr<ChBody> m_carrier_body;  ///< rig carrier body
    std::shared_ptr<ChBody> m_chassis_body;  ///< "chassis" body which carries normal load
    std::shared_ptr<ChBody> m_slip_body;     ///< intermediate body for controlling slip angle
    std::shared_ptr<ChBody> m_spindle_body;  ///< wheel spindle body

    bool m_ls_actuated;                    ///< is linear spped actuated?
    bool m_rs_actuated;                    ///< is angular speed actuated?
    std::shared_ptr<ChFunction> m_ls_fun;  ///< longitudinal speed function of time
    std::shared_ptr<ChFunction> m_rs_fun;  ///< angular speed function of time
    std::shared_ptr<ChFunction> m_sa_fun;  ///< slip angle function of time

    bool m_long_slip_constant;  ///< true if constant longitudinal slip was specified
    double m_long_slip;         ///< user-specified longitudinal slip
    double m_base_speed;        ///< base speed for long slip calculation

    std::shared_ptr<ChLinkMotorLinearSpeed> m_lin_motor;    ///< carrier actuator
    std::shared_ptr<ChLinkMotorRotationSpeed> m_rot_motor;  ///< wheel actuator
    std::shared_ptr<ChLinkLockLock> m_slip_lock;            ///< slip angle actuator
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
