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
// Implementation of a single-wheel test rig.
// - Accepts an arbitrary wheel assembly (derived from ChWheelTestRig::Wheel)
// - Accepts a Chrono::Vehicle wheel - tire assembly
// - Works with Rigid, SCM, granular, or CRM terrain
// - Allows variation of longitudinal speed, wheel angular speed, and wheel slip
//   angle as functions of time
// - Provides support for automatic selection of longitudinal and angular speeds
//   in order to enforce a specified longitudinal slip value
// - Allows specification of camber angle (kept constant through the simulation)
//
// =============================================================================

#ifndef CH_WHEEL_TEST_RIG_H
#define CH_WHEEL_TEST_RIG_H

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"
#ifdef CHRONO_CRM
    #include "chrono_vehicle/terrain/CRMTerrain.h"
#endif

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Definition of a single-wheel test rig.
class CH_VEHICLE_API ChWheelTestRig {
  public:
    /// Definition of a wheel assembly for rig testing.
    class WheelAssembly {
      public:
        virtual ~WheelAssembly() {}

        virtual double GetMass() const = 0;
        virtual double GetRadius() const = 0;
        virtual double GetWidth() const = 0;
        virtual std::shared_ptr<ChBody> GetHub() const = 0;

#ifdef CHRONO_CRM
        virtual void AddFSIBodies(CRMTerrain& terrain, double spacing) { throw std::runtime_error("CreateBCEMarkers must be implemented when using CRMTerrain."); }
#endif

        virtual void Initialize(const ChFramed& frame, bool fixed, double step_size, VisualizationType vis_type) {}
        virtual void Synchronize(double time, const ChTerrain& terrain) {}
        virtual void Advance(double step_size) {}

        virtual TerrainForce ReportForces(ChTerrain& terrain) const { return TerrainForce(); }

      protected:
        WheelAssembly(ChSystem& system) : system(system) {}
        ChSystem& system;
    };

    /// Tire test rig operation mode.
    enum class Mode {
        SUSPEND,  ///< suspended wheel (locked spindle)
        DROP,     ///< wheel drop test
        TEST      ///< normal wheel test
    };

    /// Terrain type.
    enum class TerrainType { SCM, RIGID, CRG, GRANULAR, CRM, NONE };

    /// Terrain patch dimensions
    struct CH_VEHICLE_API TerrainPatchSize {
        TerrainPatchSize() : length(0), width(0), depth(0) {}
        double length;  ///< patch length
        double width;   ///< patch width
        double depth;   ///< patch depth
    };

    /// Rigid terrain patch parameters.
    struct CH_VEHICLE_API TerrainParamsRigid {
        float friction;       ///< coefficient of friction
        float Young_modulus;  ///< Young's modulus (Pa)
        float restitution;    ///< coefficient of restitution
    };

    /// SCM terrain patch parameters.
    struct CH_VEHICLE_API TerrainParamsSCM {
        double Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
        double Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
        double Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        double Mohr_cohesion;  ///< Cohesion in, Pa, for shear failure
        double Mohr_friction;  ///< Friction angle (in degrees!), for shear failure
        double Janosi_shear;   ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
        double grid_spacing;   ///< SCM grid spacing
    };

    /// Granular terrain patch parameters.
    struct CH_VEHICLE_API TerrainParamsGranular {
        double radius;         ///< particle radius
        double density;        ///< particle material density (kg/m3)
        double friction;       ///< inter-particle coefficient of friction
        double cohesion;       ///< inter-particle cohesion pressure (Pa)
        double Young_modulus;  ///< particle contact material Young's modulus (Pa)
    };

    /// Construct a test rig within the specified system using the given custom wheel.
    ChWheelTestRig(std::shared_ptr<WheelAssembly> wheel, ChSystem& system);

    /// Construct a test rig within the specified system using the given Chrono::Vehicle wheel and tire.
    ChWheelTestRig(std::shared_ptr<ChWheel> wheel,  ///< wheel subsystem
                   std::shared_ptr<ChTire> tire,    ///< tire subsystem
                   ChSystem& system                 ///< containing mechanical system
    );

    ~ChWheelTestRig();

    /// Set gravitational acceleration (default: 9.81 m/s2).
    void SetGravitationalAcceleration(double grav) { m_grav = grav; }

    /// Set desired normal load (default: 1000 N).
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

    /// Specify a constant given longitudinal slip. This version overrides the motion functions for the carrier
    /// longitudinal slip and for the wheel angular speed to enforce the specified longitudinal slip value. A positive
    /// slip value indicates that the wheel is spinning. A negative slip value indicates that the wheel is sliding
    /// (skidding); in particular, s=-1 indicates sliding without rotation.
    void SetConstantLongitudinalSlip(double long_slip, double base_speed = 1);

    /// Set the time step for advancing wheel assembly dynamics (default: 1e-3 s).
    void SetStepsize(double step) { m_step_size = step; }

    /// Set visualization type for the wheel assembly (default: PRIMITIVES).
    void SetVisualizationType(VisualizationType vis) { m_vis_type = vis; }

    // Terrain setup

    /// Enable use of rigid terrain.
    void SetTerrainRigid(const TerrainPatchSize& size, const TerrainParamsRigid& params);

    /// Enable use of rigid terrain.
    /// Specify contact material properties and patch dimensions.
    void SetTerrainRigid(const TerrainPatchSize& size,  ///< terrain patch size
                         double friction,               ///< coefficient of friction
                         double restitution,            ///< coefficient of restitution
                         double Young_modulus           ///< contact material Young's modulus [Pa]
    );

    /// Enable use of SCM terrain.
    void SetTerrainSCM(const TerrainPatchSize& size, const TerrainParamsSCM& params);

    /// Enable use of SCM terrain.
    /// Specify SCM soil parameters and patch dimensions.
    void SetTerrainSCM(const TerrainPatchSize& size,  ///< terrain patch size
                       double Bekker_Kphi,            ///< Kphi, frictional modulus in Bekker model
                       double Bekker_Kc,              ///< Kc, cohesive modulus in Bekker model
                       double Bekker_n,               ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
                       double Mohr_cohesion,          ///< cohesion [Pa], for shear failure
                       double Mohr_friction,          ///< Friction angle [degrees], for shear failure
                       double Janosi_shear,           ///< shear parameter J [m], (usually a few mm or cm)
                       double grid_spacing = 0.125    ///< SCM grid spacing
    );

    /// Enable use of granular terrain.
    /// The terrain subsystem consists of identical spherical particles initialized in layers.
    /// A moving-patch option is used.
    void SetTerrainGranular(const TerrainPatchSize& size, const TerrainParamsGranular& params);

    /// Enable use of granular terrain.
    /// Specify contact material properties.
    /// The terrain subsystem consists of identical spherical particles initialized in layers.
    /// A moving-patch option is used.
    void SetTerrainGranular(const TerrainPatchSize& size,  ///< terrain patch size
                            double radius,                 ///< particle radius [m]
                            double density,                ///< particle material density [kg/m3]
                            double friction,               ///< inter-particle coefficient of friction
                            double cohesion,               ///< inter-particle cohesion pressure [Pa]
                            double Young_modulus           ///< particle contact material Young's modulus [Pa]
    );

#ifdef CHRONO_CRM

    struct CH_VEHICLE_API TerrainParamsCRM {
        TerrainParamsCRM();
        fsi::sph::ChFsiFluidSystemSPH::SPHParameters sph_params;             ///< SPH solver settings
        fsi::sph::ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;  ///< soil properties
    };

    /// Enable use of CRM terrain.
    /// The terrain subsystem is modeled through continuum with CRM.
    void SetTerrainCRM(const TerrainPatchSize& size, const TerrainParamsCRM& params);

    /// Enable use of CRM terrain.
    void SetTerrainCRM(const TerrainPatchSize& size,  ///< terrain patch size
                       double spacing,                ///< SPH particle spacing
                       double density,                ///< material density
                       double Young_modulus,          ///< material Young's modulus
                       double friction,               ///< material internal friction
                       double cohesion                ///< material internal cohesion
    );

    /// Set size of the active box associated with the wheel (CRM terrain only).
    /// The default size is based on the wheel AABB inflated by 25%.
    void SetWheelActiveBox(const ChVector3d& size);

#endif

    /// Set time delay before applying motion functions (default: 0 s).
    /// In TEST mode, this delay is measured after the wheel bottom point reaches the terrain height.
    void SetTimeDelay(double delay) { m_time_delay = delay; }

    /// Get time delay after which measurements are calculated.
    /// After the drop phase (TEST mode only), this value is set to the input time delay increased by the wheel drop
    /// time. Before the drop phase is completed, this function returns the input time delay.
    double GetTimeDelay() const { return m_time_delay; }

    /// Check if kinematic and kinetic quantities are collected.
    /// This function returns true only in TEST mode once the motion functions are activated.
    bool OutputEnabled() const { return m_output; }

    /// Initialize the rig system.
    /// It is the user's responsibility to set the operation mode and motion functions in a consistent way.
    /// In TEST mode, the wheel is lowered onto the terrain with a speed equal to `drop_speed`.
    void Initialize(Mode mode, double drop_speed = 0.1);

    /// Get suggested collision settings.
    /// These values are meaningful only when using granular terrain and Chrono::Multicore.
    void GetSuggestedCollisionSettings(double& collision_envelope,  ///< suggested envelope based on particle radius
                                       ChVector3i& collision_bins   ///< suggested number of bins for broad-phase collision detection
    ) const;

    /// Advance system state by the specified time step.
    void Advance(double step);

    /// Get the normal load.
    double GetNormalLoad() const { return m_normal_load; }

    /// Get total rig mass.
    double GetMass() const { return m_total_mass; }

    /// Get a handle to the underlying terrain subsystem.
    std::shared_ptr<ChTerrain> GetTerrain() const { return m_terrain; }

    /// Get current carrier body position.
    const ChVector3d& GetPos() const { return m_carrier_body->GetPos(); }

    /// Get the current terrain forces acting on the wheel as applied to the wheel hub.
    TerrainForce ReportWheelForce() const;

    /// Return current drawbar-pull value.
    /// This is the reaction force in the linear motor used to enforce the specified rig longitudinal speed.
    double GetDBP() const;

    /// Get current wheel longitudinal slip.
    /// This value is calculated from the horizontal speed of the spindle body and its angular rotation speed.
    double GetLongitudinalSlip() const;

    /// Get current wheel slip angle.
    /// This value is calculated from the current spindle normal direction.
    double GetSlipAngle() const;

    /// Get current wheel camber angle.
    /// This value is calculated from the current spindle normal direction.
    double GetCamberAngle() const;

    /// Get the spindle object.
    std::shared_ptr<ChSpindle> GetSpindle() const { return m_spindle; }

    /// Get the linear motor used to actuate the carrier.
    std::shared_ptr<ChLinkMotorLinearSpeed> GetMotorCarrier() const { return m_lin_motor; }

    /// Get the rotation motor used to actuate the wheel.
    std::shared_ptr<ChLinkMotorRotationSpeed> GetMotorWheel() const { return m_rot_motor; }

  private:
    /// Implementation of a ChWheelTestRig::WheelAssembly that wraps a Chrono::Vehicle tire and wheel assembly.
    class VehicleWheel : public WheelAssembly {
      public:
        VehicleWheel(std::shared_ptr<ChWheel> wheel, std::shared_ptr<ChTire> tire, ChSystem& system);

        virtual double GetMass() const override;
        virtual double GetRadius() const override;
        virtual double GetWidth() const override;
        virtual std::shared_ptr<ChBody> GetHub() const override;

#ifdef CHRONO_CRM
        virtual void AddFSIBodies(CRMTerrain& terrain, double spacing) override;
#endif

        virtual void Initialize(const ChFramed& frame, bool fixed, double step_size, VisualizationType vis_type) override;
        virtual void Synchronize(double time, const ChTerrain& terrain) override;
        virtual void Advance(double step_size) override;

        virtual TerrainForce ReportForces(ChTerrain& terrain) const override;

      private:
        std::shared_ptr<ChSpindle> spindle;
        std::shared_ptr<ChWheel> wheel;
        std::shared_ptr<ChTire> tire;
    };

    void CreateMechanism();

    void CreateTerrain();
    void CreateTerrainSCM();
    void CreateTerrainRigid();
    void CreateTerrainGranular();
#ifdef CHRONO_CRM
    void CreateTerrainCRM();
#endif

    ChSystem& m_system;  ///< containing Chrono system

    Mode m_mode;    ///< testing mode
    bool m_output;  ///< if false, report default measurements (typically 0)

    std::shared_ptr<ChTerrain> m_terrain;    ///< handle to underlying terrain subsystem
    std::shared_ptr<WheelAssembly> m_wheel;  ///< wheel assembly
    VisualizationType m_vis_type;            ///< visualization type for wheel assembly
    double m_step_size;                      ///< step size for wheel assembly integration
    double m_camber_angle;                   ///< wheel camber angle

    double m_grav;         ///< gravitational acceleration
    double m_normal_load;  ///< desired normal load
    double m_total_mass;   ///< total sprung mass
    double m_time_delay;   ///< time delay before applying external load

    TerrainType m_terrain_type;               ///< terrain type
    double m_terrain_offset;                  ///< Y coordinate of wheel center
    double m_terrain_height;                  ///< height coordinate for terrain subsystem
    TerrainPatchSize m_terrain_size;          ///< terrain patch dimensions
    TerrainParamsSCM m_params_SCM;            ///< SCM soil parameters
    TerrainParamsRigid m_params_rigid;        ///< rigid terrain contact material properties
    TerrainParamsGranular m_params_granular;  ///< granular terrain parameters
#ifdef CHRONO_CRM
    TerrainParamsCRM m_params_crm;  ///< granular terrain parameters
#endif

    bool m_default_AABB;
    ChVector3d m_AABB_size;

    std::shared_ptr<ChBody> m_ground_body;   ///< ground body
    std::shared_ptr<ChBody> m_carrier_body;  ///< rig carrier body
    std::shared_ptr<ChBody> m_chassis_body;  ///< "chassis" body which carries normal load
    std::shared_ptr<ChBody> m_slip_body;     ///< intermediate body for controlling slip angle
    std::shared_ptr<ChSpindle> m_spindle;    ///< wheel spindle

    bool m_ls_actuated;                    ///< is linear speed actuated?
    bool m_rs_actuated;                    ///< is angular speed actuated?
    std::shared_ptr<ChFunction> m_ls_fun;  ///< longitudinal speed function of time
    std::shared_ptr<ChFunction> m_rs_fun;  ///< angular speed function of time
    std::shared_ptr<ChFunction> m_sa_fun;  ///< slip angle function of time

    bool m_long_slip_constant;  ///< true if constant longitudinal slip was specified
    double m_long_slip;         ///< user-specified longitudinal slip
    double m_base_speed;        ///< base speed for long slip calculation

    std::shared_ptr<ChLinkMotorLinearSpeed> m_drop_motor;   ///< actuator for controlled wheel drop
    std::shared_ptr<ChLinkMotorLinearSpeed> m_lin_motor;    ///< carrier actuator
    std::shared_ptr<ChLinkMotorRotationSpeed> m_rot_motor;  ///< wheel actuator
    std::shared_ptr<ChLinkLockLock> m_slip_lock;            ///< slip angle actuator
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
