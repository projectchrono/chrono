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
// - Accepts an arbitrary wheel assembly (derived from ChWheelTestRig::Wheel).
// - Accepts a Chrono::Vehicle wheel-tire assembly.
// - The wheel assembly can be optionally attached to a suspension assembly.
// - Accepts a Chrono::Vehicle suspension assembly.
// - Works with Rigid, SCM, DEM granular, or CRM terrain.
// - Allows variation of longitudinal speed, wheel angular speed, and wheel slip
//   angle as functions of time.
// - Provides support for automatic selection of longitudinal and angular speeds
//   in order to enforce a specified longitudinal slip value.
// - Allows specification of camber angle (constant through the simulation).
//
// =============================================================================

#ifndef CH_WHEEL_TEST_RIG_H
#define CH_WHEEL_TEST_RIG_H

#include <vector>

#include "chrono/input_output/ChOutput.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsMotorSpeed.h"
#include "chrono/functions/ChFunctionInterp.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

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

/// Base class definition of a single-wheel test rig.
/// - Accepts an arbitrary wheel assembly (derived from ChWheelTestRig::WheelAssembly)
/// - Works with Rigid, SCM, DEM granular, or CRM terrain (deformable terrain requires a rigid or FEA tire; handling tires
///   can only be used with rigid terrain)
/// - Allows variation of longitudinal speed, wheel angular speed, and wheel slip angle as functions of time
/// - Provides support for automatic selection of longitudinal and angular speeds in order to enforce a specified longitudinal slip value
/// - Allows specification of camber angle (kept constant through the simulation)
class CH_VEHICLE_API ChWheelTestRig {
  public:
    /// Steering actuator of a wheel assembly.
    /// The actuator coordinate is the scalar input of the steering mechanism (e.g., a rack displacement or a crank angle),
    /// with a value of zero in the initial configuration of the wheel assembly.
    class CH_VEHICLE_API SteeringActuator {
      public:
        virtual ~SteeringActuator() {}

        /// Set the actuator coordinate as a function of time.
        virtual void SetFunction(std::shared_ptr<ChFunction> fun) = 0;

        /// Get the current actuator coordinate.
        virtual double GetCoordinate() const = 0;
    };

    /// Steering actuator based on a linear position motor (actuator coordinate: motor displacement).
    class CH_VEHICLE_API SteeringActuatorLinear : public SteeringActuator {
      public:
        SteeringActuatorLinear(std::shared_ptr<ChLinkMotorLinearPosition> motor) : m_motor(motor) {}
        virtual void SetFunction(std::shared_ptr<ChFunction> fun) override { m_motor->SetMotionFunction(fun); }
        virtual double GetCoordinate() const override { return m_motor->GetMotorPos(); }

      private:
        std::shared_ptr<ChLinkMotorLinearPosition> m_motor;
    };

    /// Steering actuator based on a rotation angle motor (actuator coordinate: motor angle).
    class CH_VEHICLE_API SteeringActuatorRotational : public SteeringActuator {
      public:
        SteeringActuatorRotational(std::shared_ptr<ChLinkMotorRotationAngle> motor) : m_motor(motor) {}
        virtual void SetFunction(std::shared_ptr<ChFunction> fun) override { m_motor->SetAngleFunction(fun); }
        virtual double GetCoordinate() const override { return m_motor->GetMotorAngle(); }

      private:
        std::shared_ptr<ChLinkMotorRotationAngle> m_motor;
    };

    /// Definition of a wheel assembly for rig testing.
    /// Requirements for a wheel assembly implementation:
    /// - GetHub() must return the body carrying the wheel, with the wheel spin axis along the body Y axis; the rig
    ///   calculates the slip, camber, and steering angles from its orientation.
    /// - GetMass() must return the total mass supported by the wheel (wheel and, if present, suspension and steering
    ///   mechanism), as available after Initialize(). GetWheelMass() is used only to size rig bodies.
    /// - GetRadius() and GetWidth() must be available after Initialize().
    /// - AddFSIBodies() must be implemented to use the assembly with CRM terrain.
    /// - ReportForces() should report the terrain force on the wheel (see ChWheelTestRig::CalculateTerrainForce). With a
    ///   suspension, the rig uses it in TEST mode to end the drop phase once the wheel carries the normal load; if an
    ///   assembly reports no force, the drop phase ends when the wheel reaches the terrain.
    /// - A steerable assembly must create its steering mechanism in Initialize() and provide an actuator for it through
    ///   GetSteeringActuator().
    class WheelAssembly {
      public:
        virtual ~WheelAssembly() {}

        /// Return `true` if the assembly includes a suspension subsystem and `false` otherwise.
        virtual bool HasSuspension() const = 0;

        /// Return the steering actuator of the wheel assembly, or null if the assembly cannot be steered (default).
        /// This function is called after Initialize(). The rig holds the actuator at zero unless a steering input is
        /// specified (see ChWheelTestRig::SetSteeringAngleFunction and ChWheelTestRig::SetSteeringActuatorFunction).
        virtual std::shared_ptr<SteeringActuator> GetSteeringActuator() const { return nullptr; }

        /// Return the total mass of the wheel assembly.
        virtual double GetMass() const = 0;

        /// Return the mass of the wheel.
        virtual double GetWheelMass() const = 0;

        /// Return the wheel assembly outer radius.
        virtual double GetRadius() const = 0;

        /// Return the wheel assembly width.
        virtual double GetWidth() const = 0;

        /// Get a handle to the hub body.
        virtual std::shared_ptr<ChBody> GetHub() const = 0;

#ifdef CHRONO_CRM
        virtual void AddFSIBodies(CRMTerrain& terrain, double spacing) { throw std::runtime_error("CreateBCEMarkers must be implemented when using CRMTerrain."); }
#endif

        /// Initialize the wheel assembly.
        /// An assembly that includes a suspension subsystem should attach the suspension to the provided chassis body at the provided frame.
        /// An assembly without a suspension subsystem should only create the wheel at the provided frame and, if indicated, set it as fixed.
        virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis_body, const ChFramed& frame, bool fixed_wheel, double step_size, VisualizationType vis_type) {}

        /// Synchronize the wheel assembly at the current simulation time.
        virtual void Synchronize(double time, const ChTerrain& terrain) {}

        /// Advance the wheel assembly dynamics (if any) by the given step size.
        virtual void Advance(double step_size) {}

        /// Report the current resultant terrain forces and torques as applied to the hub.
        virtual TerrainForce ReportForces(ChTerrain& terrain) const { return TerrainForce(); }

        /// Append assembly-specific output channels to the given list (default: none).
        /// The set of channels (names and order) must be the same at every call. Values should be in SI units.
        virtual void AddOutputChannels(std::vector<ChOutput::Channel>& channels) const {}

        /// Remove from the containing system all physics items created by this assembly in Initialize().
        /// This function is called by the test rig destructor. It should remove only items that the wheel assembly
        /// created directly; subsystems that remove their own items on destruction (e.g., wheel, tire, suspension) are not affected.
        virtual void RemoveFromSystem() {}

      protected:
        /// Construct the wheel assembly in the specified Chrono system.
        /// The constructor must create and set all wheel assembly parts so that its properties can be queried even before initialization.
        WheelAssembly(ChSystem& system) : system(system) {}

        ChSystem& system;  ///< containing system
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
        TerrainPatchSize(double length, double width, double depth = 0) : length(length), width(width), depth(depth) {}
        double length;  ///< patch length
        double width;   ///< patch width
        double depth;   ///< patch depth
    };

    /// Rigid terrain patch parameters.
    struct CH_VEHICLE_API TerrainParamsRigid {
        TerrainParamsRigid() : mu(0.8f), cr(0.0f), Y(2e7f) {}
        TerrainParamsRigid(float mu, float cr, float Y) : mu(mu), cr(cr), Y(Y) {}
        float mu;  ///< coefficient of friction
        float cr;  ///< coefficient of restitution
        float Y;   ///< Young's modulus (Pa)
    };

    /// SCM terrain patch parameters.
    /// The default soil parameters match the SCMTerrain defaults.
    struct CH_VEHICLE_API TerrainParamsSCM {
        TerrainParamsSCM() : Bekker_Kphi(2e6), Bekker_Kc(0), Bekker_n(1.1), Mohr_cohesion(50), Mohr_friction(20), Janosi_shear(0.01), grid_spacing(0.125), vis_mesh(true) {}
        double Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
        double Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
        double Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        double Mohr_cohesion;  ///< Cohesion in, Pa, for shear failure
        double Mohr_friction;  ///< Friction angle (in degrees!), for shear failure
        double Janosi_shear;   ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
        double grid_spacing;   ///< SCM grid spacing
        bool vis_mesh;         ///< visualization mesh
    };

    /// Granular terrain patch parameters.
    struct CH_VEHICLE_API TerrainParamsGranular {
        TerrainParamsGranular() : radius(0.02), density(2000), friction(0.9), cohesion(0), Young_modulus(1e7) {}
        double radius;         ///< particle radius
        double density;        ///< particle material density (kg/m3)
        double friction;       ///< inter-particle coefficient of friction
        double cohesion;       ///< inter-particle cohesion pressure (Pa)
        double Young_modulus;  ///< particle contact material Young's modulus (Pa), used with SMC contact
    };

    /// Construct a test rig within the specified system using the given custom wheel.
    ChWheelTestRig(ChSystem& system, std::shared_ptr<WheelAssembly> wheel);

    /// Construct a test rig within the specified system using the given Chrono::Vehicle wheel and tire.
    ChWheelTestRig(ChSystem& system,                ///< containing mechanical system
                   std::shared_ptr<ChWheel> wheel,  ///< wheel subsystem
                   std::shared_ptr<ChTire> tire     ///< tire subsystem
    );

    /// Construct a test rig within the specified system using the given Chrono::Vehicle wheel, tire, and suspension subsystems.
    ChWheelTestRig(ChSystem& system,                         ///< containing mechanical system
                   std::shared_ptr<ChWheel> wheel,           ///< wheel subsystem
                   std::shared_ptr<ChTire> tire,             ///< tire subsystem
                   std::shared_ptr<ChSuspension> suspension  ///< suspension subsystem
    );

    ~ChWheelTestRig();

    /// Set gravitational acceleration (default: 9.81 m/s2).
    void SetGravitationalAcceleration(double grav) { m_grav = grav; }

    /// Set camber angle (default: 0 rad).
    /// Camber angle is set only if the wheel is not attached to a suspension subsystem
    /// (in that case, the suspension geometry dictates camber).
    void SetCamberAngle(double camber) { m_camber_angle = camber; }

    /// Set desired normal load (default: 1000 N).
    void SetNormalLoad(double load) { m_normal_load = load; }

    /// Specify rig carrier longitudinal speed as function of time (default: none).
    /// If a function is not specified, the carrier is not actuated.
    void SetLongSpeedFunction(std::shared_ptr<ChFunction> funct);

    /// Specify wheel angular speed as function of time (default: none).
    /// If a function is not specified, the wheel is not actuated.
    void SetAngSpeedFunction(std::shared_ptr<ChFunction> funct);

    /// Specify a constant given longitudinal slip. This version overrides the motion functions for the carrier
    /// longitudinal slip and for the wheel angular speed to enforce the specified longitudinal slip value. A positive
    /// slip value indicates that the wheel is spinning. A negative slip value indicates that the wheel is sliding
    /// (skidding); in particular, s=-1 indicates sliding without rotation.
    void SetConstantLongitudinalSlip(double long_slip, double base_speed = 1);

    /// Specify wheel slip angle as function of time (default: constant value 0).
    /// A slip angle function can be used only if the wheel is not attached to a suspension subsystem
    /// (in that case, the suspension dictates slip angle).
    void SetSlipAngleFunction(std::shared_ptr<ChFunction> funct) { m_sa_fun = funct; }

    /// Specify wheel steering angle as function of time (default: none).
    /// Steering requires a wheel assembly with a steering actuator (see WheelAssembly::GetSteeringActuator) and is ignored
    /// otherwise; for a wheel without a suspension, use SetSlipAngleFunction instead. The wheel is steered through the
    /// assembly steering mechanism (for a Chrono::Vehicle steerable suspension, a rack to which the tierods are attached).
    /// The actuator coordinate is obtained from the specified steering angle through a kinematic map, computed during
    /// initialization at the loaded configuration of the suspension. Steering angles are relative to that configuration
    /// and positive to the left (positive rotation about the vertical axis). Away from it, the actual steering angle
    /// includes bump and compliance steer.
    /// Note that a zero steering angle retains the static toe of the suspension in the loaded configuration (in general
    /// different from the design toe, because of bump steer), so that the wheel may run at a non-zero slip angle.
    /// This function overrides any steering actuator function.
    void SetSteeringAngleFunction(std::shared_ptr<ChFunction> funct);

    /// Specify the steering actuator coordinate as function of time (default: none).
    /// The actuator coordinate is specific to the steering mechanism of the wheel assembly (for a Chrono::Vehicle
    /// steerable suspension, the displacement of the steering rack along the rig lateral axis). See
    /// SetSteeringAngleFunction for the steering requirements.
    /// This function overrides any steering angle function.
    void SetSteeringActuatorFunction(std::shared_ptr<ChFunction> funct);

    /// Enable feedback correction of the steering actuator coordinate (default: false).
    /// Used only with a steering angle function. An integral correction, normalized by the local slope of the kinematic
    /// steering map, drives the actual steering angle to the specified one with the given time constant. Note that this
    /// cancels bump and compliance steer.
    void EnableSteeringFeedback(bool enable, double time_constant = 0.05);

    /// Set the step size for the multibody system and the wheel assembly dynamics (default: 1e-3 s).
    /// With CRM terrain, the SPH (CFD) step size is set separately, through TerrainParamsCRM::step_cfd.
    void SetStepsize(double step) { m_step_size = step; }

    /// Set visualization type for the wheel assembly (default: PRIMITIVES).
    void SetVisualizationType(VisualizationType vis) { m_vis_type = vis; }

    // Terrain setup

    /// Enable use of rigid terrain.
    void SetTerrainRigid(const TerrainPatchSize& size, const TerrainParamsRigid& params);

    /// Enable use of rigid terrain.
    /// Specify contact material properties and patch dimensions.
    void SetTerrainRigid(const TerrainPatchSize& size,  ///< terrain patch size
                         float mu,                      ///< coefficient of friction
                         float cr,                      ///< coefficient of restitution
                         float Y                        ///< contact material Young's modulus [Pa]
    );

    /// Enable use of SCM terrain.
    /// For a headless run, the visualization of the SCM terrain patch can be disabled (via TerrainParamsSCM::vis_mesh).
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
                       double grid_spacing = 0.125,   ///< SCM grid spacing
                       bool vis_mesh = true           ///< enable visualization mesh
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
                            double Young_modulus           ///< particle contact material Young's modulus [Pa], used with SMC contact
    );

#ifdef CHRONO_CRM

    struct CH_VEHICLE_API TerrainParamsCRM {
        TerrainParamsCRM();
        fsi::sph::ChFsiFluidSystemSPH::SPHParameters sph_params;  ///< SPH solver settings
        fsi::sph::ChFsiFluidSystemSPH::SoilProperties mat_props;  ///< soil properties
        double step_cfd;                                          ///< SPH (CFD) step size; if not positive, the multibody step size is used (default: 0)
        int sides;                                                ///< patch sides with boundary BCE markers, as BoxSide flags (default: all but the top)
        bool depth_based_pressure;                                ///< initialize particle pressures from their depth below the patch surface (default: false)
    };

    /// Enable use of CRM terrain.
    /// This version sets all SPH solver settings and CRM material properties from the given structure.
    void SetTerrainCRM(const TerrainPatchSize& size, const TerrainParamsCRM& params);

    /// Enable use of CRM terrain.
    /// This version sets only the SPH initial particle spacing (leaving all other SPH solver settings to their default values)
    /// and a simple CRM material with constant friction rheology. For real mu(I) or MCC rheologies, use the version of SetTerrainCRM
    /// that accepts a full set of CRM soil properties.
    void SetTerrainCRM(const TerrainPatchSize& size,  ///< terrain patch size
                       double spacing,                ///< SPH particle spacing
                       double density,                ///< material density
                       double Young_modulus,          ///< material Young's modulus
                       double friction,               ///< material internal friction
                       double cohesion                ///< material internal cohesion
    );

    /// Set a single active domain of specified size associated with the entire wheel assembly (CRM terrain only).
    /// This active AABB is associated with the hub body. If the wheel assembly has multiple bodies,
    /// it may be more efficient to set CRM active domains for each body individually.
    void SetWheelActiveDomain(const ChAABB& aabb);

    /// Set a single active domain of estimated dimensions associated with the entire wheel assembly (CRM terrain only).
    /// The default size is 2.5 times the wheel AABB.
    /// This active AABB is associated with the hub body. If the wheel assembly has multiple bodies, it may be more
    /// efficient to set CRM active domains for each body individually.
    void SetWheelActiveDomain();

    /// Get the current wheel active domain AABB.
    const ChAABB& GetWheelActiveDomain() const;

    /// Get the SPH domain for a CRM terrain.
    const ChAABB& GetTerrainSPHBoundingBox() const;

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
    /// In TEST mode, the wheel is lowered onto the terrain with a speed equal to `drop_speed`. For a wheel assembly with a
    /// suspension, the suspension travel is held during this drop phase at the static equilibrium configuration under the
    /// specified normal load (computed during initialization) and released once the wheel reaches the terrain.
    void Initialize(Mode mode, double drop_speed = 0.1);

    /// Advance the rig state by the specified time interval.
    /// The multibody system is advanced with the step size set through SetStepsize, using as many steps as needed to
    /// cover the given interval. With CRM terrain, the interval is the FSI coupling step: the multibody and SPH systems
    /// are each advanced over it with their own step sizes and exchange data at its end.
    void Advance(double step);

    /// Get suggested collision settings.
    /// These values are meaningful only when using granular terrain and Chrono::Multicore.
    void GetSuggestedCollisionSettings(double& collision_envelope,  ///< suggested envelope based on particle radius
                                       ChVector3i& collision_bins   ///< suggested number of bins for broad-phase collision detection
    ) const;

    /// Get the normal load.
    double GetNormalLoad() const { return m_normal_load; }

    /// Get total rig mass.
    double GetMass() const { return m_total_mass; }

    /// Get a handle to the underlying terrain subsystem.
    std::shared_ptr<ChTerrain> GetTerrain() const { return m_terrain; }

    /// Get current carrier body position.
    const ChVector3d& GetPos() const { return m_carrier_body->GetPos(); }

    /// Get current wheel position.
    ChVector3d GetWheelPos() const { return m_wheel_assembly->GetHub()->GetPos(); }

    /// Return true if the wheel has reached either end of the terrain patch.
    /// The patch end is reached when the wheel hub is within one wheel radius of the patch boundary, measured along the
    /// direction of travel. Always false for terrain types without a fixed patch (granular terrain uses a moving patch).
    bool PatchEndReached() const;

    /// Get the current terrain forces acting on the wheel as applied to the wheel hub.
    TerrainForce ReportWheelForce() const;

    /// Calculate the resultant terrain force and moment on the specified bodies, applied at the given point.
    /// The force and moment are expressed in the global frame. The interaction forces are obtained from the FSI system
    /// for CRM terrain, from the terrain for SCM terrain, and from the collision system otherwise.
    /// This utility can be used to implement WheelAssembly::ReportForces for a custom wheel assembly.
    static TerrainForce CalculateTerrainForce(const std::vector<std::shared_ptr<ChBody>>& bodies, ChTerrain& terrain, const ChVector3d& point);

    /// Return current drawbar-pull value.
    /// This is the reaction force in the linear motor used to enforce the specified rig longitudinal speed.
    double GetDBP() const;

    /// Get current wheel longitudinal slip.
    /// This value is calculated from the horizontal speed of the hub body and its angular rotation speed.
    double GetLongitudinalSlip() const;

    /// Get current wheel slip angle.
    /// This value is calculated from the current hub normal direction, relative to the direction of travel. For a wheel
    /// with a steerable suspension, the slip angle includes the static toe of the loaded configuration, i.e., it is the
    /// negative of the sum of the steering angle (see GetSteeringAngle) and the loaded toe angle.
    double GetSlipAngle() const;

    /// Get current wheel camber angle.
    /// This value is calculated from the current hub normal direction.
    double GetCamberAngle() const;

    /// Get current wheel steering angle, relative to the initial configuration (positive to the left).
    /// This value is calculated from the current hub normal direction, relative to the rig chassis. For a wheel with a
    /// suspension, the initial configuration is the loaded configuration with the steering actuator at zero; a zero
    /// steering angle therefore retains the static toe of the suspension in that configuration.
    double GetSteeringAngle() const;

    /// Get current steering actuator coordinate (0 if the wheel assembly has no steering actuator).
    double GetSteeringActuatorCoordinate() const;

    /// Get the current values of the rig output channels.
    /// The channels include the rig measurements (zero until measurements are enabled, see OutputEnabled), the rig and
    /// wheel kinematics, and any channels provided by the wheel assembly (see WheelAssembly::AddOutputChannels). All
    /// quantities are in SI units, with angles in radians. The set of channels is the same at every call.
    std::vector<ChOutput::Channel> GetOutputChannels() const;

    /// Get the kinematic steering map (steering angle as a function of steering actuator coordinate).
    /// This map is computed during initialization, only if a steering angle function was specified.
    const ChFunctionInterp& GetSteeringMap() const { return m_steer_map; }

    /// Get the linear motor used to actuate the carrier.
    std::shared_ptr<ChLinkMotorLinearSpeed> GetMotorCarrier() const { return m_lin_motor; }

    /// Get the rotation motor used to actuate the wheel.
    std::shared_ptr<ChShaftsMotorSpeed> GetMotorWheel() const { return m_rot_motor; }

  private:
    /// Create the single-wheel test rig mechanism.
    void CreateMechanism();
    void CreateWheelSuspensionMechanism(double dim);
    void CreateWheelMechanism(double dim);

    /// Compute the kinematic steering map through a sweep of position-level assemblies.
    void ComputeSteeringMap();

    /// Find and set the suspension travel at static equilibrium under the normal load.
    void ComputeLoadedTravel();

    /// Set the terrain offset and height based on the current wheel position.
    void SetTerrainLocation();

    /// Calculate the current hub yaw angle relative to the rig chassis.
    double CalcHubYaw() const;

    /// Create the terrain patch of specified type.
    void CreateTerrain();
    double GetPatchCenterX() const;

    void CreateTerrainSCM();
    void CreateTerrainRigid();
    void CreateTerrainGranular();
#ifdef CHRONO_CRM
    void CreateTerrainCRM();
#endif

    /// Add the specified physics item to the containing system and record it for removal on rig destruction.
    template <typename T>
    std::shared_ptr<T> AddItem(std::shared_ptr<T> item) {
        m_system.Add(item);
        m_items.push_back(item);
        return item;
    }

    ChSystem& m_system;  ///< containing Chrono system

    std::vector<std::shared_ptr<ChPhysicsItem>> m_items;  ///< all physics items created by the rig

    Mode m_mode;    ///< testing mode
    bool m_output;  ///< if false, report default measurements (typically 0)

    std::shared_ptr<ChTerrain> m_terrain;             ///< handle to underlying terrain subsystem
    std::shared_ptr<WheelAssembly> m_wheel_assembly;  ///< wheel assembly
    VisualizationType m_vis_type;                     ///< visualization type for wheel assembly
    double m_step_size;                               ///< step size for wheel assembly integration

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

    double m_camber_angle;  ///< wheel camber angle (used only if no suspension)

    ChAABB m_wheel_AABB;  ///< AABB for the entire wheel assembly (estimated or user-provided)

    std::shared_ptr<ChBody> m_ground_body;         ///< ground body
    std::shared_ptr<ChBodyAuxRef> m_carrier_body;  ///< rig carrier body
    std::shared_ptr<ChBodyAuxRef> m_chassis_body;  ///< "chassis" body which carries normal load
    std::shared_ptr<ChBody> m_slip_body;           ///< intermediate body for controlling slip angle

    bool m_ls_actuated;  ///< is linear speed actuated?
    bool m_rs_actuated;  ///< is angular speed actuated?

    bool m_long_slip_constant;  ///< true if constant longitudinal slip was specified
    double m_long_slip;         ///< user-specified longitudinal slip
    double m_base_speed;        ///< base speed for long slip calculation

    std::shared_ptr<ChFunction> m_ls_fun;  ///< longitudinal speed function of time
    std::shared_ptr<ChFunction> m_rs_fun;  ///< angular speed function of time
    std::shared_ptr<ChFunction> m_sa_fun;  ///< slip angle function of time

    enum class SteeringInput { NONE, ANGLE, ACTUATOR };
    SteeringInput m_steer_input;              ///< type of steering input
    std::shared_ptr<ChFunction> m_steer_fun;  ///< steering angle or rack displacement function of time
    bool m_steer_feedback;                    ///< use feedback correction of rack displacement?
    double m_steer_tau;                       ///< time constant for steering feedback correction
    double m_steer_yaw0;                      ///< hub yaw angle in the initial configuration
    double m_steer_offset;                    ///< actuator coordinate correction (steering feedback)
    ChFunctionInterp m_steer_map;             ///< steering angle as function of actuator coordinate
    ChFunctionInterp m_steer_map_inv;         ///< actuator coordinate as function of steering angle

    std::shared_ptr<ChLinkMotorLinearSpeed> m_drop_motor;  ///< actuator for controlled wheel drop
    std::shared_ptr<ChLinkMotorLinearSpeed> m_lin_motor;   ///< carrier actuator
    std::shared_ptr<ChShaftsMotorSpeed> m_rot_motor;       ///< wheel actuator
    std::shared_ptr<ChLinkLockLock> m_slip_lock;           ///< slip angle actuator
    std::shared_ptr<ChLinkMotorLinearPosition> m_travel_motor;  ///< suspension travel actuator during the drop phase
    std::shared_ptr<SteeringActuator> m_steer_actuator;         ///< steering actuator of the wheel assembly
    double m_drop_contact_time;                                 ///< time at which the wheel reached the terrain (drop phase)
    double m_hub_offset0;                                       ///< hub height relative to chassis in design configuration
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
