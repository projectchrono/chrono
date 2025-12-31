// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// RoboSimian model classes.
//
// For a description of this robot, see:
//  Satzinger B.W., Lau C., Byl M., Byl K. (2016)
//  Experimental Results for Dexterous Quadruped Locomotion Planning with RoboSimian.
//  In: Hsieh M., Khatib O., Kumar V. (eds) Experimental Robotics.
//  Springer Tracts in Advanced Robotics, vol 109. Springer, Cham.
//  https://doi.org/10.1007/978-3-319-23778-7_3
//
// =============================================================================

#ifndef ROBOSIMIAN_H
#define ROBOSIMIAN_H

#include <array>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {

/// Namespace with classes for the RoboSimian model.
namespace robosimian {

/// @addtogroup robot_models_robosimian
/// @{

// -----------------------------------------------------------------------------
// Various definitions
// -----------------------------------------------------------------------------

/// RoboSimian limb identifiers.
enum LimbID {
    FR = 0,  ///< front right
    RR = 1,  ///< rear right
    RL = 2,  ///< rear left
    FL = 3   ///< front left
};

/// RoboSimian collision families.
namespace CollisionFamily {
enum Enum {
    LIMB_FR = 1,  ///< front-right limb
    LIMB_RR = 2,  ///< rear-right limb
    LIMB_RL = 3,  ///< rear-left limb
    LIMB_FL = 4,  ///< front-left limb
    CHASSIS = 5,  ///< chassis (torso)
    SLED = 6,     ///< sled
    WHEEL_DD = 7  ///< direct-drive wheels
};
}

/// RoboSimian collision flags (specify which part carries collision shapes).
namespace CollisionFlags {
enum Enum {
    NONE = 0,          ///< no collision shapes on any body
    CHASSIS = 1 << 0,  ///< chassis (torso)
    SLED = 1 << 1,     ///< sled
    LIMBS = 1 << 2,    ///< all limb bodies (excluding final wheels)
    WHEELS = 1 << 3,   ///< all wheels
    ALL = 0xFFFF       ///< collision enabled on all bodies
};
}

/// RoboSimian actuation modes.
enum class ActuationMode {
    ANGLE,  ///< prescribe time-series for joint angle
    SPEED   ///< prescribe time-series for joint angular speed
};

/// RoboSimian locomotion modes.
enum class LocomotionMode {
    WALK,      ///< walking
    SCULL,     ///< crawl/slide on the sled
    INCHWORM,  ///< inchworm-type movement
    DRIVE      ///< driving
};

// -----------------------------------------------------------------------------
// Definition of a part (body + collision shapes + visualization assets)
// -----------------------------------------------------------------------------

struct CH_MODELS_API BoxShape {
    BoxShape(const chrono::ChVector3d& pos, const chrono::ChQuaternion<>& rot, const chrono::ChVector3d& dims)
        : m_pos(pos), m_rot(rot), m_dims(dims) {}
    chrono::ChVector3d m_pos;
    chrono::ChQuaternion<> m_rot;
    chrono::ChVector3d m_dims;
};

struct CH_MODELS_API SphereShape {
    SphereShape(const chrono::ChVector3d& pos, double radius) : m_pos(pos), m_radius(radius) {}
    chrono::ChVector3d m_pos;
    double m_radius;
};

struct CH_MODELS_API CylinderShape {
    CylinderShape(const chrono::ChVector3d& pos, const chrono::ChQuaternion<>& rot, double radius, double length)
        : m_pos(pos), m_rot(rot), m_radius(radius), m_length(length) {}
    chrono::ChVector3d m_pos;
    chrono::ChQuaternion<> m_rot;
    double m_radius;
    double m_length;
};

struct CH_MODELS_API MeshShape {
    enum class Type { CONVEX_HULL, TRIANGLE_SOUP, NODE_CLOUD };
    MeshShape(const chrono::ChVector3d& pos, const chrono::ChQuaternion<>& rot, const std::string& name, Type type)
        : m_pos(pos), m_rot(rot), m_name(name), m_type(type) {}
    chrono::ChVector3d m_pos;
    chrono::ChQuaternion<> m_rot;
    std::string m_name;
    Type m_type;
};

/// RoboSimian part.
/// A robot part encapsulates a Chrono body with its collision and visualization shapes.
class CH_MODELS_API RS_Part {
  public:
    RS_Part(const std::string& name, std::shared_ptr<ChContactMaterial> mat, chrono::ChSystem* system);
    virtual ~RS_Part() {}

    const std::string& GetName() const { return m_name; }
    void SetName(const std::string& name) { m_name = name; }
    void SetVisualizationType(VisualizationType vis);

    std::shared_ptr<chrono::ChBodyAuxRef> GetBody() const { return m_body; }
    const chrono::ChVector3d& GetPos() const { return m_body->GetFrameRefToAbs().GetPos(); }
    const chrono::ChQuaternion<>& GetRot() const { return m_body->GetFrameRefToAbs().GetRot(); }

  protected:
    void AddVisualizationAssets(VisualizationType vis);
    void AddCollisionShapes();

    std::string m_name;                            ///< subsystem name
    std::shared_ptr<chrono::ChBodyAuxRef> m_body;  ///< rigid body
    std::shared_ptr<ChContactMaterial> m_mat;      ///< contact material (shared among all shapes)
    std::vector<BoxShape> m_boxes;                 ///< set of collision boxes
    std::vector<SphereShape> m_spheres;            ///< set of collision spheres
    std::vector<CylinderShape> m_cylinders;        ///< set of collision cylinders
    std::vector<MeshShape> m_meshes;               ///< set of collision meshes
    std::string m_mesh_name;                       ///< visualization mesh name
    chrono::ChVector3d m_offset;                   ///< offset for visualization mesh
    chrono::ChColor m_color;                       ///< visualization asset color

    friend class RoboSimian;
    friend class RS_Limb;
};

// -----------------------------------------------------------------------------
// Robot chassis (torso)
// -----------------------------------------------------------------------------

/// RoboSimian chassis (torso).
class CH_MODELS_API RS_Chassis : public RS_Part {
  public:
    RS_Chassis(const std::string& name,
               bool is_fixed,
               std::shared_ptr<ChContactMaterial> mat,
               chrono::ChSystem* system);
    ~RS_Chassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(const chrono::ChCoordsys<>& pos);

    /// Enable/disable collision for the sled (Default: false).
    void EnableCollision(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const chrono::ChVector3d& shift);

    bool m_collide;  ///< true if collision enabled

    friend class RoboSimian;
};

// -----------------------------------------------------------------------------
// Robot sled (fixed to chassis)
// -----------------------------------------------------------------------------

/// RoboSimian sled (attached to chassis).
class CH_MODELS_API RS_Sled : public RS_Part {
  public:
    RS_Sled(const std::string& name, std::shared_ptr<ChContactMaterial> mat, chrono::ChSystem* system);
    ~RS_Sled() {}

    /// Initialize the sled at the specified position (relative to the chassis).
    void Initialize(std::shared_ptr<chrono::ChBodyAuxRef> chassis,  ///< chassis body
                    const chrono::ChVector3d& xyz,                  ///< location (relative to chassis)
                    const chrono::ChVector3d& rpy                   ///< roll-pitch-yaw (relative to chassis)
    );

    /// Enable/disable collision for the sled (default: true).
    void EnableCollision(bool state);

  private:
    /// Translate the sled by the specified value.
    void Translate(const chrono::ChVector3d& shift);

    bool m_collide;  ///< true if collision enabled

    friend class RoboSimian;
};

// -----------------------------------------------------------------------------
// Direct-drive robot wheels (not used in current model)
// -----------------------------------------------------------------------------

/// RoboSimian direct-drive wheel.
/// Note that this part is not used in the current model.
class CH_MODELS_API RS_WheelDD : public RS_Part {
  public:
    RS_WheelDD(const std::string& name, int id, std::shared_ptr<ChContactMaterial> mat, chrono::ChSystem* system);
    ~RS_WheelDD() {}

    /// Initialize the direct-drive wheel at the specified position (relative to the chassis).
    void Initialize(std::shared_ptr<chrono::ChBodyAuxRef> chassis,  ///< chassis body
                    const chrono::ChVector3d& xyz,                  ///< location (relative to chassis)
                    const chrono::ChVector3d& rpy                   ///< roll-pitch-yaw (relative to chassis)
    );
};

// -----------------------------------------------------------------------------
// Robot limb components
// -----------------------------------------------------------------------------

/// RoboSimian link.
/// A robot link encapsulates information for a body in a robot limb.
class CH_MODELS_API Link {
  public:
    Link(const std::string& mesh_name,             ///< name of associated mesh
         const chrono::ChVector3d& offset,         ///< mesh offset
         const chrono::ChColor& color,             ///< mesh color
         double mass,                              ///< link mass
         const chrono::ChVector3d& com,            ///< location of COM
         const chrono::ChVector3d& inertia_xx,     ///< moments of inertia
         const chrono::ChVector3d& inertia_xy,     ///< products of inertia
         const std::vector<CylinderShape>& shapes  ///< list of collision shapes
         )
        : m_mesh_name(mesh_name),
          m_offset(offset),
          m_color(color),
          m_mass(mass),
          m_com(com),
          m_inertia_xx(inertia_xx),
          m_inertia_xy(inertia_xy),
          m_shapes(shapes) {}

  private:
    std::string m_mesh_name;
    chrono::ChVector3d m_offset;
    chrono::ChColor m_color;
    double m_mass;
    chrono::ChVector3d m_com;
    chrono::ChVector3d m_inertia_xx;
    chrono::ChVector3d m_inertia_xy;
    std::vector<CylinderShape> m_shapes;

    friend class RS_Limb;
};

struct CH_MODELS_API LinkData {
    LinkData(std::string myname, Link mylink, bool inc) : name(myname), link(mylink), include(inc) {}
    ////LinkData(LinkData&&) {}

    std::string name;
    Link link;
    bool include;
};

struct CH_MODELS_API JointData {
    std::string name;
    std::string linkA;
    std::string linkB;
    bool is_fixed;
    chrono::ChVector3d xyz;
    chrono::ChVector3d rpy;
    chrono::ChVector3d axis;
};

/// RoboSimian limb.
/// A robot limb represents a multibody chain composed of robot links and joints.
class CH_MODELS_API RS_Limb {
  public:
    RS_Limb(const std::string& name,                       ///< limb name
            LimbID id,                                     ///< limb ID
            const LinkData data[],                         ///< data for limb links
            std::shared_ptr<ChContactMaterial> wheel_mat,  ///< contact material for the limb wheel
            std::shared_ptr<ChContactMaterial> link_mat,   ///< contact material for the limb links
            chrono::ChSystem* system                       ///< containing system
    );
    ~RS_Limb() {}

    /// Initialize the limb at the specified position (relative to the chassis).
    void Initialize(std::shared_ptr<chrono::ChBodyAuxRef> chassis,  ///< chassis body
                    const chrono::ChVector3d& xyz,                  ///< location (relative to chassis)
                    const chrono::ChVector3d& rpy,                  ///< roll-pitch-yaw (relative to chassis)
                    CollisionFamily::Enum collision_family,         ///< collision family
                    ActuationMode wheel_mode                        ///< motor type for wheel actuation
    );

    /// Set visualization type for all limb links.
    void SetVisualizationType(VisualizationType vis);

    /// Enable/disable collision on all links, except final wheel (default: false).
    void SetCollideLinks(bool state);

    /// Enable/disable collision for final wheel (default: true).
    void SetCollideWheel(bool state);

    /// Get the total mass of this limb.
    double GetMass() const;

    /// Get a handle to the wheel body.
    std::shared_ptr<chrono::ChBodyAuxRef> GetWheelBody() const { return m_wheel->GetBody(); }

    /// Get location of the wheel body.
    const chrono::ChVector3d& GetWheelPos() const { return m_wheel->GetPos(); }

    /// Get angular velocity of the wheel body (expressed in local coordinates).
    chrono::ChVector3d GetWheelAngVelocity() const { return m_wheel->GetBody()->GetAngVelLocal(); }

    /// Get wheel angle.
    double GetWheelAngle() const { return m_wheel_motor->GetMotorAngle(); }

    /// Get wheel angular speed.
    double GetWheelOmega() const { return m_wheel_motor->GetMotorAngleDt(); }

    /// Get angle for specified motor.
    /// Motors are named "joint1", "joint2", ... , "joint8", starting at the chassis.
    double GetMotorAngle(const std::string& motor_name) const;

    /// Get angular speed for specified motor.
    /// Motors are named "joint1", "joint2", ... , "joint8", starting at the chassis.
    double GetMotorOmega(const std::string& motor_name) const;

    /// Get actuator reaction torque [Nm] for specified motor.
    /// Motors are named "joint1", "joint2", ... , "joint8", starting at the chassis.
    double GetMotorTorque(const std::string& motor_name) const;

    /// Get angles for all 8 limb motors.
    std::array<double, 8> GetMotorAngles();

    /// Get angular velocities for all 8 limb motors.
    std::array<double, 8> GetMotorOmegas();

    /// Get actuator torques for all 8 limb motors.
    std::array<double, 8> GetMotorTorques();

    /// Get current motor actuations.
    void GetMotorActuations(std::array<double, 8>& angles, std::array<double, 8>& speeds);

    /// Set activation for given motor at current time.
    void Activate(const std::string& motor_name, double time, double val);

    /// Set activations for all motors at current time.
    void Activate(double time, const std::array<double, 8>& vals);

  private:
    /// Translate the limb bodies by the specified value.
    void Translate(const chrono::ChVector3d& shift);

    std::string m_name;
    std::unordered_map<std::string, std::shared_ptr<RS_Part>> m_links;
    std::unordered_map<std::string, std::shared_ptr<chrono::ChLink>> m_joints;
    std::unordered_map<std::string, std::shared_ptr<chrono::ChLinkMotorRotation>> m_motors;
    std::shared_ptr<RS_Part> m_wheel;
    std::shared_ptr<chrono::ChLinkMotorRotation> m_wheel_motor;

    bool m_collide_links;  ///< collide flag for all links (except final wheel)
    bool m_collide_wheel;  ///< collide flag for the final wheel

    friend class RoboSimian;
};

// -----------------------------------------------------------------------------
// Definition of the RoboSimian robot
// -----------------------------------------------------------------------------

class ContactManager;
class ContactMaterial;
class RS_Driver;

/// RoboSimian robot model.
/// The robot model consists of a chassis (torso) with an attached sled and four limbs (legs).
class CH_MODELS_API RoboSimian {
  public:
    /// Construct a RoboSimian with an implicit Chrono system.
    RoboSimian(chrono::ChContactMethod contact_method,  ///< contact formulation (SMC or NSC)
               bool has_sled = false,                   ///< true if robot has sled body attached to chassis
               bool is_fixed = false                    ///< true if robot chassis fixed to ground
    );

    /// Construct a RoboSimian within the specified Chrono system.
    RoboSimian(chrono::ChSystem* system,  ///< containing system
               bool has_sled = false,     ///< true if robot has sled body attached to chassis
               bool is_fixed = false      ///< true if robot chassis fixed to ground
    );

    ~RoboSimian();

    /// Get the containing system.
    chrono::ChSystem* GetSystem() { return m_system; }

    /// Set actuation type for wheel motors (default: SPEED).
    void SetMotorActuationMode(ActuationMode mode) { m_wheel_mode = mode; }

    /// Set collision flags for the various subsystems.
    /// By default, collision is enabled for the sled and wheels only.
    /// The 'flags' argument can be any of the CollisionFlag enums, or a combination thereof (using bit-wise operators).
    void EnableCollision(int flags);

    /// Set coefficients of friction for sled-terrain and wheel-terrain contacts.
    /// Default values: 0.8.
    void SetFrictionCoefficients(float sled_friction, float wheel_friction);

    /// Attach a driver system.
    void SetDriver(std::shared_ptr<RS_Driver> driver);

    /// Set visualization type for chassis subsystem.
    void SetVisualizationTypeChassis(VisualizationType vis);
    /// Set visualization type for sled subsystem.
    void SetVisualizationTypeSled(VisualizationType vis);
    /// Set visualization type for all limb subsystems.
    void SetVisualizationTypeLimbs(VisualizationType vis);
    /// Set visualization type for thr specified limb subsystem.
    void SetVisualizationTypeLimb(LimbID id, VisualizationType vis);
    /// Set visualization type for all wheel subsystem.
    void SetVisualizationTypeWheels(VisualizationType vis);

    /// Set output directory.
    void SetOutputDirectory(const std::string& outdir, const std::string& root = "results");

    /// Get the total mass of the robot.
    double GetMass() const;

    /// Get a handle to the robot's chassis subsystem.
    std::shared_ptr<RS_Chassis> GetChassis() const { return m_chassis; }

    /// Get a handle to the robot's chassis body.
    std::shared_ptr<chrono::ChBodyAuxRef> GetChassisBody() const { return m_chassis->GetBody(); }

    /// Get location of the chassis body.
    const chrono::ChVector3d& GetChassisPos() const { return m_chassis->GetPos(); }

    /// Get orientation of the chassis body.
    const chrono::ChQuaternion<>& GetChassisRot() const { return m_chassis->GetRot(); }

    /// Get a handle to the robot's sled subsystem.
    std::shared_ptr<RS_Sled> GetSled() const { return m_sled; }

    /// Get a handle to the robot's sled body.
    std::shared_ptr<chrono::ChBodyAuxRef> GetSledBody() const { return m_sled->GetBody(); }

    /// Get a handle to the robot's specified limb subsystem.
    std::shared_ptr<RS_Limb> GetLimb(LimbID id) const { return m_limbs[id]; }

    /// Get a handle to the wheel body for the specified limb.
    std::shared_ptr<chrono::ChBodyAuxRef> GetWheelBody(LimbID id) const { return m_limbs[id]->GetWheelBody(); }

    /// Get location of the wheel body for the specified limb.
    const chrono::ChVector3d& GetWheelPos(LimbID id) const { return m_limbs[id]->GetWheelPos(); }

    /// Get angular velocity of the wheel body for the specified limb (expressed in local coordinates).
    chrono::ChVector3d GetWheelAngVelocity(LimbID id) const { return m_limbs[id]->GetWheelAngVelocity(); }

    /// Get wheel angle for the specified limb.
    double GetWheelAngle(LimbID id) const { return m_limbs[id]->GetWheelAngle(); }

    /// Get wheel angular speed for the specified limb.
    double GetWheelOmega(LimbID id) const { return m_limbs[id]->GetWheelOmega(); }

    /// Get angles for all 8 limb motors.
    std::array<double, 8> GetMotorAngles(LimbID id) { return m_limbs[id]->GetMotorAngles(); }

    /// Get angular velocities for all 8 limb motors.
    std::array<double, 8> GetMotorOmegas(LimbID id) { return m_limbs[id]->GetMotorOmegas(); }

    /// Get actuator torques for all 8 limb motors.
    std::array<double, 8> GetMotorTorques(LimbID id) { return m_limbs[id]->GetMotorTorques(); }

    /// Access the chassis contact material.
    std::shared_ptr<ChContactMaterial> GetChassisContactMaterial() { return m_chassis_material; }

    /// Access the sled contact material.
    std::shared_ptr<ChContactMaterial> GetSledContactMaterial() { return m_sled_material; }

    /// Access the wheel contact material. Note that this material is shared by all wheels.
    std::shared_ptr<ChContactMaterial> GetWheelContactMaterial() { return m_wheel_material; }

    /// Access the wheelDD contact material. Note that this material is shared by all DD wheels.
    std::shared_ptr<ChContactMaterial> GetWheelDDContactMaterial() { return m_wheelDD_material; }

    /// Access the link contact material. Note that this material is shared by all non-wheel links.
    std::shared_ptr<ChContactMaterial> GetLinkContactMaterial() { return m_link_material; }

    /// Initialize the robot at the specified chassis position and orientation.
    void Initialize(const chrono::ChCoordsys<>& pos);

    /// Directly activate the specified motor on the specified limb.
    void Activate(LimbID id, const std::string& motor_name, double time, double val);

    /// Advance dynamics of underlying system.
    /// If a driver system is specified, apply motor actuations at current time.
    void DoStepDynamics(double step);

    /// Translate all robot bodies by the specified value.
    void Translate(const chrono::ChVector3d& shift);

    /// Output current data.
    void Output();

    /// Report current contacts for all robot parts.
    void ReportContacts();

  private:
    void Create(bool has_sled, bool is_fixed);

    chrono::ChSystem* m_system;  ///< pointer to the Chrono system
    bool m_owns_system;          ///< true if system created at construction

    std::shared_ptr<RS_Chassis> m_chassis;          ///< robot chassis
    std::shared_ptr<RS_Sled> m_sled;                ///< optional sled attached to chassis
    std::vector<std::shared_ptr<RS_Limb>> m_limbs;  ///< robot limbs
    ////std::shared_ptr<RS_WheelDD> m_wheel_left;       ///< left DD wheel
    ////std::shared_ptr<RS_WheelDD> m_wheel_right;      ///< right DD wheel

    ActuationMode m_wheel_mode;  ///< type of actuation for wheel motor

    std::shared_ptr<ChContactMaterial> m_chassis_material;  ///< chassis contact material
    std::shared_ptr<ChContactMaterial> m_sled_material;     ///< sled contact material
    std::shared_ptr<ChContactMaterial> m_wheel_material;    ///< wheel contact material (shared across limbs)
    std::shared_ptr<ChContactMaterial> m_link_material;     ///< link contact material (shared across limbs)
    std::shared_ptr<ChContactMaterial> m_wheelDD_material;  ///< wheelDD contact material (shared across limbs)

    float m_wheel_friction;  ///< coefficient of friction wheel-terrain (used in material_override)
    float m_sled_friction;   ///< coefficient of friction sled-terrain (used in material_override)

    std::shared_ptr<RS_Driver> m_driver;   ///< robot driver system
    ContactManager* m_contact_reporter;    ///< contact reporting callback class
    ContactMaterial* m_material_override;  ///< contact material override callback class

    std::string m_outdir;     ///< path of output directory
    std::string m_root;       ///< prefix of output filenames
    std::ofstream m_outf[4];  ///< output file streams (one per limb)

    friend class ContactMaterial;
};

// -----------------------------------------------------------------------------
// RoboSimian driver classes
// -----------------------------------------------------------------------------

typedef std::array<std::array<double, 8>, 4> Actuation;

/// Driver for the RoboSimian robot.
class CH_MODELS_API RS_Driver {
  public:
    /// Driving phases.
    enum Phase { POSE, HOLD, START, CYCLE, STOP };

    RS_Driver(const std::string& filename_start,  ///< name of file with joint actuations for start phase
              const std::string& filename_cycle,  ///< name of file with joint actuations for cycle phase
              const std::string& filename_stop,   ///< name of file with joint actuations for stop phase
              bool repeat = false                 ///< true if cycle phase is looped
    );

    ~RS_Driver();

    /// Specify time intervals to assume and then hold the initial pose.
    void SetTimeOffsets(double time_pose, double time_hold);

    /// Return the current limb motor actuations.
    Actuation GetActuation() { return m_actuations; }

    /// Directly feed actuations to the motors.
    void SetActuation(Actuation ext_act) { m_actuations = ext_act; }

    /// Set the driving mode to accept external inputs.
    void SetDrivingMode(bool drivemode) { driven = drivemode; }

    /// Set the motor type (setpoint/torque).
    void UseTorqueMotors(bool use_tm) { torque_actuated = use_tm; }

    /// Return the current phase
    std::string GetCurrentPhase() const { return m_phase_names[m_phase]; }

    /// Class to be used as callback interface for user-defined actions at phase changes.
    class CH_MODELS_API PhaseChangeCallback {
      public:
        virtual ~PhaseChangeCallback() {}
        /// Function called on each phase change.
        virtual void OnPhaseChange(RS_Driver::Phase old_phase, RS_Driver::Phase new_phase) = 0;
    };

    /// Register a phase-change callback object.
    void RegisterPhaseChangeCallback(PhaseChangeCallback* callback) { m_callback = callback; }

  private:
    void Update(double time);
    void LoadDataLine(double& time, Actuation& activations);

    std::ifstream m_ifs_start;        ///< input file stream for start phase
    std::ifstream m_ifs_cycle;        ///< input file stream for cycle phase
    std::ifstream m_ifs_stop;         ///< input file stream for stop phase
    std::ifstream* m_ifs;             ///< active input file stream
    double m_time_pose;               ///< time interval to assume initial pose
    double m_time_hold;               ///< time interval to hold initial pose
    double m_offset;                  ///< current time offset in input files
    bool m_repeat;                    ///< repeat cycle
    Phase m_phase;                    ///< current phase
    double m_time_1;                  ///< time for cached actuations
    double m_time_2;                  ///< time for cached actuations
    Actuation m_actuations_1;         ///< cached actuations (before)
    Actuation m_actuations_2;         ///< cached actuations (after)
    Actuation m_actuations;           ///< current actuations
    PhaseChangeCallback* m_callback;  ///< user callback for phase change
    bool driven = false;           ///< true if the driver is expecting external inputs instead of reading from a file
    bool torque_actuated = false;  ///< true if using torque actuation instead of setpoints

    static const std::string m_phase_names[5];  ///< names of various driver phases

    friend class RoboSimian;
};

/// Robot driver callback to keep track of average speed and distance between phase changes.
class CH_MODELS_API RS_DriverCallback : public RS_Driver::PhaseChangeCallback {
  public:
    RS_DriverCallback(RoboSimian* robot) : m_start_x(0), m_start_time(0), m_robot(robot) {}
    virtual void OnPhaseChange(RS_Driver::Phase old_phase, RS_Driver::Phase new_phase) override;

    /// Get distance traveled from last phase change.
    double GetDistance() const { return m_robot->GetChassisPos().x() - m_start_x; }
    /// Get time elapsed since last phase change.
    double GetDuration() const { return m_robot->GetSystem()->GetChTime() - m_start_time; }
    /// Get average robot speed since last phase change.
    double GetAvgSpeed() const { return GetDistance() / GetDuration(); }

    double m_start_x;     ///< location at start of current phase
    double m_start_time;  ///< time at start of current phase

  private:
    RoboSimian* m_robot;
};

/// @} robot_models_robosimian

}  // namespace robosimian
}  // namespace chrono
#endif
