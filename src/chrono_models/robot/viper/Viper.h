// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou, Radu Serban
// =============================================================================
//
// NASA VIPER Lunar Rover Model Class.
// This class contains model for NASA's VIPER lunar rover for NASA's 2024 Moon
// exploration mission.
//
// =============================================================================

#ifndef VIPER_H
#define VIPER_H

#include <string>
#include <array>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {

/// Namespace with classes for the Viper model.
namespace viper {

/// @addtogroup robot_models_viper
/// @{

/// Viper wheel/suspension identifiers.
enum ViperWheelID {
    V_LF = 0,  ///< left front
    V_RF = 1,  ///< right front
    V_LB = 2,  ///< left back
    V_RB = 3   ///< right back
};

/// Viper wheel type.
enum class ViperWheelType {
    RealWheel,    ///< actual geometry of the Viper wheel
    SimpleWheel,  ///< simplified wheel geometry
    CylWheel      ///< cylindrical wheel geometry
};

// -----------------------------------------------------------------------------

/// Base class definition for all Viper parts.
/// Viper Rover Parts include Chassis, Steering, Upper Suspension Arm, Bottom Suspension Arm and Wheel.
class CH_MODELS_API ViperPart {
  public:
    ViperPart(const std::string& name,                 ///< part name
              const ChFrame<>& rel_pos,                ///< position relative to chassis frame
              std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
              bool collide                             ///< enable collision?
    );
    virtual ~ViperPart() {}

    /// Return the name of the part.
    const std::string& GetName() const { return m_name; }

    /// Set the name of the part.
    void SetName(const std::string& name) { m_name = name; }

    /// Enable/disable visualization.
    void SetVisualize(bool state) { m_visualize = state; }

    /// Enable/disable collision.
    void SetCollide(bool state) { m_collide = state; }

    /// Initialize the rover part by attaching it to the specified chassis body.
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis);

    /// Return the ChBody of the corresponding Viper part.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Return the position of the Viper part.
    /// This is the absolute location of the part reference frame.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Return the rotation of the Viper part.
    /// This is the orientation wrt the global frame of the part reference frame.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

    /// Return the linear velocity of the Viper part.
    /// This is the absolute linear velocity of the part reference frame.
    const ChVector<>& GetLinVel() const { return m_body->GetFrame_REF_to_abs().GetPos_dt(); }

    /// Return the angular velocity of the Viper part.
    /// This is the absolute angular velocity of the part reference frame.
    const ChVector<> GetAngVel() const { return m_body->GetFrame_REF_to_abs().GetWvel_par(); }

    /// Return the linear acceleration of the Viper part.
    /// This is the absolute linear acceleration of the part reference frame.
    const ChVector<>& GetLinAcc() const { return m_body->GetFrame_REF_to_abs().GetPos_dtdt(); }

    /// Return the angular acceleratino of the Viper part.
    /// This is the absolute angular acceleratin of the part reference frame.
    const ChVector<> GetAngAcc() const { return m_body->GetFrame_REF_to_abs().GetWacc_par(); }

  protected:
    /// Utility function for calculating mass properties using the part's collision mesh.
    void CalcMassProperties(double density);

    /// Construct the part body.
    void Construct(ChSystem* system);

    std::string m_name;                        ///< part name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< part rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material

    std::string m_mesh_name;  ///< visualization mesh name
    ChFrame<> m_mesh_xform;   ///< mesh transform (translate, rotate, scale)
    ChColor m_color;          ///< visualization asset color

    ChFrame<> m_pos;       ///< relative position wrt the chassis
    double m_mass;         ///< mass
    ChVector<> m_inertia;  ///< principal moments of inertia
    ChFrame<> m_cog;       ///< COG frame (relative to body frame)

    bool m_visualize;  ///< part visualization flag
    bool m_collide;    ///< part collision flag
};

/// Viper rover Chassis.
class CH_MODELS_API ViperChassis : public ViperPart {
  public:
    ViperChassis(const std::string& name,                ///< part name
                 std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~ViperChassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(ChSystem* system, const ChFrame<>& pos);
};

/// Viper rover Wheel.
class CH_MODELS_API ViperWheel : public ViperPart {
  public:
    ViperWheel(const std::string& name,                 ///< part name
               const ChFrame<>& rel_pos,                ///< position relative to chassis frame
               std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
               ViperWheelType wheel_type                ///< wheel type
    );
    ~ViperWheel() {}

    friend class Viper;
};

/// The upper arm of the Viper rover suspension.
class CH_MODELS_API ViperUpperArm : public ViperPart {
  public:
    ViperUpperArm(const std::string& name,                 ///< part name
                  const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                  std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                  const int& side                          ///< vehicle side 0: L, 1: R
    );
    ~ViperUpperArm() {}
};

/// The bottom arm of the Viper rover suspension.
class CH_MODELS_API ViperLowerArm : public ViperPart {
  public:
    ViperLowerArm(const std::string& name,                 ///< part name
                  const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                  std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                  const int& side                          ///< vehicle side 0: L, 1: R
    );
    ~ViperLowerArm() {}
};

/// Steering rod of the Viper rover.
/// The steering rod is connected to the steering cyl, this link is controlled steering.
/// There are two connecting rods on the steering rod, linking to upper and bottom arms of the suspension.
class CH_MODELS_API ViperUpright : public ViperPart {
  public:
    ViperUpright(const std::string& name,                 ///< part name
                 const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                 std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                 const int& side                          ///< vehicle side 0: L, 1: R
    );
    ~ViperUpright() {}
};

// -----------------------------------------------------------------------------

class ViperDriver;

/// Viper rover class.
/// This class encapsulates the location and rotation information of all Viper parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API Viper {
  public:
    Viper(ChSystem* system, ViperWheelType wheel_type = ViperWheelType::RealWheel);

    ~Viper() {}

    /// Get the containing system.
    ChSystem* GetSystem() const { return m_system; }

    /// Attach a driver system.
    void SetDriver(std::shared_ptr<ViperDriver> driver);

    /// Set wheel contact material.
    void SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat);

    /// Fix the chassis to ground.
    void SetChassisFixed(bool fixed);

    /// Enable/disable visualization of the rover chassis (default: true).
    void SetChassisVisualization(bool state);

    /// Enable/disable visualization of rover wheels (default: true).
    void SetWheelVisualization(bool state);

    /// Enable/disable visualization of rover suspensions (default: true).
    void SetSuspensionVisualization(bool state);

    /// Initialize the Viper rover at the specified position.
    void Initialize(const ChFrame<>& pos);

    /// Get the rover chassis.
    std::shared_ptr<ViperChassis> GetChassis() const { return m_chassis; }

    /// Get all rover wheels.
    std::array<std::shared_ptr<ViperWheel>, 4> GetWheels() const { return m_wheels; }

    /// Get the specified rover wheel.
    std::shared_ptr<ViperWheel> GetWheel(ViperWheelID id) const { return m_wheels[id]; }

    /// Get the specified rover upright.
    std::shared_ptr<ViperUpright> GetUpright(ViperWheelID id) const { return m_uprights[id]; }

    /// Get the specified rover upper arm.
    std::shared_ptr<ViperUpperArm> GetUpperArm(ViperWheelID id) const { return m_upper_arms[id]; }

    /// Get the specified rover lower arm.
    std::shared_ptr<ViperLowerArm> GetLowerArm(ViperWheelID id) const { return m_lower_arms[id]; }

    /// Get the specified rover driveshaft.
    std::shared_ptr<ChShaft> GetDriveshaft(ViperWheelID id) const { return m_drive_shafts[id]; }

    /// Get chassis position.
    ChVector<> GetChassisPos() const { return m_chassis->GetPos(); }

    /// Get chassis orientation.
    ChQuaternion<> GetChassisRot() const { return m_chassis->GetRot(); }

    /// Get chassis linear velocity.
    ChVector<> GetChassisVel() const { return m_chassis->GetLinVel(); }

    /// Get chassis linear acceleration.
    ChVector<> GetChassisAcc() const { return m_chassis->GetLinAcc(); }

    /// Get wheel speed.
    ChVector<> GetWheelLinVel(ViperWheelID id) const { return m_wheels[id]->GetLinVel(); }

    /// Get wheel angular velocity.
    ChVector<> GetWheelAngVel(ViperWheelID id) const { return m_wheels[id]->GetAngVel(); }

    /// Get wheel contact force.
    ChVector<> GetWheelContactForce(ViperWheelID id) const;

    /// Get wheel contact torque.
    ChVector<> GetWheelContactTorque(ViperWheelID id) const;

    /// Get wheel total applied force.
    ChVector<> GetWheelAppliedForce(ViperWheelID id) const;

    /// Get wheel tractive torque - if DC control set to off
    double GetWheelTracTorque(ViperWheelID id) const;

    /// Get wheel total applied torque.
    ChVector<> GetWheelAppliedTorque(ViperWheelID id) const;

    /// Get total rover mass.
    double GetRoverMass() const;

    /// Get total wheel mass.
    double GetWheelMass() const;

    /// Get drive motor function.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChFunction_Setpoint> GetDriveMotorFunc(ViperWheelID id) const { return m_drive_motor_funcs[id]; }

    /// Get steer motor function.
    std::shared_ptr<ChFunction_Const> GetSteerMotorFunc(ViperWheelID id) const { return m_steer_motor_funcs[id]; }

    /// Get drive motor.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChLinkMotorRotation> GetDriveMotor(ViperWheelID id) const { return m_drive_motors[id]; }

    /// Get steer motor.
    std::shared_ptr<ChLinkMotorRotation> GetSteerMotor(ViperWheelID id) const { return m_steer_motors[id]; }

    /// Viper update function.
    /// This function must be called before each integration step.
    void Update();

  private:
    /// Create the rover parts.
    void Create(ViperWheelType wheel_type);

    ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_chassis_fixed;  ///< fix chassis to ground

    std::shared_ptr<ViperChassis> m_chassis;                     ///< rover chassis
    std::array<std::shared_ptr<ViperWheel>, 4> m_wheels;         ///< rover wheels (LF, RF, LR, RB)
    std::array<std::shared_ptr<ViperUpperArm>, 4> m_upper_arms;  ///< rover upper arms (LF, RF, LR, RB)
    std::array<std::shared_ptr<ViperLowerArm>, 4> m_lower_arms;  ///< rover lower arms (LF, RF, LR, RB)
    std::array<std::shared_ptr<ViperUpright>, 4> m_uprights;     ///< rover steering stand (LF, RF, LR, RB)
    std::array<std::shared_ptr<ChBody>, 4> m_steering_rods;      ///< rover steering rod (LF, RF, LR, RB)

    std::array<std::shared_ptr<ChLinkMotorRotation>, 4> m_drive_motors;  ///< drive motors
    std::array<std::shared_ptr<ChLinkMotorRotation>, 4> m_steer_motors;  ///< steering motors
    std::array<std::shared_ptr<ChLinkMotorRotation>, 4> m_lift_motors;   ///< lifting motors

    std::array<std::shared_ptr<ChFunction_Setpoint>, 4> m_drive_motor_funcs;  ///< drive motor functions
    std::array<std::shared_ptr<ChFunction_Const>, 4> m_steer_motor_funcs;     ///< steering motor functions
    std::array<std::shared_ptr<ChFunction_Const>, 4> m_lift_motor_funcs;      ///< lifting motor functions

    std::array<std::shared_ptr<ChLinkTSDA>, 4> m_springs;    ///< suspension springs
    std::array<std::shared_ptr<ChShaft>, 4> m_drive_shafts;  ///< wheel drive-shafts

    std::shared_ptr<ViperDriver> m_driver;  ///< rover driver system

    std::shared_ptr<ChMaterialSurface> m_default_material;  ///< common contact material for all non-wheel parts
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)

    static const double m_max_steer_angle;  ///< maximum steering angle

    friend class ViperDCMotorControl;
};

// -----------------------------------------------------------------------------

/// Base class definition for a Viper driver.
/// A derived class must implement the Update function to set the various motor controls at the current time.
/// Alternatively, a derived class may directly access the associate Viper rover and control it through different means
/// (such as applying torques to the wheel driveshafts).
class CH_MODELS_API ViperDriver {
  public:
    /// Type of drive motor control.
    enum class DriveMotorType {
        SPEED,  ///< angular speed
        TORQUE  ///< torque
    };
    virtual ~ViperDriver() {}

    /// Indicate the control type for the drive motors.
    virtual DriveMotorType GetDriveMotorType() const = 0;

    /// Set current steering input (angle: negative for left, positive for right).
    void SetSteering(double angle);

    /// Set current steering input (angle: negative for left turn, positive for right turn).
    /// This function sets the steering angle for the specified wheel.
    void SetSteering(double angle, ViperWheelID id);

    /// Set current lift input angle.
    void SetLifting(double angle);

  protected:
    ViperDriver();

    /// Set the current rover driver inputs.
    /// This function is called by the associated Viper at each rover Update. A derived class must update the values for
    /// the angular speeds for the drive motors, as well as the angles for the steering motors and the lift motors at
    /// the specified time. A positive steering input corresponds to a left turn and a negative value to a right turn.
    virtual void Update(double time) = 0;

    Viper* viper;  ///< associated Viper rover

    std::array<double, 4> drive_speeds;  ///< angular speeds for drive motors
    std::array<double, 4> steer_angles;  ///< angles for steer motors
    std::array<double, 4> lift_angles;   ///< angles for lift motors

    friend class Viper;
};

/// Concrete Viper driver class for a simple DC motor control.
/// This implements a simplistic DC motor control by directly applying torques to the rover's driveshafts.
/// Control of the steering is left to the caller (through SetSteering).
class CH_MODELS_API ViperDCMotorControl : public ViperDriver {
  public:
    ViperDCMotorControl();
    ~ViperDCMotorControl() {}

    /// Set motor stall torque for the specified wheel (default: 300).
    void SetMotorStallTorque(double torque, ViperWheelID id) { m_stall_torque[id] = torque; }

    /// Set DC motor no load speed (default: pi).
    void SetMotorNoLoadSpeed(double speed, ViperWheelID id) { m_no_load_speed[id] = speed; }

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::TORQUE; }
    virtual void Update(double time) override;

    std::array<double, 4> m_stall_torque;   ///< stall torque of the motors
    std::array<double, 4> m_no_load_speed;  ///< no load speed of the motors
};

/// Concrete Viper speed driver.
/// This driver applies the same angular speed (ramped from 0 to a prescribed value) to all wheels.
class CH_MODELS_API ViperSpeedDriver : public ViperDriver {
  public:
    ViperSpeedDriver(double time_ramp, double speed);
    ~ViperSpeedDriver() {}

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override;

    double m_ramp;
    double m_speed;
};

/// @} robot_models_viper

}  // namespace viper
}  // namespace chrono

#endif
