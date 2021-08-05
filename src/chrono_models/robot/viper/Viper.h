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
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {

/// Namespace with classes for the Viper model.
namespace viper {

/// @addtogroup robot_models_viper
/// @{

/// Viper wheel/suspension identifiers.
enum WheelID {
    LF = 0,  ///< left front
    RF = 1,  ///< right front
    LB = 2,  ///< left back
    RB = 3   ///< right back
};

/// Viper wheel type.
enum class WheelType {
    RealWheel,    ///< actual geometry of the Viper wheel
    SimpleWheel,  ///< simplified wheel geometry
    CylWheel      ///< cylindrical wheel geometry
};

// -----------------------------------------------------------------------------

/// Base class definition for all Viper parts.
/// Viper Rover Parts include Chassis, Steering, Upper Suspension Arm, Bottom Suspension Arm and Wheel.
/// This class encapsulates base fields and functions.
class CH_MODELS_API ViperPart {
  public:
    ViperPart(const std::string& name, const ChFrame<>& rel_pos, std::shared_ptr<ChMaterialSurface> mat, bool collide);
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
    /// Complete construction of the part.
    void Construct(ChSystem* system);

    std::string m_name;                        ///< part name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< part rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material (shared among all shapes)

    ChFrame<> m_mesh_xform;   ///< mesh transform (translate, rotate, scale)
    std::string m_mesh_name;  ///< visualization mesh name
    ChColor m_color;          ///< visualization asset color

    ChFrame<> m_pos;   ///< relative position wrt chassis
    double m_density;  ///< part density

    bool m_visualize;  ///< part visualization flag
    bool m_collide;    ///< part collision flag
};

/// Viper rover Chassis.
class CH_MODELS_API ViperChassis : public ViperPart {
  public:
    ViperChassis(const std::string& name, std::shared_ptr<ChMaterialSurface> mat);
    ~ViperChassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(ChSystem* system, const ChFrame<>& pos);
};

/// Viper rover Wheel.
class CH_MODELS_API ViperWheel : public ViperPart {
  public:
    ViperWheel(const std::string& name,
               const ChFrame<>& rel_pos,
               std::shared_ptr<ChMaterialSurface> mat,
               WheelType wheel_type);
    ~ViperWheel() {}

    friend class Viper;
};

/// The upper arm of the Viper rover suspension.
class CH_MODELS_API ViperUpperArm : public ViperPart {
  public:
    ViperUpperArm(const std::string& name,
                  const ChFrame<>& rel_pos,
                  std::shared_ptr<ChMaterialSurface> mat,
                  const int& side);  ///< indicate which side of the suspension 0->L, 1->R
    ~ViperUpperArm() {}
};

/// The bottom arm of the Viper rover suspension.
class CH_MODELS_API ViperLowerArm : public ViperPart {
  public:
    ViperLowerArm(const std::string& name,
                  const ChFrame<>& rel_pos,
                  std::shared_ptr<ChMaterialSurface> mat,
                  const int& side);  ///< indicate which side of the suspension 0->L, 1->R
    ~ViperLowerArm() {}
};

/// Steering rod of the Viper rover.
/// The steering rod is connected to the steering cyl, this link is controlled steering.
/// There are two connecting rods on the steering rod, linking to upper and bottom arms of the suspension.
class CH_MODELS_API ViperUpright : public ViperPart {
  public:
    ViperUpright(const std::string& name,
                 const ChFrame<>& rel_pos,
                 std::shared_ptr<ChMaterialSurface> mat,
                 const int& side);  ///< indicate which side of the rover 0->L, 1->R
    ~ViperUpright() {}
};

// -----------------------------------------------------------------------------

class ViperDriver;

/// Viper rover class.
/// This class encapsulates the location and rotation information of all Viper parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API Viper {
  public:
    Viper(ChSystem* system, WheelType wheel_type = WheelType::RealWheel);

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

    /// Get the chassis part.
    std::shared_ptr<ViperChassis> GetChassis() const { return m_chassis; }

    /// Get the specified rover wheel.
    std::shared_ptr<ViperWheel> GetWheel(WheelID id) const { return m_wheels[id]; }

    /// Get the specified rover upright.
    std::shared_ptr<ViperUpright> GetUpright(WheelID id) const { return m_uprights[id]; }

    /// Get the specified rover upper arm.
    std::shared_ptr<ViperUpperArm> GetUpperArm(WheelID id) const { return m_upper_arms[id]; }

    /// Get the specified rover lower arm.
    std::shared_ptr<ViperLowerArm> GetLowerArm(WheelID id) const { return m_lower_arms[id]; }

    /// Get the specified rover driveshaft.
    std::shared_ptr<ChShaft> GetDriveshaft(WheelID id) const { return m_drive_shafts[id]; }

    /// Get chassis position.
    ChVector<> GetChassisPos() const { return m_chassis->GetPos(); }

    /// Get chassis orientation.
    ChQuaternion<> GetChassisRot() const { return m_chassis->GetRot(); }

    /// Get chassis linear velocity.
    ChVector<> GetChassisVel() const { return m_chassis->GetLinVel(); }

    /// Get chassis linear acceleration.
    ChVector<> GetChassisAcc() const { return m_chassis->GetLinAcc(); }

    /// Get wheel speed.
    ChVector<> GetWheelLinVel(WheelID id) const { return m_wheels[id]->GetLinVel(); }

    /// Get wheel angular velocity.
    ChVector<> GetWheelAngVel(WheelID id) const { return m_wheels[id]->GetAngVel(); }

    /// Get wheel contact force.
    ChVector<> GetWheelContactForce(WheelID id) const;

    /// Get wheel contact torque.
    ChVector<> GetWheelContactTorque(WheelID id) const;

    /// Get wheel total applied force.
    ChVector<> GetWheelAppliedForce(WheelID id) const;

    /// Get wheel tractive torque - if DC control set to off
    double GetWheelTracTorque(WheelID id) const;

    /// Get wheel total applied torque.
    ChVector<> GetWheelAppliedTorque(WheelID id) const;

    /// Get total rover mass.
    double GetRoverMass() const;

    /// Get total wheel mass.
    double GetWheelMass() const;

    /// Get drive motor function.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChFunction_Setpoint> GetDriveMotorFunc(WheelID id) { return m_drive_motor_funcs[id]; }

    /// Get steer motor function.
    std::shared_ptr<ChFunction_Const> GetSteerMotorFunc(WheelID id) { return m_steer_motor_funcs[id]; }

    /// Get drive motor.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChLinkMotorRotationSpeed> GetDriveMotor(WheelID id) { return m_drive_motors[id]; }

    /// Get steer motor.
    std::shared_ptr<ChLinkMotorRotationSpeed> GetSteerMotor(WheelID id) { return m_steer_motors[id]; }

    /// Get Viper turning angle.
    double GetTurnAngle() const;

    /// Viper update function.
    /// This function must be called before each integration step.
    void Update();

  private:
    /// Create the rover parts.
    void Create(WheelType wheel_type);

    ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_chassis_fixed;  ///< fix chassis to ground

    std::shared_ptr<ViperChassis> m_chassis;                     ///< rover chassis
    std::array<std::shared_ptr<ViperWheel>, 4> m_wheels;         ///< rover wheels (LF, RF, LR, RB)
    std::array<std::shared_ptr<ViperUpperArm>, 4> m_upper_arms;  ///< rover upper arms (LF, RF, LR, RB)
    std::array<std::shared_ptr<ViperLowerArm>, 4> m_lower_arms;  ///< rover lower arms (LF, RF, LR, RB)
    std::array<std::shared_ptr<ViperUpright>, 4> m_uprights;     ///< rover steering stand (LF, RF, LR, RB)
    std::array<std::shared_ptr<ChBody>, 4> m_steering_rods;      ///< rover steering rod (LF, RF, LR, RB)

    std::array<std::shared_ptr<ChLinkMotorRotationSpeed>, 4> m_drive_motors;  ///< drive motors
    std::array<std::shared_ptr<ChLinkMotorRotationSpeed>, 4> m_steer_motors;  ///< steering motors
    std::array<std::shared_ptr<ChLinkMotorRotationSpeed>, 8> m_lift_motors;   ///< lifting motors

    std::array<std::shared_ptr<ChFunction_Setpoint>, 4> m_drive_motor_funcs;  ///< drive motor functions
    std::array<std::shared_ptr<ChFunction_Const>, 4> m_steer_motor_funcs;     ///< steering motor functions
    std::array<std::shared_ptr<ChFunction_Const>, 4> m_lift_motor_funcs;      ///< lifting motor functions

    std::array<std::shared_ptr<ChLinkTSDA>, 4> m_springs;    ///< suspension springs
    std::array<std::shared_ptr<ChShaft>, 4> m_drive_shafts;  ///< wheel drive-shafts

    std::shared_ptr<ViperDriver> m_driver;  ///< rover driver system

    std::shared_ptr<ChMaterialSurface> m_default_material;  ///< common contact material for all non-wheel parts
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)

    static const double m_max_steer_angle;  ///< maximum steering angle
    static const double m_max_steer_speed;  ///< maximum steering speed

    friend class ViperDCMotorControl;
};

// -----------------------------------------------------------------------------

/// Base class definition for a Viper driver.
/// A derived class must implement the Update function to set the various motor controls at the current time.
/// Alternatively, a derived class may directly access the associate Viper rover and control it through different means
/// (such as applying torques to the wheel driveshafts).
class CH_MODELS_API ViperDriver {
  public:
    virtual ~ViperDriver() {}

    /// Indicate if driver controls driveshafts through applied torque.
    virtual bool TorqueControl() { return false; }

    /// Set the current rover driver inputs.
    /// This function is called by the associated Viper at each rover Update. A derived class must update the values for
    /// the angular speeds for the drive motors, the steering motors, and the lift motors at the specified time. A
    /// positive steering input corresponds to a left turn and a negative value to a right turn.
    virtual void Update(double time) = 0;

  protected:
    ViperDriver();

    Viper* viper;  ///< associated Viper rover

    std::array<double, 4> drive_speeds;  ///< angular speeds for drive motors
    std::array<double, 4> steer_speeds;  ///< angular speeds for steer motors
    std::array<double, 4> lift_speeds;   ///< angular speeds for lift motors

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
    void SetMotorStallTorque(double torque, WheelID id) { m_stall_torque[id] = torque; }

    /// Set DC motor no load speed (default: pi).
    void SetMotorNoLoadSpeed(double speed, WheelID id) { m_no_load_speed[id] = speed; }

    /// Set current steering input (angular speed: negative for left, positive for right).
    void SetSteering(double speed);

    /// Set current lift input (angular speed).
    void SetLift(double speed);

  private:
    virtual bool TorqueControl() override { return true; }
    virtual void Update(double time) override;

    std::array<double, 4> m_stall_torque;   ///< stall torque of the motors
    std::array<double, 4> m_no_load_speed;  ///< no load speed of the motors
};

/// @} robot_models_viper

}  // namespace viper
}  // namespace chrono
#endif
