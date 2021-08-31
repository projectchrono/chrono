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
// Authors: Jason Zhou
// =============================================================================
//
// NASA Curiosity Mars Rover Model Class.
// This class contains model for NASA's 6-wheel mars rover curiosity
//
// =============================================================================

#ifndef CURIOSITY_H
#define CURIOSITY_H

#include <array>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsBody.h"

#include "chrono_models/ChApiModels.h"

#include "chrono/physics/ChInertiaUtils.h"

namespace chrono {

/// Namespace with classes for the Curiosity model.
namespace curiosity {

/// @addtogroup robot_models_curiosity
/// @{

/// Curiosity wheel/suspension identifiers.
enum WheelID {
    LF,  ///< left front
    RF,  ///< right front
    LM,  ///< left middle
    RM,  ///< right middle
    LB,  ///< left back
    RB   ///< right back
};

/// Curiosity chassis type
enum Chassis_Type { FullRover, Scarecrow };

/// Curiostiy wheel type
enum Wheel_Type { RealWheel, SimpleWheel, CylWheel };

/// Base class definition of the Curiosity Rover Part.
class CH_MODELS_API Curiosity_Part {
  public:
    Curiosity_Part(const std::string& name,
                   bool fixed,
                   ChSystem* system,
                   const ChFrame<>& body_pos,
                   std::shared_ptr<ChBodyAuxRef> chassis_body,
                   bool collide);
    virtual ~Curiosity_Part() {}

    /// Return the name of the part.
    const std::string& GetName() const { return m_name; }

    /// Set the name of the part.
    void SetName(const std::string& name) { m_name = name; }

    /// Return the ChBody of the corresponding Curiosity part.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Return the ChBody of the chassis wrt the Curiosity part.
    std::shared_ptr<ChBodyAuxRef> GetChassis() const { return m_chassis; }

    /// Return the Position of the Curiosity part.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Return the Rotation of the Curiosity part.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

    /// Initialize the rover part by attaching it to the specified chassis body.
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis);

  protected:
    void Construct(ChSystem* system);

    /// Enable/disable collision.
    void SetCollide(bool state);

    ChFrame<> m_mesh_xform;

    std::string m_name;                        ///< subsystem name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material (shared among all shapes)

    std::string m_mesh_name;                  ///< visualization mesh name
    ChColor m_color;                          ///< visualization asset color
    ChSystem* m_system;                       ///< system which Curiosity Part belongs to
    std::shared_ptr<ChBodyAuxRef> m_chassis;  ///< the chassis body for the rover

    ChFrame<> m_pos;   ///< Curiosity part's relative position wrt the chassis
    double m_density;  ///< Curiosity part's density

    bool m_visualize;  ///< part visualization flag
    bool m_collide;    ///< Curiosity part's collision indicator
    bool m_fixed;      ///< Curiosity part's fixed indication
};

/// Curiosity rover Chassis.
class CH_MODELS_API Curiosity_Chassis : public Curiosity_Part {
  public:
    Curiosity_Chassis(const std::string& name,
                      bool fixed,
                      ChSystem* system,
                      const ChFrame<>& body_pos,
                      bool collide,
                      Chassis_Type chassis_type);
    ~Curiosity_Chassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(ChSystem* system, const ChFrame<>& pos);

  private:
    Chassis_Type m_chassis_type = Chassis_Type::FullRover;

    friend class CuriosityRover;
};

/// Curiosity rover Wheel.
class CH_MODELS_API Curiosity_Wheel : public Curiosity_Part {
  public:
    Curiosity_Wheel(const std::string& name,
                    bool fixed,
                    ChSystem* system,
                    const ChFrame<>& body_pos,
                    std::shared_ptr<ChBodyAuxRef> chassis,
                    bool collide,
                    Wheel_Type wheel_type);
    ~Curiosity_Wheel() {}
    friend class CuriosityRover;
};

/// Curiosity rover Connecting Arm.
class CH_MODELS_API Curiosity_Arm : public Curiosity_Part {
  public:
    Curiosity_Arm(const std::string& name,
                  bool fixed,
                  ChSystem* system,
                  const ChFrame<>& body_pos,
                  std::shared_ptr<ChBodyAuxRef> chassis,
                  bool collide,
                  const int& side);  // 0 == FL, 1 == FR, 2 == BL, 3 == BR
    ~Curiosity_Arm() {}
    friend class CuriosityRover;
};

/// Curiosity rover steering rod.
class CH_MODELS_API Curiosity_Steer : public Curiosity_Part {
  public:
    Curiosity_Steer(const std::string& name,
                    bool fixed,
                    ChSystem* system,
                    const ChFrame<>& body_pos,
                    std::shared_ptr<ChBodyAuxRef> chassis,
                    bool collide);
    ~Curiosity_Steer() {}
    friend class Curiosity_Rover;
};

/// Curiosity rover steering rod.
class CH_MODELS_API Curiosity_Balancer : public Curiosity_Part {
  public:
    Curiosity_Balancer(const std::string& name,
                       bool fixed,
                       ChSystem* system,
                       const ChFrame<>& body_pos,
                       std::shared_ptr<ChBodyAuxRef> chassis,
                       bool collide,
                       const int& side);  // 0 - > L, 1 - > R, 2 - > M
    ~Curiosity_Balancer() {}
    friend class CuriosityRover;
};

// =========================================

class CuriosityDriver;

// =========================================

/// Curiosity rover class.
/// This class encapsulates the location and rotation information of all Curiosity parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API CuriosityRover {
  public:
    CuriosityRover(ChSystem* system,
                   Chassis_Type chassis_type = Chassis_Type::FullRover,
                   Wheel_Type wheel_type = Wheel_Type::RealWheel);
    ~CuriosityRover();

    /// Initialize the Curiosity rover using current parameters.
    void Initialize(const ChFrame<>& pos);

    /// General control updated function
    /// Note: This is the main caller function, submodule controls such as steering and traction
    /// are handled by sub helper functions
    void Update();

    /// Get the ChSystem
    ChSystem* GetSystem() { return m_system; }

    /// Get Chassis Type
    Chassis_Type GetChassisType();

    /// Get wheel speed
    ChVector<> GetWheelSpeed(WheelID id);

    /// Get wheel angular velocity
    ChVector<> GetWheelAngVel(WheelID id);

    /// Get wheel contact force
    ChVector<> GetWheelContactForce(WheelID id);

    /// Get wheel contact torque
    ChVector<> GetWheelContactTorque(WheelID id);

    /// Get wheel total applied force
    ChVector<> GetWheelAppliedForce(WheelID id);

    /// Get wheel total applied torque
    ChVector<> GetWheelAppliedTorque(WheelID id);

    /// Get wheel tractive torque - if DC control is set to off
    double GetWheelTracTorque(WheelID id);

    /// Get the chassis body
    std::shared_ptr<ChBodyAuxRef> GetChassisBody();

    /// Get the wheel body
    std::shared_ptr<ChBodyAuxRef> GetWheelBody(WheelID id);

    /// Get the wheel trimesh
    /// This function is required for GPU module coupling
    std::shared_ptr<geometry::ChTriangleMeshConnected> GetWheelTrimesh(WheelID id);

    /// Get the wheel part
    /// This function returns the curiosity part, required for GPU module coupling
    std::shared_ptr<Curiosity_Wheel> GetWheelPart(WheelID id);

    /// Get chassis speedometer
    ChVector<> GetChassisVel();

    /// Get chassis accelerometer
    ChVector<> GetChassisAcc();

    /// Get chassis rotation
    ChQuaternion<> GetChassisRot();

    /// Get steering motor angle -/+ range
    double GetSteerAngle(WheelID id);

    /// Get steering motor speed -/+ range
    double GetSteerSpeed(WheelID id);

    /// Get total rover mass
    double GetRoverMass();

    /// Get total wheel mass
    double GetWheelMass();

    /// Set the curiosity driver
    void SetDriver(std::shared_ptr<CuriosityDriver> driver);

    /// Set wheel contact material.
    void SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat);

  private:
    ChSystem* m_system;  ///< pointer to the Chrono system

    Chassis_Type m_chassis_type = Chassis_Type::FullRover;  ///< curiosity chassis type
    Wheel_Type m_wheel_type = Wheel_Type::RealWheel;        ///< curiosity wheel type

    bool m_dc_motor_control = false;  ///< enable dc motor control, default set to false

    std::shared_ptr<Curiosity_Chassis> m_chassis;              ///< rover chassis
    std::array<std::shared_ptr<Curiosity_Wheel>, 6> m_wheels;  ///< rover wheels - 1:LF, 2:RF, 3:LM, 4:RM, 5:LB, 6:RB
    std::array<std::shared_ptr<Curiosity_Arm>, 4> m_arms;      ///< rover arms
    std::array<std::shared_ptr<Curiosity_Steer>, 4> m_steers;  ///< rover steering rods
    std::array<std::shared_ptr<Curiosity_Balancer>, 3> m_balancers;  ///< rover balancers parts

    ChFrame<> m_rover_pos;  ///< rover placement position

    std::array<std::shared_ptr<ChLinkMotorRotation>, 6> m_motors;        ///< array to store motors
                                                                         ///< 0-LF,1-RF,2-LM,3-RM,4-LB,5-RB
    std::array<std::shared_ptr<ChLinkMotorRotation>, 6> m_steer_motors;  ///< array to store steering motors
                                                                         ///< 0-LF,1-RF,2-LM,3-RM,4-LB,5-RB

    std::array<std::shared_ptr<ChShaft>, 6> m_drive_shafts;  ///< power shafts for torque-controlled drive mode

    std::shared_ptr<CuriosityDriver> m_driver;  ///< rover driver system

    std::array<std::shared_ptr<ChFunction_Setpoint>, 6> m_motors_func;  ///< constant motor speed functions
    std::array<std::shared_ptr<ChFunction_Const>, 6>
        m_steer_motors_func;  ///< constant steering motor angular speed func

    std::array<double, 6> m_stall_torque;   ///< stall torque of the motors
    std::array<double, 6> m_no_load_speed;  ///< no load speed of the motors

    static const double m_max_steer_angle;  ///< maximum steering angle
    friend class CuriosityDCMotorControl;
};

/// Base class definition for a Curiosity driver.
/// A derived class must implement the Update function to set the various motor controls at the current time.
/// Alternatively, a derived class may directly access the associate Curiosity rover and control it through different
/// means (such as applying torques to the wheel driveshafts).
class CH_MODELS_API CuriosityDriver {
  public:
    /// Type of drive motor control.
    enum class DriveMotorType {
        SPEED,  ///< angular speed
        TORQUE  ///< torque
    };
    virtual ~CuriosityDriver() {}

    /// Indicate the control type for the drive motors.
    virtual DriveMotorType GetDriveMotorType() const = 0;

    /// Set the current rover driver inputs.
    /// This function is called by the associated Curiosity at each rover Update. A derived class must update the values
    /// for the angular speeds for the drive motors, as well as the angles for the steering motors and the lift motors
    /// at the specified time. A positive steering input corresponds to a left turn and a negative value to a right
    /// turn.
    virtual void Update(double time) = 0;

  protected:
    CuriosityDriver();

    CuriosityRover* curiosity;  ///< associated Curiosity rover

    std::array<double, 6> drive_speeds;  ///< angular speeds for drive motors
    std::array<double, 4> steer_angles;  ///< angles for steer motors

    friend class CuriosityRover;
};

/// Concrete Curiosity driver class for a simple DC motor control.
/// This implements a simplistic DC motor control by directly applying torques to the rover's driveshafts.
/// Control of the steering is left to the caller (through SetSteering).
class CH_MODELS_API CuriosityDCMotorControl : public CuriosityDriver {
  public:
    CuriosityDCMotorControl();
    ~CuriosityDCMotorControl() {}

    /// Set motor stall torque for the specified wheel (default: 300).
    void SetMotorStallTorque(double torque, WheelID id) { m_stall_torque[id] = torque; }

    /// Set DC motor no load speed (default: pi).
    void SetMotorNoLoadSpeed(double speed, WheelID id) { m_no_load_speed[id] = speed; }

    /// Set current steering input (angle: negative for left, positive for right).
    void SetSteering(double angle, WheelID id);

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::TORQUE; }
    virtual void Update(double time) override;

    std::array<double, 6> m_stall_torque;   ///< stall torque of the motors
    std::array<double, 6> m_no_load_speed;  ///< no load speed of the motors
};

/// Concrete Curiosity driver class for a constant speed drive motor control.
/// Control of the steering is left to the caller (through SetSteering).
class CH_MODELS_API CuriosityConstMotorControl : public CuriosityDriver {
  public:
    CuriosityConstMotorControl();
    ~CuriosityConstMotorControl() {}

    /// Set motor const speed (default: pi).
    void SetMotorSpeed(double speed, WheelID id) { m_const_speed[id] = speed; }

    /// Set current steering input
    void SetSteering(double angle, WheelID id);

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override;

    std::array<double, 6> m_const_speed;  ///< current constant motor speed (default: pi).
};

/// @} robot_models_curiosity

}  // namespace curiosity
}  // namespace chrono
#endif
