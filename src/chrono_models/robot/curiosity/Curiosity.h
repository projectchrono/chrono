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
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotation.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {

/// Namespace with classes for the Curiosity model.
namespace curiosity {

/// @addtogroup robot_models_curiosity
/// @{

// -----------------------------------------------------------------------------

/// Curiosity wheel/suspension identifiers.
enum CuriosityWheelID {
    C_LF = 0,  ///< left front
    C_RF = 1,  ///< right front
    C_LM = 2,  ///< left middle
    C_RM = 3,  ///< right middle
    C_LB = 4,  ///< left back
    C_RB = 5   ///< right back
};

/// Curiosity chassis type.
enum class CuriosityChassisType { FullRover, Scarecrow };

/// Curiostiy wheel type.
enum class CuriosityWheelType { RealWheel, SimpleWheel, CylWheel };

// -----------------------------------------------------------------------------

/// Base class definition for all Curiosity Rover parts.
class CH_MODELS_API CuriosityPart {
  public:
    CuriosityPart(const std::string& name,                 ///< part name
                  const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                  std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                  bool collide                             ///< enable collision?
    );
    virtual ~CuriosityPart() {}

    /// Return the name of the part.
    const std::string& GetName() const { return m_name; }

    /// Set the name of the part.
    void SetName(const std::string& name) { m_name = name; }

    /// Enable/disable visualization.
    void SetVisualize(bool state) { m_visualize = state; }

    /// Enable/disable collision.
    void SetCollide(bool state) { m_collide = state; }

    /// Return the ChBody of the corresponding Curiosity part.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Return the position of the Curiosity part.
    /// This is the absolute location of the part reference frame.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Return the rotation of the Curiosity part.
    /// This is the orientation wrt the global frame of the part reference frame.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

    /// Return the linear velocity of the Curiopsity part.
    /// This is the absolute linear velocity of the part reference frame.
    const ChVector<>& GetLinVel() const { return m_body->GetFrame_REF_to_abs().GetPos_dt(); }

    /// Return the angular velocity of the Curiosity part.
    /// This is the absolute angular velocity of the part reference frame.
    const ChVector<> GetAngVel() const { return m_body->GetFrame_REF_to_abs().GetWvel_par(); }

    /// Return the linear acceleration of the Curiosity part.
    /// This is the absolute linear acceleration of the part reference frame.
    const ChVector<>& GetLinAcc() const { return m_body->GetFrame_REF_to_abs().GetPos_dtdt(); }

    /// Return the angular acceleratino of the Curiosity part.
    /// This is the absolute angular acceleratin of the part reference frame.
    const ChVector<> GetAngAcc() const { return m_body->GetFrame_REF_to_abs().GetWacc_par(); }

    /// Initialize the rover part by attaching it to the specified chassis body.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis);

  protected:
    /// Utility function for calculating mass properties using the part's collision mesh.
    void CalcMassProperties(double density);

    /// Construct the part body.
    void Construct(ChSystem* system);

    std::string m_name;                        ///< subsystem name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material

    std::string m_mesh_name;  ///< visualization mesh name
    ChFrame<> m_mesh_xform;   ///< mesh transform (translate, rotate, scale)
    ChColor m_color;          ///< visualization asset color

    ChFrame<> m_pos;       ///< relative position wrt the chassis
    double m_mass;         ///< mass
    ChVector<> m_inertia;  ///< principal moments of inertia
    ChFrame<> m_cog;       ///< COG frame (relative to body frame)

    bool m_visualize;  ///< part visualization flag
    bool m_collide;    ///< Curiosity part's collision indicator
};

// -----------------------------------------------------------------------------

/// Curiosity rover Chassis.
class CH_MODELS_API CuriosityChassis : public CuriosityPart {
  public:
    CuriosityChassis(const std::string& name,                ///< part name
                     CuriosityChassisType chassis_type,      ///< chassis type
                     std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~CuriosityChassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(ChSystem* system, const ChFrame<>& pos);

  private:
    CuriosityChassisType m_chassis_type;
};

/// Curiosity rover Wheel.
class CH_MODELS_API CuriosityWheel : public CuriosityPart {
  public:
    CuriosityWheel(const std::string& name,                 ///< part name
                   const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                   std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                   CuriosityWheelType wheel_type            ///< wheel type
    );
    ~CuriosityWheel() {}

    friend class Curiosity;
};

/// Curiosity rover suspension rocker.
class CH_MODELS_API CuriosityRocker : public CuriosityPart {
  public:
    CuriosityRocker(const std::string& name,                 ///< part name
                    const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                    std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                    int side                                 ///< rover side (0: L, 1: R)
    );
    ~CuriosityRocker() {}
};

/// Curiosity rover suspension bogie.
class CH_MODELS_API CuriosityBogie : public CuriosityPart {
  public:
    CuriosityBogie(const std::string& name,                 ///< part name
                   const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                   std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                   int side                                 ///< rover side (0: L, 1: R)
    );
    ~CuriosityBogie() {}
};

/// Curiosity rover steering upright.
class CH_MODELS_API CuriosityUpright : public CuriosityPart {
  public:
    CuriosityUpright(const std::string& name,                ///< part name
                     const ChFrame<>& rel_pos,               ///< position relative to chassis frame
                     std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~CuriosityUpright() {}
};

/// Curiosity rover differential bar.
class CH_MODELS_API CuriosityDifferentialBar : public CuriosityPart {
  public:
    CuriosityDifferentialBar(const std::string& name,                ///< part name
                             const ChFrame<>& rel_pos,               ///< position relative to chassis frame
                             std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~CuriosityDifferentialBar() {}
};

/// Curiosity rover differential link.
class CH_MODELS_API CuriosityDifferentialLink : public CuriosityPart {
  public:
    CuriosityDifferentialLink(const std::string& name,                 ///< part name
                              const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                              std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                              int side                                 ///< rover side (0: L, 1: R)
    );
    ~CuriosityDifferentialLink() {}
};

// -----------------------------------------------------------------------------

class CuriosityDriver;

/// Curiosity rover class.
/// This class encapsulates the location and rotation information of all Curiosity parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API Curiosity {
  public:
    Curiosity(ChSystem* system,
              CuriosityChassisType chassis_type = CuriosityChassisType::FullRover,
              CuriosityWheelType wheel_type = CuriosityWheelType::RealWheel);
    ~Curiosity() {}

    /// Get the containing system.
    ChSystem* GetSystem() { return m_system; }

    /// Set the curiosity driver
    void SetDriver(std::shared_ptr<CuriosityDriver> driver);

    /// Set wheel contact material.
    void SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat);

    /// Fix the chassis to ground.
    /// This function can only be invoked after the call to Initialize().
    void FixChassis(bool fixed);

    /// Fix the suspension joints.
    /// This function can only be invoked after the call to Initialize().
    void FixSuspension(bool fixed);

    /// Enable/disable visualization of the rover chassis (default: true).
    void SetChassisVisualization(bool state);

    /// Enable/disable visualization of rover wheels (default: true).
    void SetWheelVisualization(bool state);

    /// Enable/disable visualization of rover suspensions (default: true).
    void SetSuspensionVisualization(bool state);

    /// Initialize the Curiosity rover at the specified position.
    void Initialize(const ChFrame<>& pos);

    /// General control update function.
    /// This function must be called before each integration step.
    void Update();

    /// Get the rover chassis.
    std::shared_ptr<CuriosityChassis> GetChassis() const { return m_chassis; }

    /// Get the specified rover wheel.
    std::shared_ptr<CuriosityWheel> GetWheel(CuriosityWheelID id) const { return m_wheels[id]; }

    /// Get all rover wheels.
    std::array<std::shared_ptr<CuriosityWheel>, 6> GetWheels() const { return m_wheels; }

    /// Get the specified rover driveshaft.
    std::shared_ptr<ChShaft> GetDriveshaft(CuriosityWheelID id) const { return m_drive_shafts[id]; }

    /// Get chassis position.
    ChVector<> GetChassisPos() const { return m_chassis->GetPos(); }

    /// Get chassis orientation.
    ChQuaternion<> GetChassisRot() const { return m_chassis->GetRot(); }

    /// Get chassis linear velocity.
    ChVector<> GetChassisVel() const { return m_chassis->GetLinVel(); }

    /// Get chassis linear acceleration.
    ChVector<> GetChassisAcc() const { return m_chassis->GetLinAcc(); }

    /// Get wheel speed.
    ChVector<> GetWheelLinVel(CuriosityWheelID id) const { return m_wheels[id]->GetLinVel(); }

    /// Get wheel angular velocity.
    ChVector<> GetWheelAngVel(CuriosityWheelID id) const { return m_wheels[id]->GetAngVel(); }

    /// Get wheel contact force.
    ChVector<> GetWheelContactForce(CuriosityWheelID id) const;

    /// Get wheel contact torque.
    ChVector<> GetWheelContactTorque(CuriosityWheelID id) const;

    /// Get wheel total applied force.
    ChVector<> GetWheelAppliedForce(CuriosityWheelID id) const;

    /// Get wheel total applied torque.
    ChVector<> GetWheelAppliedTorque(CuriosityWheelID id) const;

    /// Get wheel tractive torque - if DC control is set to off.
    double GetWheelTracTorque(CuriosityWheelID id) const;

    /// Get total rover mass.
    double GetRoverMass() const;

    /// Get total wheel mass.
    double GetWheelMass() const;

    /// Get drive motor function.
    std::shared_ptr<ChFunction_Setpoint> GetDriveMotorFunc(CuriosityWheelID id) const {
        return m_drive_motor_funcs[id];
    }
    /// Get rocker steer motor function (side: 0 for left and 1 for right).
    std::shared_ptr<ChFunction_Const> GetRockerSteerMotorFunc(int side) const { return m_rocker_motor_funcs[side]; }
    /// Get bogie steer motor function (side: 0 for left and 1 for right).
    std::shared_ptr<ChFunction_Const> GetBogieSteerMotorFunc(int side) const { return m_bogie_motor_funcs[side]; }

    /// Get drive motor.
    std::shared_ptr<ChLinkMotorRotation> GetDriveMotor(CuriosityWheelID id) const { return m_drive_motors[id]; }
    /// Get rocker steer motor (side: 0 for left and 1 for right).
    std::shared_ptr<ChLinkMotorRotation> GetRockerSteerMotor(int side) const { return m_rocker_motors[side]; }
    /// Get bogie steer motor (side: 0 for left and 1 for right).
    std::shared_ptr<ChLinkMotorRotation> GetBogieSteerMotor(int side) const { return m_bogie_motors[side]; }

  private:
    /// Create the rover parts.
    void Create(CuriosityChassisType chassis_type, CuriosityWheelType wheel_type);

    ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_initialized;  ///< flag indicating whether or not the rover was initialized

    std::shared_ptr<CuriosityChassis> m_chassis;                             ///< chassis
    std::array<std::shared_ptr<CuriosityWheel>, 6> m_wheels;                 ///< wheels (see CuriosityWheelID)
    std::array<std::shared_ptr<CuriosityRocker>, 2> m_rockers;               ///< suspension rockers (L/R)
    std::array<std::shared_ptr<CuriosityBogie>, 2> m_bogies;                 ///< suspension bogies (L/R)
    std::array<std::shared_ptr<CuriosityUpright>, 2> m_rocker_uprights;      ///< steering rocker uprights (L/R)
    std::array<std::shared_ptr<CuriosityUpright>, 2> m_bogie_uprights;       ///< steering bogie uprights (L/R)
    std::shared_ptr<CuriosityDifferentialBar> m_diff_bar;                    ///< differential bar
    std::array<std::shared_ptr<CuriosityDifferentialLink>, 3> m_diff_links;  ///< differential links (L/R)

    std::shared_ptr<ChLinkLockRevolute> m_diff_joint;                    ///< joint connecting the differential bar
    std::array<std::shared_ptr<ChLinkLockRevolute>, 2> m_rocker_joints;  ///< joints connecting suspension rockers (L/R)
    std::array<std::shared_ptr<ChLinkLockRevolute>, 2> m_bogie_joints;   ///< joints connecting suspension bogies (L/R)

    std::array<std::shared_ptr<ChLinkMotorRotation>, 2> m_rocker_motors;    ///< rocker steering motors
    std::array<std::shared_ptr<ChLinkMotorRotation>, 2> m_bogie_motors;     ///< bogie steering motors
    std::array<std::shared_ptr<ChFunction_Const>, 2> m_rocker_motor_funcs;  ///< rocker steering motor functions
    std::array<std::shared_ptr<ChFunction_Const>, 2> m_bogie_motor_funcs;   ///< bogie steering motor functions

    std::array<std::shared_ptr<ChLinkMotorRotation>, 6> m_drive_motors;       ///< drive motors
    std::array<std::shared_ptr<ChFunction_Setpoint>, 6> m_drive_motor_funcs;  ///< drive motor functions

    std::array<std::shared_ptr<ChShaft>, 6> m_drive_shafts;  ///< power shafts for torque-controlled drive mode

    std::shared_ptr<CuriosityDriver> m_driver;  ///< rover driver system

    std::shared_ptr<ChMaterialSurface> m_default_material;  ///< common contact material for all non-wheel parts
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)

    friend class CuriosityDCMotorControl;
};

// -----------------------------------------------------------------------------

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

    /// Set current steering input (angle: negative for left turn, positive for right turn).
    /// This function sets the same steering angle for all steerable wheels.
    void SetSteering(double angle);

    /// Set current steering input (angle: negative for left turn, positive for right turn).
    /// This function sets the steering angle for the specified wheel.
    void SetSteering(double angle, CuriosityWheelID id);

    /// Indicate the control type for the drive motors.
    virtual DriveMotorType GetDriveMotorType() const = 0;

  protected:
    CuriosityDriver();

    /// Set the current rover driver inputs.
    /// This function is called by the associated Curiosity at each rover Update. A derived class must update the values
    /// for the angular speeds for the drive motors, as well as the angles for the steering motors and the lift motors
    /// at the specified time. A positive steering input corresponds to a left turn and a negative value to a right
    /// turn.
    virtual void Update(double time) = 0;

    Curiosity* curiosity;  ///< associated Curiosity rover

    std::array<double, 6> drive_speeds;  ///< angular speeds for drive motors (positive for forward motion)
    std::array<double, 4> steer_angles;  ///< angles for steer motors (negative for left turn, positive for right turn)

    friend class Curiosity;
};

/// Concrete Curiosity driver class for a simple DC motor control.
/// This implements a simplistic DC motor control by directly applying torques to the rover's driveshafts.
/// Control of the steering is left to the caller (through SetSteering).
class CH_MODELS_API CuriosityDCMotorControl : public CuriosityDriver {
  public:
    CuriosityDCMotorControl();
    ~CuriosityDCMotorControl() {}

    /// Set motor stall torque for the specified wheel (default: 300).
    void SetMotorStallTorque(double torque, CuriosityWheelID id) { m_stall_torque[id] = torque; }

    /// Set DC motor no load speed (default: pi).
    void SetMotorNoLoadSpeed(double speed, CuriosityWheelID id) { m_no_load_speed[id] = speed; }

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::TORQUE; }
    virtual void Update(double time) override;

    std::array<double, 6> m_stall_torque;   ///< stall torque of the motors
    std::array<double, 6> m_no_load_speed;  ///< no load speed of the motors
};

/// Concrete Curiosity speed driver.
/// This driver applies the same angular speed (ramped from 0 to a prescribed value) to all wheels.
class CH_MODELS_API CuriositySpeedDriver : public CuriosityDriver {
  public:
    CuriositySpeedDriver(double time_ramp, double speed);
    ~CuriositySpeedDriver() {}

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override;

    double m_ramp;
    double m_speed;
};

/// @} robot_models_curiosity

}  // namespace curiosity
}  // namespace chrono

#endif
