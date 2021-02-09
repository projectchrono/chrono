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
// NASA VIPER Lunar Rover Model Class.
// This class contains model for NASA's VIPER lunar rover for NASA's 2024 Moon
// exploration mission. 
//
// =============================================================================

#ifndef VIPER_H
#define VIPER_H

#include <array>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_models/ChApiModels.h"

#include "chrono/physics/ChInertiaUtils.h"

namespace chrono {

/// Namespace with classes for the Viper model.
namespace viper {

/// @addtogroup robot_models_viper
/// @{

/// Viper wheel/suspension identifiers.
enum WheelID {
    LF,  ///< left front
    RF,  ///< right front
    LB,  ///< left back
    RB   ///< right back
};

/// Viper turning signal
enum TurnSig{
  L,  ///< left turn signal
  R,  ///< right turn signal
  HOLD  ///< hold signal
};

/// Base class definition of the Viper Rover Part.
/// Viper Rover Parts include Chassis, Steering, Upper Suspension Arm, Bottom Suspension Arm and Wheel
/// This class encapsulates base fields and functions 
class CH_MODELS_API Viper_Part {
  public:
    Viper_Part(const std::string& name,
               bool fixed,
               std::shared_ptr<ChMaterialSurface> mat,
               ChSystem* system,
               const ChVector<>& body_pos,
               const ChQuaternion<>& body_rot,
               std::shared_ptr<ChBodyAuxRef> chassis_body,
               bool collide);
    virtual ~Viper_Part() {}

    /// Return the name of the part.
    const std::string& GetName() const { return m_name; }

    /// Set the name of the part.
    void SetName(const std::string& name) { m_name = name; }

    /// Return the ChBody of the corresponding Viper part.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Return the ChBody of the chassis wrt the Viper part.
    std::shared_ptr<ChBodyAuxRef> GetChassis() const { return m_chassis; }

    /// Return the Position of the Viper part.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Return the Rotation of the Viper part.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

  protected:

    /// Initialize the visulization mesh of the Viper part.
    void AddVisualizationAssets();

    /// Initialize the collision mesh of the Viper part.
    void AddCollisionShapes();

    /// Enable/disable collision.
    void SetCollide(bool state);

    std::string m_name;                        ///< subsystem name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material (shared among all shapes)

    std::string m_mesh_name;                  ///< visualization mesh name
    ChVector<> m_offset;                      ///< offset for visualization mesh
    ChColor m_color;                          ///< visualization asset color
    ChSystem* m_system;                       ///< system which Viper Part belongs to
    std::shared_ptr<ChBodyAuxRef> m_chassis;  ///< the chassis body for the rover

    ChVector<> m_pos;      ///< Viper part's relative position wrt the chassis
    ChQuaternion<> m_rot;  ///< Viper part's relative rotation wrt the chassis
    double m_density;      ///< Viper part's density

    bool m_collide; ///< Viper part's collision indicator
    bool m_fixed; ///< Viper part's fixed indication
};

/// Viper rover Chassis.
class CH_MODELS_API Viper_Chassis : public Viper_Part {
  public:
    Viper_Chassis(const std::string& name,
                  bool fixed,
                  std::shared_ptr<ChMaterialSurface> mat,
                  ChSystem* system,
                  const ChVector<>& body_pos,
                  const ChQuaternion<>& body_rot,
                  bool collide);
    ~Viper_Chassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the rover chassis.
    void SetCollide(bool state);

  private:

    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

/// Viper rover Wheel.
class CH_MODELS_API Viper_Wheel : public Viper_Part {
  public:
    Viper_Wheel(const std::string& name,
                bool fixed,
                std::shared_ptr<ChMaterialSurface> mat,
                ChSystem* system,
                const ChVector<>& body_pos,
                const ChQuaternion<>& body_rot,
                std::shared_ptr<ChBodyAuxRef> chassis,
                bool collide);
    ~Viper_Wheel() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:

    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

/// The upper arm of the Viper rover suspension.
class CH_MODELS_API Viper_Up_Arm : public Viper_Part {
  public:
    Viper_Up_Arm(const std::string& name,
                 bool fixed,
                 std::shared_ptr<ChMaterialSurface> mat,
                 ChSystem* system,
                 const ChVector<>& body_pos,
                 const ChQuaternion<>& body_rot,
                 std::shared_ptr<ChBodyAuxRef> chassis,
                 bool collide,
                 const int& side);  ///< indicate which side of the suspension 0->L, 1->R
    ~Viper_Up_Arm() {}

    /// Initialize the upper arm of the suspension at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the upper arm of the suspension.
    void SetCollide(bool state);

  private:

    /// Translate the upper arm by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

/// The bottom arm of the Viper rover suspension.
class CH_MODELS_API Viper_Bottom_Arm : public Viper_Part {
  public:
    Viper_Bottom_Arm(const std::string& name,
                 bool fixed,
                 std::shared_ptr<ChMaterialSurface> mat,
                 ChSystem* system,
                 const ChVector<>& body_pos,
                 const ChQuaternion<>& body_rot,
                 std::shared_ptr<ChBodyAuxRef> chassis,
                 bool collide,
                 const int& side);  ///< indicate which side of the suspension 0->L, 1->R
    ~Viper_Bottom_Arm() {}

    /// Initialize the bottom arm of the suspension at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the bottom arm.
    void SetCollide(bool state);

  private:

    /// Translate the bottom arm by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

/// Steering rod of the Viper rover.
/// The steering rod is connected to the steering cyl, this link is controlled steering.
/// There are two connecting rods on the steering rod, linking to upper and bottom arms of the suspension.
class CH_MODELS_API Viper_Steer : public Viper_Part {
  public:
    Viper_Steer(const std::string& name,
                bool fixed,
                std::shared_ptr<ChMaterialSurface> mat,
                ChSystem* system,
                const ChVector<>& body_pos,
                const ChQuaternion<>& body_rot,
                std::shared_ptr<ChBodyAuxRef> chassis,
                bool collide,
                const int& side);  ///< indicate which side of the rover 0->L, 1->R
    ~Viper_Steer() {}

    /// Initialize the steering rod at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the steering rod.
    void SetCollide(bool state);

  private:

    /// Translate the steering rod by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};



/// Viper rover class.
/// This class encapsulates the location and rotation information of all Viper parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API ViperRover {
  public:
    ViperRover(ChSystem* system, 
              const ChVector<>& rover_pos, 
              const ChQuaternion<>& rover_rot, 
              std::shared_ptr<ChMaterialSurface> wheel_mat);
    ViperRover(ChSystem* system, 
              const ChVector<>& rover_pos, 
              const ChQuaternion<>& rover_rot);
    ~ViperRover();

    /// Initialize the Viper rover using current parameters.
    void Initialize();

    /// Get the ChSystem
    ChSystem* GetSystem() { return m_system; }

    /// Set Motor Speed
    void SetMotorSpeed(double rad_speed, WheelID id);

    /// Get wheel speed
    ChVector<> GetWheelSpeed(WheelID id);

    /// Get wheel angular velocity
    ChQuaternion<> GetWheelAngVel(WheelID id);

    /// Get wheel contact force
    ChVector<> GetWheelContactForce(WheelID id);

    /// Get wheel contact torque
    ChVector<> GetWheelContactTorque(WheelID id);

    /// Get wheel total applied force
    ChVector<> GetWheelAppliedForce(WheelID id);

    /// Get wheel total applied torque
    ChVector<> GetWheelAppliedTorque(WheelID id);

    /// Get the chassis body
    std::shared_ptr<ChBodyAuxRef> GetChassisBody();

    /// Get the wheel body
    std::shared_ptr<ChBodyAuxRef> GetWheelBody(WheelID id);


    /// Get the steering body
    std::shared_ptr<ChBodyAuxRef> GetSteeringBody(WheelID id);

    /// Get the upper arm body
    std::shared_ptr<ChBodyAuxRef> GetUpArmBody(WheelID id);

    /// Get the bottom arm body
    std::shared_ptr<ChBodyAuxRef> GetBottomArmBody(WheelID id);

    /// Get total rover mass
    double GetRoverMass();

    /// Get total wheel mass
    double GetWheelMass();

    /// Get main motor function ptr
    std::shared_ptr<ChFunction_Const> GetMainMotorFunc(WheelID id);

    /// Get steer motor function ptr
    std::shared_ptr<ChFunction_Const> GetSteerMotorFunc(WheelID id);

    /// Get main motor link ptr
    std::shared_ptr<ChLinkMotorRotationSpeed> GetMainMotorLink(WheelID id);

    /// Get steer motor link ptr
    std::shared_ptr<ChLinkMotorRotationSpeed> GetSteerMotorLink(WheelID id);

    /// Set viper turning signal left/right/hold
    void SetTurn(TurnSig id, double turn_speed=0.0);

    /// Get viper turning angle - ranges from -CH_C_PI/3 to CH_C_PI/3
    double GetTurnAngle();

    /// Get viper turning state - HOLD, L(EFT), OR R(IGHT)
    TurnSig GetTurnState();

    /// A viper status check and update function
    /// Note: this function needs to be included in the main simulation loop
    void Update();

    
  private:

    /// This function initializes all parameters for the rover
    /// Note: The rover will not be constructed in the ChSystem until Initialize() is called
    void Create();

    ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_custom_wheel_mat;  ///< bool indicating whether the wheel material is customized

    std::shared_ptr<Viper_Chassis> m_chassis;               ///< rover chassis
    std::vector<std::shared_ptr<Viper_Wheel>> m_wheels;     ///< rover wheels - 1:FL, 2:FR, 3:RL, 4:RR
    std::vector<std::shared_ptr<Viper_Up_Arm>> m_up_suss;   ///< rover upper Suspensions - 1:FL, 2:FR, 3:RL, 4:RR
    std::vector<std::shared_ptr<Viper_Bottom_Arm>> m_bts_suss;  ///< rover bottom suspensions - 1:FL, 2:FR, 3:RL, 4:RR
    std::vector<std::shared_ptr<Viper_Steer>> m_steers;     ///< rover steering stand - 1:FL, 2:FR, 3:RL, 4:RR
    std::vector<std::shared_ptr<ChBody>> m_steers_rod;      ///< rover steering rod, no vis - 1:FL, 2:FR, 3:RL, 4:RR

    ChQuaternion<> m_rover_rot;
    ChVector<> m_rover_pos;

    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> m_motors;  ///< vector to store motors
                                                                      ///< 1-LF,2-RF,3-LB,4-RB
    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> m_steer_motors; ///< vector to store steering motors
                                                                          ///< 1-LF,2-RF,3-LB,4-RB
    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> m_lift_motors; ///TODO: < vector to store lifting motors
                                                                          ///< 1-LF,2-RF,3-LB,4-RB

    TurnSig cur_turn_state = TurnSig::HOLD;       ///< Turning state of the viper rover

    std::vector<std::shared_ptr<ChFunction_Const>> m_motors_func;  ///< constant motor angular speed func
    std::vector<std::shared_ptr<ChFunction_Const>> m_steer_motors_func; ///< constant steering motor angular speed func
    std::vector<std::shared_ptr<ChFunction_Const>> m_lift_motors_func; ///TODO: < constant lifting motor angular speed func
    // suspension spring
    std::vector<std::shared_ptr<ChLinkTSDA>> m_sus_springs;  ///< suspension springs

    // model parts material
    std::shared_ptr<ChMaterialSurface> m_chassis_material;  ///< chassis contact material
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)
    std::shared_ptr<ChMaterialSurface> m_steer_material;
    std::shared_ptr<ChMaterialSurface> m_suspension_material;  ///< link contact material (shared across limbs)
};

/// @} robot_models_viper

}  // namespace viper
}  // namespace chrono
#endif
