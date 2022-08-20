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
// Turtlebot Robot Class
// This is a modified version of the famous turtlebot 2e
// The geometries use the following resources as references:
// https://groups.google.com/g/sydney_ros/c/z05uQTCuDTQ
// https://grabcad.com/library/interbotix-turtlebot-2i-1
// https://www.turtlebot.com/turtlebot2/
//
// =============================================================================

#ifndef TURTLEBOT_H
#define TURTLEBOT_H

#include <array>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {

/// Namespace with classes for the Turtlebot model.
namespace turtlebot {

/// @addtogroup robot_models_turtlebot
/// @{

/// Turtlebot collision families.
namespace CollisionFamily {
enum Enum {
    CHASSIS = 1,        ///< chassis
    ACTIVE_WHEEL = 2,   ///< active cylinderical drive wheel
    PASSIVE_WHEEL = 3,  ///< passive cylinderical wheel
    ROD = 4,            ///< short and long supporting rods
    BOTTOM_PLATE = 5,   ///< bottom plate
    MIDDLE_PLATE = 6,   ///< middle plate
    TOP_PLATE = 7       ///< top plate
};
}

/// TurtleBot wheel identifiers.
enum WheelID {
    LD,  ///< left driven
    RD,  ///< right driven
};

/// Base class definition of the Turtlebot Robot Part.
/// This class encapsulates base fields and functions.
class CH_MODELS_API Turtlebot_Part {
  public:
    Turtlebot_Part(const std::string& name,
                   bool fixed,
                   std::shared_ptr<ChMaterialSurface> mat,
                   ChSystem* system,
                   const ChVector<>& body_pos,
                   const ChQuaternion<>& body_rot,
                   std::shared_ptr<ChBodyAuxRef> chassis_body,
                   bool collide);
    virtual ~Turtlebot_Part() {}

    /// Return the name of the part.
    const std::string& GetName() const { return m_name; }

    /// Set the name of the part.
    void SetName(const std::string& name) { m_name = name; }

    /// Return the ChBody of the corresponding Turtlebot part.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Return the ChBody of the chassis wrt the Turtlebot part.
    std::shared_ptr<ChBodyAuxRef> GetChassis() const { return m_chassis; }

    /// Return the Position of the Turtlebot part.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Return the Rotation of the Turtlebot part.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

  protected:
    /// Initialize the visulization mesh of the Turtlebot part.
    void AddVisualizationAssets();

    /// Initialize the collision mesh of the Turtlebot part.
    void AddCollisionShapes();

    /// Enable/disable collision.
    void SetCollide(bool state);

    std::string m_name;                        ///< subsystem name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material (shared among all shapes)

    std::string m_mesh_name;                  ///< visualization mesh name
    ChVector<> m_offset;                      ///< offset for visualization mesh
    ChColor m_color;                          ///< visualization asset color
    ChSystem* m_system;                       ///< system which Turtlebot Part belongs to
    std::shared_ptr<ChBodyAuxRef> m_chassis;  ///< the chassis body for the robot

    ChVector<> m_pos;      ///< Turtlebot part's relative position wrt the chassis
    ChQuaternion<> m_rot;  ///< Turtlebot part's relative rotation wrt the chassis
    double m_density;      ///< Turtlebot part's density

    bool m_collide;  ///< Turtlebot part's collision indicator
    bool m_fixed;    ///< Turtlebot part's fixed indication
};

/// Turtlebot Chassis class definition
class CH_MODELS_API Turtlebot_Chassis : public Turtlebot_Part {
  public:
    Turtlebot_Chassis(const std::string& name,
                      bool fixed,
                      std::shared_ptr<ChMaterialSurface> mat,
                      ChSystem* system,
                      const ChVector<>& body_pos,
                      const ChQuaternion<>& body_rot,
                      bool collide);
    ~Turtlebot_Chassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the robot chassis.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Turtlebot Active Drive Wheel class definition
class CH_MODELS_API Turtlebot_ActiveWheel : public Turtlebot_Part {
  public:
    Turtlebot_ActiveWheel(const std::string& name,
                          bool fixed,
                          std::shared_ptr<ChMaterialSurface> mat,
                          ChSystem* system,
                          const ChVector<>& body_pos,
                          const ChQuaternion<>& body_rot,
                          std::shared_ptr<ChBodyAuxRef> chassis,
                          bool collide);
    ~Turtlebot_ActiveWheel() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Turtlebot Passive Driven Wheel class definition
class CH_MODELS_API Turtlebot_PassiveWheel : public Turtlebot_Part {
  public:
    Turtlebot_PassiveWheel(const std::string& name,
                           bool fixed,
                           std::shared_ptr<ChMaterialSurface> mat,
                           ChSystem* system,
                           const ChVector<>& body_pos,
                           const ChQuaternion<>& body_rot,
                           std::shared_ptr<ChBodyAuxRef> chassis,
                           bool collide);
    ~Turtlebot_PassiveWheel() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Short Supporting Rod class definition
class CH_MODELS_API Turtlebot_Rod_Short : public Turtlebot_Part {
  public:
    Turtlebot_Rod_Short(const std::string& name,
                        bool fixed,
                        std::shared_ptr<ChMaterialSurface> mat,
                        ChSystem* system,
                        const ChVector<>& body_pos,
                        const ChQuaternion<>& body_rot,
                        std::shared_ptr<ChBodyAuxRef> chassis,
                        bool collide);
    ~Turtlebot_Rod_Short() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Turtlebot Bottom Plate class definition
class CH_MODELS_API Turtlebot_BottomPlate : public Turtlebot_Part {
  public:
    Turtlebot_BottomPlate(const std::string& name,
                          bool fixed,
                          std::shared_ptr<ChMaterialSurface> mat,
                          ChSystem* system,
                          const ChVector<>& body_pos,
                          const ChQuaternion<>& body_rot,
                          std::shared_ptr<ChBodyAuxRef> chassis,
                          bool collide);
    ~Turtlebot_BottomPlate() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Turtlebot Middle Plate class definition
class CH_MODELS_API Turtlebot_MiddlePlate : public Turtlebot_Part {
  public:
    Turtlebot_MiddlePlate(const std::string& name,
                          bool fixed,
                          std::shared_ptr<ChMaterialSurface> mat,
                          ChSystem* system,
                          const ChVector<>& body_pos,
                          const ChQuaternion<>& body_rot,
                          std::shared_ptr<ChBodyAuxRef> chassis,
                          bool collide);
    ~Turtlebot_MiddlePlate() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Turtlebot Top Plate class definition
class CH_MODELS_API Turtlebot_TopPlate : public Turtlebot_Part {
  public:
    Turtlebot_TopPlate(const std::string& name,
                       bool fixed,
                       std::shared_ptr<ChMaterialSurface> mat,
                       ChSystem* system,
                       const ChVector<>& body_pos,
                       const ChQuaternion<>& body_rot,
                       std::shared_ptr<ChBodyAuxRef> chassis,
                       bool collide);
    ~Turtlebot_TopPlate() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Long Supporting Rod class definition
class CH_MODELS_API Turtlebot_Rod_Long : public Turtlebot_Part {
  public:
    Turtlebot_Rod_Long(const std::string& name,
                       bool fixed,
                       std::shared_ptr<ChMaterialSurface> mat,
                       ChSystem* system,
                       const ChVector<>& body_pos,
                       const ChQuaternion<>& body_rot,
                       std::shared_ptr<ChBodyAuxRef> chassis,
                       bool collide);
    ~Turtlebot_Rod_Long() {}

    /// Initialize the wheel at the specified (absolute) position.
    void Initialize();

    /// Enable/disable collision for the wheel.
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class TurtleBot;
};

/// Turtlebot Robot class
/// This class assemble and initialize a complete turtlebot robot
/// This class also handles general control commands of the robot
class CH_MODELS_API TurtleBot {
  public:
    TurtleBot(ChSystem* system,
              const ChVector<>& robot_pos,
              const ChQuaternion<>& robot_rot,
              std::shared_ptr<ChMaterialSurface> wheel_mat = nullptr);
    ~TurtleBot();

    /// Initialize the turtlebot robot using current parameters.
    void Initialize();

    /// Set active drive wheel speed
    void SetMotorSpeed(double rad_speed, WheelID id);

    /// Get active drive wheel speed
    ChVector<> GetActiveWheelSpeed(WheelID id);

    /// Get active driver wheel angular velocity
    ChVector<> GetActiveWheelAngVel(WheelID id);

  private:
    /// This function initializes all parameters for the robot.
    /// Note: The robot will not be constructed in the ChSystem until Initialize() is called.
    void Create();

    ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_dc_motor_control = false;

    std::shared_ptr<Turtlebot_Chassis> m_chassis;                           ///< robot chassis
    std::vector<std::shared_ptr<Turtlebot_ActiveWheel>> m_drive_wheels;     ///< 2 active robot drive wheels
    std::vector<std::shared_ptr<Turtlebot_PassiveWheel>> m_passive_wheels;  ///< 2 passive robot driven wheels

    std::vector<std::shared_ptr<Turtlebot_Rod_Short>> m_1st_level_rods;  ///< six first level supporting short rods
    std::vector<std::shared_ptr<Turtlebot_Rod_Short>> m_2nd_level_rods;  ///< six second level supporting short rods
    std::vector<std::shared_ptr<Turtlebot_Rod_Long>> m_3rd_level_rods;   ///< six third level support long rods
    std::shared_ptr<Turtlebot_BottomPlate> m_bottom_plate;               ///< bottom plate of the turtlebot robot
    std::shared_ptr<Turtlebot_MiddlePlate> m_middle_plate;               ///< middle plate of the turtlebot robot
    std::shared_ptr<Turtlebot_TopPlate> m_top_plate;                     ///< top plate of the turtlebot robot

    ChQuaternion<> m_robot_rot;  ///< robot rotation
    ChVector<> m_robot_pos;      ///< robot translation position

    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> m_motors;  ///< vector to store motors

    std::vector<std::shared_ptr<ChFunction_Const>> m_motors_func;  ///< constant motor angular speed func

    // model parts material
    std::shared_ptr<ChMaterialSurface> m_chassis_material;  ///< chassis contact material
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)
};

}  // namespace turtlebot
}  // namespace chrono
#endif
