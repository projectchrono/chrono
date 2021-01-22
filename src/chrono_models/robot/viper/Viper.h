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
// Authors: Jason Zhou
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

/*
/// Visualization type for a Viper part.
enum class VisualizationType {
    NONE,       ///< no visualization
    COLLISION,  ///< render primitive collision shapes
    MESH        ///< render meshes
};
*/

/// RoboSimian collision families.
namespace CollisionFamily {
enum Enum {
    CHASSIS = 1,  ///< front-right limb
    FL_SUSPENSION = 2,
    FR_SUSPENSION = 3,
    RL_SUSPENSION = 4,
    RR_SUSPENSION = 5,
    WHEEL = 6
};
}

enum class SideNo { LF, RF, LB, RB };

// ==================================================================================
/// Viper Rover Part
class CH_MODELS_API Viper_Part {
  public:
    Viper_Part(const std::string& name,
               bool fixed,
               std::shared_ptr<ChMaterialSurface> mat,
               ChSystem* system,
               ChVector<> body_pos,
               ChQuaternion<> body_rot,
               std::shared_ptr<ChBodyAuxRef> chassis_body,
               bool collide);
    virtual ~Viper_Part() {}

    const std::string& GetName() const { return m_name; }
    void SetName(const std::string& name) { m_name = name; }
    // void SetVisualizationType(VisualizationType vis);

    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }
    std::shared_ptr<ChBodyAuxRef> GetChassis() const { return m_chassis; }
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

  protected:
    void AddVisualizationAssets();
    void AddCollisionShapes();
    void SetCollide(bool state);

    std::string m_name;                        ///< subsystem name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material (shared among all shapes)
    // std::vector<BoxShape> m_boxes;                 ///< set of collision boxes
    // std::vector<SphereShape> m_spheres;            ///< set of collision spheres
    // std::vector<CylinderShape> m_cylinders;        ///< set of collision cylinders
    // std::vector<MeshShape> m_meshes;               ///< set of collision meshes
    std::string m_mesh_name;                  ///< visualization mesh name
    ChVector<> m_offset;                      ///< offset for visualization mesh
    ChColor m_color;                          ///< visualization asset color
    ChSystem* m_system;                       ///< system which Viper Part belongs to
    std::shared_ptr<ChBodyAuxRef> m_chassis;  ///< the chassis body for the rover

    ChVector<> m_pos;      ///< Viper part's relative position wrt the chassis
    ChQuaternion<> m_rot;  ///< Viper part's relative rotation wrt the chassis
    double m_density;      ///< Viper part's density

    bool m_collide;
    bool m_fixed;
};

// ==========================================================================
/// Viper Rover Part: Chassis
class CH_MODELS_API Viper_Chassis : public Viper_Part {
  public:
    Viper_Chassis(const std::string& name,
                  bool fixed,
                  std::shared_ptr<ChMaterialSurface> mat,
                  ChSystem* system,
                  ChVector<> body_pos,
                  ChQuaternion<> body_rot,
                  bool collide);
    ~Viper_Chassis() {}
    /// Initialize the chassis at the specified (absolute) position.
    void Initialize();
    /// Enable/disable collision for the sled (Default: false).
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

// ===========================================================
/// Viper Rover Part: Wheel
class CH_MODELS_API Viper_Wheel : public Viper_Part {
  public:
    Viper_Wheel(const std::string& name,
                bool fixed,
                std::shared_ptr<ChMaterialSurface> mat,
                ChSystem* system,
                ChVector<> body_pos,
                ChQuaternion<> body_rot,
                std::shared_ptr<ChBodyAuxRef> chassis,
                bool collide);
    ~Viper_Wheel() {}
    /// Initialize the chassis at the specified (absolute) position.
    void Initialize();
    /// Enable/disable collision for the sled (Default: false).
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

// ===========================================================
/// Viper Rover Part: Upper Suspension
class CH_MODELS_API Viper_Up_Sus : public Viper_Part {
  public:
    Viper_Up_Sus(const std::string& name,
                 bool fixed,
                 std::shared_ptr<ChMaterialSurface> mat,
                 ChSystem* system,
                 ChVector<> body_pos,
                 ChQuaternion<> body_rot,
                 std::shared_ptr<ChBodyAuxRef> chassis,
                 bool collide,
                 int side);  ///< indicate which side of the suspension 0->L, 1->R
    ~Viper_Up_Sus() {}
    /// Initialize the chassis at the specified (absolute) position.
    void Initialize();
    /// Enable/disable collision for the sled (Default: false).
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

// ===========================================================
/// Viper Rover Part: Bottom Suspension
class CH_MODELS_API Viper_Bt_Sus : public Viper_Part {
  public:
    Viper_Bt_Sus(const std::string& name,
                 bool fixed,
                 std::shared_ptr<ChMaterialSurface> mat,
                 ChSystem* system,
                 ChVector<> body_pos,
                 ChQuaternion<> body_rot,
                 std::shared_ptr<ChBodyAuxRef> chassis,
                 bool collide,
                 int side);  ///< indicate which side of the suspension 0->L, 1->R
    ~Viper_Bt_Sus() {}
    /// Initialize the chassis at the specified (absolute) position.
    void Initialize();
    /// Enable/disable collision for the sled (Default: false).
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

// ===========================================================
/// Viper Rover Part: Steering Rod
class CH_MODELS_API Viper_Steer : public Viper_Part {
  public:
    Viper_Steer(const std::string& name,
                bool fixed,
                std::shared_ptr<ChMaterialSurface> mat,
                ChSystem* system,
                ChVector<> body_pos,
                ChQuaternion<> body_rot,
                std::shared_ptr<ChBodyAuxRef> chassis,
                bool collide,
                int side);  ///< indicate which side of the suspension 0->L, 1->R
    ~Viper_Steer() {}
    /// Initialize the chassis at the specified (absolute) position.
    void Initialize();
    /// Enable/disable collision for the sled (Default: false).
    void SetCollide(bool state);

  private:
    /// Translate the chassis by the specified value.
    void Translate(const ChVector<>& shift);
    friend class Viper_Rover;
};

// ===========================================================
/// Viper Rover model.
class CH_MODELS_API ViperRover {
  public:
    ViperRover(ChSystem* system, ChVector<> rover_pos, ChQuaternion<> rover_rot);
    ~ViperRover();
    void Initialize();

    // Get the pointer for the ChSystem
    ChSystem* GetSystem() { return m_system; }

    // Set one Motor Speed
    void SetMotorSpeed(double rad_speed, SideNo motor_num);

    // Get one wheel spped
    ChVector<> GetWheelSpeed(SideNo motor_num);

    // Get one wheel angular velocity
    ChQuaternion<> GetWheelAngVel(SideNo motor_num);

    // Get one wheel contact force
    ChVector<> GetWheelContactForce(SideNo motor_num);

    // Get one wheel contact torque
    ChVector<> GetWheelContactTorque(SideNo motor_num);

    // Get one wheel applied force
    ChVector<> GetWheelAppliedForce(SideNo motor_num);

    // Get one wheel applied torque
    ChVector<> GetWheelAppliedTorque(SideNo motor_num);

    // Get the chassis body
    std::shared_ptr<ChBodyAuxRef> GetChassisBody();

    // Get the pointer of wheel body
    std::shared_ptr<ChBodyAuxRef> GetWheelBody(SideNo motor_num);

    // Get total rover mass
    double GetRoverMass();

    // Get total wheel mass
    double GetWheelMass();

  private:
    void Create();

    ChSystem* m_system;  ///< pointer to the Chrono system

    std::shared_ptr<Viper_Chassis> m_chassis;               ///< rover chassis
    std::vector<std::shared_ptr<Viper_Wheel>> m_wheels;     ///< rover wheels - 1:FL, 2:FR, 3:RL, 4:RR
    std::vector<std::shared_ptr<Viper_Up_Sus>> m_up_suss;   ///< rover upper Suspensions - 1:FL, 2:FR, 3:RL, 4:RR
    std::vector<std::shared_ptr<Viper_Bt_Sus>> m_bts_suss;  ///< rover bottom suspensions - 1:FL, 2:FR, 3:RL, 4:RR
    std::vector<std::shared_ptr<Viper_Steer>> m_steers;     ///< rover bottom suspensions - 1:FL, 2:FR, 3:RL, 4:RR

    ChQuaternion<> m_rover_rot;
    ChVector<> m_rover_pos;

    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> m_motors;  ///< vector to store motors
                                                                      ///< 1-LF,2-RF,3-LB,4-RB

    std::vector<std::shared_ptr<ChFunction_Const>> m_motors_func;  ///< constant motor angular speed func

    // suspension spring
    std::vector<std::shared_ptr<ChLinkTSDA>> m_sus_springs;  ///< suspension springs

    // model parts material
    std::shared_ptr<ChMaterialSurface> m_chassis_material;  ///< chassis contact material
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)
    std::shared_ptr<ChMaterialSurface> m_steer_material;
    std::shared_ptr<ChMaterialSurface> m_suspension_material;  ///< link contact material (shared across limbs)
};

}  // namespace viper
}  // namespace chrono
#endif
