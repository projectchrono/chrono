// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Parser utility class for URDF input files.
//
// =============================================================================

#ifndef CH_PARSER_URDF_H
#define CH_PARSER_URDF_H

#include "chrono_parsers/ChApiParsers.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/physics/ChLinkMotor.h"
#include "chrono/physics/ChMaterialSurface.h"

#include <urdf_parser/urdf_parser.h>

#include <tinyxml2.h>

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// URDF input file parser.
class ChApiParsers ChParserURDF {
  public:
    /// Motor actuation types.
    enum class ActuationType {
        POSITION,  ///< position (if linear motor) or angle (if rotation motor)
        SPEED,     ///< linear speed (if linear motor) or angular speed (if rotation motor)
        FORCE      ///< force (if linear motor) or torque (if rotation motor)
    };

    /// Collision type for meshes.
    enum class MeshCollisionType {
        TRIANGLE_MESH,  ///< use the triangular mesh itself
        CONVEX_HULL,    ///< use the convex hull of the trimesh
        NODE_CLOUD      ///< use spheres at mesh vertices
    };

    /// Construct a Chrono parser for the specified URDF file.
    ChParserURDF(const std::string& filename);

    /// Get access to the raw XML string in the provided URDF file.
    const std::string& GetXMLstring() const { return m_xml_string; }

    /// Get the URDF model name.
    const std::string& GetModelName() const { return m_model->getName(); }

    /// Get the URDF model tree.
    const urdf::ModelInterfaceSharedPtr& GetModelTree() const { return m_model; }

    /// Print the body tree from parsed URDF file.
    void PrintModelBodyTree();

    /// Print the list of bodies from parsed URDF file.
    void PrintModelBodies();

    /// Print the list of joints from parsed URDF file.
    void PrintModelJoints();

    /// Set the initial pose of the model (root link).
    void SetRootInitPose(const ChFrame<>& init_pose);

    /// Set the specified joint as actuated, using the specified actuation type.
    /// By default, all URDF joints are translated into Chrono kinematic joints. Joints marked as actuated will be
    /// translated to Chrono motors (with same kinematics as the corresponding passive joint). This function has no
    /// effect for joints other than Revolute, Continuous, or Prismatic.
    void SetJointActuationType(const std::string& joint_name, ActuationType actuation_type);

    /// Set all candidate joints in the URDF model as actuated, using the specified actuation type.
    /// This function has no effect for joints other than Revolute, Continuous, or Prismatic.
    void SetAllJointsActuationType(ActuationType actuation_type);

    /// Set the collision type for mesh collision (default: TRIMESH).
    /// This is interpreted only if the specified body has a mesh collision shape.
    void SetBodyMeshCollisionType(const std::string& body_name, MeshCollisionType collision_type);

    /// Set the collision type for all bodies with mesh collision shapes (default: TRIMESH).
    void SetAllBodiesMeshCollisinoType(MeshCollisionType collision_type);

    /// Set default contact material properties.
    /// All bodies for which SetBodyContactMaterial was not explicitly called will be constructed with this contact
    /// material for all their collision shapes.
    void SetDefaultContactMaterial(const ChContactMaterialData& mat_data);

    /// Set contact material properties for the specified body.
    /// Bodies for which this function is not explictly called are constructed with the default contact material.
    void SetBodyContactMaterial(const std::string& body_name, const ChContactMaterialData& mat_data);

    /// Enable visualization of collision shapes (default: visualization shapes).
    void EnableCollisionVisualization();

    /// Create the Chrono model in the given system from the parsed URDF model.
    void PopulateSystem(ChSystem& sys);

    /// Get the root body of the Chrono model.
    /// This function must be called after PopulateSystem.
    std::shared_ptr<ChBodyAuxRef> GetRootChBody() const;

    /// Get the body with specified name in the Chrono model.
    /// This function must be called after PopulateSystem.
    std::shared_ptr<ChBody> GetChBody(const std::string& name) const;

    /// Get the joint with specified name in the Chrono model.
    /// This function must be called after PopulateSystem.
    std::shared_ptr<ChLinkBase> GetChLink(const std::string& name) const;

    /// Get the motor with specified name in the Chrono model.
    /// This function must be called after PopulateSystem.
    std::shared_ptr<ChLinkMotor> GetChMotor(const std::string& name) const;

    /// Set the actuation function for the specified Chrono motor.
    /// The return value of this function has different meaning, depending on the type of motor, and can represent a
    /// position, angle, linear speed, angular speed, force, or torque.
    /// No action is taken if a joint with the specified does not exist or if it was not marked as actuated.
    /// This function must be called after PopulateSystem.
    void SetMotorFunction(const std::string& motor_name, const std::shared_ptr<ChFunction> function);

    /// Class to be used as a callback interface for custom processing of the URDF XML string.
    class ChApiParsers CustomProcessor {
      public:
        virtual ~CustomProcessor() {}

        /// Callback function for user processing of specific XML elements.
        virtual void Process(const tinyxml2::XMLElement& element, ChSystem& system) = 0;
    };

    /// Scan the URDF XML for all objects of the specified key and execute the Process() function of the provided callback object.
    /// Only direct children of the "robot" element in the input URDF are processed. 
    void CustomProcess(const std::string& key, std::shared_ptr<CustomProcessor> callback);

  private:
    ChColor toChColor(const urdf::Color& color);
    ChVector<> toChVector(const urdf::Vector3& vec);
    ChQuaternion<> toChQuaternion(const urdf::Rotation& rot);
    ChFrame<> toChFrame(const urdf::Pose& pose);
    std::shared_ptr<ChVisualShape> toChVisualShape(const urdf::GeometrySharedPtr geometry);
    std::shared_ptr<ChBodyAuxRef> toChBody(urdf::LinkConstSharedPtr link);
    std::shared_ptr<ChLink> toChLink(urdf::JointSharedPtr& joint);

    /// Recursively create bodies and joints in the Chrono model.
    void createChildren(urdf::LinkConstSharedPtr parent, const ChFrame<>& parent_frame);

    /// Attach visualization assets to a Chrono body.
    void attachVisualization(std::shared_ptr<ChBody> body, urdf::LinkConstSharedPtr link, const ChFrame<>& ref_frame);

    /// Attach collision assets to a Chrono body.
    void attachCollision(std::shared_ptr<ChBody> body, urdf::LinkConstSharedPtr link, const ChFrame<>& ref_frame);

    std::string m_filename;                                   ///< URDF file name
    std::string m_filepath;                                   ///< path of URDF file
    std::string m_xml_string;                                 ///< raw model XML string
    urdf::ModelInterfaceSharedPtr m_model;                    ///< parsed URDF model
    ChSystem* m_sys;                                          ///< containing Chrono system
    ChFrame<> m_init_pose;                                    ///< root body initial pose
    bool m_vis_collision;                                     ///< visualize collision shapes
    std::shared_ptr<ChBodyAuxRef> m_root_body;                ///< model root body
    std::map<std::string, std::string> m_discarded;           ///< discarded bodies
    std::map<std::string, ChContactMaterialData> m_mat_data;  ///< body contact material data
    std::map<std::string, MeshCollisionType> m_coll_type;     ///< mesh collision type
    std::map<std::string, ActuationType> m_actuated_joints;   ///< actuated joints
    ChContactMaterialData m_default_mat_data;                 ///< default contact material data
};

/// @} parsers_module

}  // end namespace parsers
}  // end namespace chrono

#endif
