// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_YAML_PARSER_H
#define CH_YAML_PARSER_H

#include <string>
#include <vector>
#include <unordered_map>

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"

#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Utility class to parse a YAML specification of a Chrono model and populate a Chrono system.
class ChApi ChYamlParser {
  public:
    ChYamlParser();
    ChYamlParser(const std::string& yaml_filename);
    ~ChYamlParser();

    void SetVerbose(bool verbose) { m_verbose = verbose; }

    void Load(const std::string& yaml_filename);
    void Populate(ChSystem& sys);
    void Depopulate(ChSystem& sys);

  public:
    /// Load and return a ChVector3d from the specified node.
    ChVector3d ReadVector(const YAML::Node& a);

    ///  Load and return a ChQuaternion from the specified node.
    ChQuaterniond ReadQuaternion(const YAML::Node& a);

    /// Load a Cardan angle sequence from the specified node and return as a quaternion.
    /// The sequence is assumed to be extrinsic rotations X-Y-Z.
    ChQuaterniond ReadCardanAngles(const YAML::Node& a);

    /// Return a quaternion loaded from the specified node.
    /// Data is assumed to provide a quaternion or a Cardan extrinsic X-Y-Z angle set.
    ChQuaterniond ReadRotation(const YAML::Node& a);
    
    /// Load and return a coordinate system from the specified node.
    ChCoordsysd ReadCoordinateSystem(const YAML::Node& a);

    ///  Load and return a ChColor from the specified node.
    ChColor ReadColor(const YAML::Node& a);

    ChContactMaterialData ReadMaterialData(const YAML::Node& mat);

    /// Load and return bushing data from the specified node.
    std::shared_ptr<ChJoint::BushingData> ReadBushingData(const YAML::Node& bd);

    /// Load and return a joint type from the specified node.
    ChJoint::Type ReadJointType(const YAML::Node& a);

    /// Load and return joint coordinate syustem from the specified node.
    ChCoordsysd ReadJointFrame(const YAML::Node& a);

    /// Load and return a geometry structure from the specified node.
    /// Collision geometry and contact material information is set in the return ChBodyGeometry object if the given
    /// object has a member "Contact". Visualization geometry is loaded if the object has a member "Visualization".
    ChBodyGeometry ReadGeometry(const YAML::Node& d);

    /// Load and return a TSDA geometry structure from the specified node.
    ChTSDAGeometry ReadTSDAGeometry(const YAML::Node& d);

    /// Load and return a TSDA functor object from the specified node.
    /// The TSDA free length is also set if the particular functor type defines it.
    std::shared_ptr<ChLinkTSDA::ForceFunctor> ReadTSDAFunctor(const YAML::Node& td, double& free_length);

    /// Load and return an RSDA functor object.
    /// The TSDA free angle is also set if the particular functor type defines it.
    std::shared_ptr<ChLinkRSDA::TorqueFunctor> ReadRSDAFunctor(const YAML::Node& td, double& free_angle);

  private:
    /// Internal specification of a suspension body.
    struct Body {
        Body();
        void PrintInfo(const std::string& name);

        std::shared_ptr<ChBodyAuxRef> body;  ///< underlying Chrono body
        ChVector3d pos;                      ///< body position (in global frame)
        ChQuaterniond rot;                   ///< body orientation (in global frame)
        bool is_fixed;                       ///< indicate if body fixed relative to global frame
        double mass;                         ///< body mass
        ChFramed com;                        ///< centroidal frame (relative to body frame)
        ChVector3d inertia_moments;          ///< moments of inertia (relative to centroidal frame)
        ChVector3d inertia_products;         ///< products of inertia (relative to centroidal frame)
        ChBodyGeometry geometry;             ///< visualization and collision geometry
    };

    /// Internal specification of a suspension joint.
    struct Joint {
        Joint();
        void PrintInfo(const std::string& name);

        std::shared_ptr<ChJoint> joint;               ///< underlying Chrono joint
        ChJoint::Type type;                           ///< joint type
        std::string body1;                            ///< identifier of 1st body
        std::string body2;                            ///< identifier of 2nd body
        ChCoordsysd csys;                             ///< joint position and orientation (in global frame)
        std::shared_ptr<ChJoint::BushingData> bdata;  ///< bushing data
    };

    /// Internal specification of a distance constraint.
    struct DistanceConstraint {
        DistanceConstraint();
        void PrintInfo(const std::string& name);

        std::shared_ptr<ChLinkDistance> dist;  ///< underlying Chrono distance constraint
        std::string body1;                     ///< identifier of 1st body
        std::string body2;                     ///< identifier of 2nd body
        ChVector3d point1;                     ///< point on body1 (in global frame)
        ChVector3d point2;                     ///< point on body2 (in global frame)
    };

    /// Internal specification of a suspension TSDA.
    struct TSDA {
        TSDA();
        void PrintInfo(const std::string& name);

        std::shared_ptr<ChLinkTSDA> tsda;                 ///< underlying Chrono TSDA element
        std::string body1;                                ///< identifier of 1st body
        std::string body2;                                ///< identifier of 2nd body
        ChVector3d point1;                                ///< point on body1 (in global frame)
        ChVector3d point2;                                ///< point on body2 (in global frame)
        double free_length;                               ///< TSDA free (rest) length
        std::shared_ptr<ChLinkTSDA::ForceFunctor> force;  ///< force functor
        ChTSDAGeometry geometry;                          ///< (optional) visualization geometry
    };

    /// Internal specification of a suspension RSDA.
    struct RSDA {
        RSDA();
        void PrintInfo(const std::string& name);

        std::shared_ptr<ChLinkRSDA> rsda;                   ///< underlying Chrono RSDA element
        std::string body1;                                  ///< identifier of 1st body
        std::string body2;                                  ///< identifier of 2nd body
        ChVector3d pos;                                     ///< RSDA position (in global frame, visualization)
        ChVector3d axis;                                    ///< axis of action for the RSDA (in global frame)
        double free_angle;                                  ///< RSDA free (rest) angle
        std::shared_ptr<ChLinkRSDA::TorqueFunctor> torque;  ///< torque functor
    };

    std::shared_ptr<ChBodyAuxRef> ChYamlParser::FindBody(const std::string& name) const;

    std::unordered_map<std::string, Body> m_bodies;               ///< bodies
    std::unordered_map<std::string, Joint> m_joints;              ///< joints
    std::unordered_map<std::string, TSDA> m_tsdas;                ///< TSDA force elements
    std::unordered_map<std::string, RSDA> m_rsdas;                ///< RSDA force elements
    std::unordered_map<std::string, DistanceConstraint> m_dists;  ///< distance constraints

    ChSystem* m_sys;
    bool m_initialized;  ///< YAML file loaded
    bool m_verbose;      ///< verbose terminal output (default: false)
    bool m_use_degrees;  ///< all angles given in degrees (default: true)
};

/// @} chrono_utils

}  // namespace utils
}  // namespace chrono

#endif
