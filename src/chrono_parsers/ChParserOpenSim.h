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
// Authors: Conlain Kelly
// =============================================================================
//
// Parser utility class for OpenSim input files.
//
// =============================================================================

#ifndef CH_PARSER_OPENSIM_H
#define CH_PARSER_OPENSIM_H

#include <functional>
#include <map>

#include "chrono_parsers/ChApiParsers.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// OpenSim input file parser.
class ChApiParsers ChParserOpenSim {
  public:
    enum VisType { PRIMITIVES, MESH, NONE };

    /// Report containing information about objects parsed from file
    class ChApiParsers Report {
      public:
        /// Information about a joint read in from OpenSim.
        struct JointInfo {
            std::string type;               ///< joint type as shown in osim file
            std::shared_ptr<ChLink> joint;  ///< Chrono link (joint)
            bool standin;                   ///< true if OpenSim joint replaced with spherical
        };

        /// Information about a custom load created from OpenSim.
        struct ForceInfo {
            std::string type;                  ///< load type as shown in osim file
            std::shared_ptr<ChLoadBase> load;  ///< Chrono load object
        };

        std::unordered_map<std::string, std::shared_ptr<ChBodyAuxRef>> bodies;  ///< list of body information
        std::unordered_map<std::string, JointInfo> joints;                      ///< list of joint information
        std::unordered_map<std::string, ForceInfo> forces;                      ///< list of force information

        /// Print information on all modeling elements parsed from osim file.
        void Print() const;

        /// Get a handle to the body with specified name.
        /// If none exists, an empty shared pointer is returned.
        /// Note that all bodies created by the parser are of type ChBodyAuxRef
        /// (i.e., using a non-centroidal reference frame).
        std::shared_ptr<ChBodyAuxRef> GetBody(const std::string& name) const;

        /// Get a handle to the joint with specified name.
        /// If none exists, an empty shared pointer is returned.
        /// The caller may need to downcast to the appropriate type.
        std::shared_ptr<ChLink> GetJoint(const std::string& name) const;

        /// Get a handle to the force element with specified name.
        /// If none exists, an empty shared pointer is returned.
        /// The caller may need to downcast to the appropriate type.
        std::shared_ptr<ChLoadBase> GetForce(const std::string& name) const;
    };

    ChParserOpenSim();
    ~ChParserOpenSim() {}

    /// Set coefficient of friction.
    /// The default value is 0.6
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restitution.
    /// The default value is 0.4
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

    /// Set contact material properties.
    /// These values are used to calculate contact material coefficients (if the containing
    /// system is so configured and if the SMC contact method is being used).
    /// The default values are: Y = 2e5 and nu = 0.3
    void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                      float poisson_ratio   ///< [in] Poisson ratio
    );

    /// Set contact material coefficients.
    /// These values are used directly to compute contact forces (if the containing system
    /// is so configured and if the SMC contact method is being used).
    /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
    void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                        float gn,  ///< [in] normal contact damping
                                        float kt,  ///< [in] tangential contact stiffness
                                        float gt   ///< [in] tangential contact damping
    );

    /// Enable collision between bodies in this model (default: false).
    void SetCollide(bool val) { m_collide = val; }

    /// Set collision families (to disable collision between a body and its parent).
    /// Note: automaticaly enables collision.
    void SetCollisionFamilies(int family_1 = 1,  ///< [in] first collision family
                              int family_2 = 2   ///< [in] second collision family
    );

    /// Set body visualization type (default: NONE).
    void SetVisualizationType(VisType val) { m_visType = val; }

    /// Enable/disable verbose parsing output (default: false).
    void SetVerbose(bool val) { m_verbose = val; }

    /// Activate actuators.
    /// By default, any actuator read in from the osim file is inactive (zero excitation).
    /// If enabled, all actuators will receive a constant excitation function with value 1.
    /// The excitation function for an actuator can be specified, after parsing,
    /// using SetExcitationFunction().
    void ActivateActuators(bool val) { m_activate_actuators = val; }

    /// Parse the specified OpenSim input file and create the model in the given system.
    void Parse(ChSystem& system,            ///< [in] containing Chrono system
               const std::string& filename  ///< [in] OpenSim input file name
    );

    /// Parse the specified OpenSim input file and create the model in a new system.
    /// Note that the created system is not deleted in the parser's destructor;
    /// rather, ownership is transferred to the caller.
    ChSystem* Parse(const std::string& filename,                           ///< [in] OpenSim input file name
                    ChContactMethod contact_method = ChContactMethod::NSC  ///< [in] contact method
    );

    /// Get the report for this parser.
    /// This contains the lists of bodies, joints, and forces that were created from the input osim file.
    const Report& GetReport() const { return m_report; }

    /// Print the parser's report.
    void PrintReport() const { m_report.Print(); }

    /// Set excitation function for the actuator with the specified name.
    /// This method should be invoked only after parsing an osim file.
    void SetExcitationFunction(const std::string& name, std::shared_ptr<ChFunction> modulation);

  private:
    /// Setup lambda table for body parsing
    void initFunctionTable();
    Report m_report;

    /// Creates load object and parses its various properties from its XML child nodes
    bool parseForce(rapidxml::xml_node<>* bodyNode, ChSystem& system, std::shared_ptr<ChLoadContainer> container);

    /// Creates body and parses its various properties from its XML child nodes
    bool parseBody(rapidxml::xml_node<>* bodyNode, ChSystem& system);

    // Initializes visualization shapes for bodies connected to each link
    void initShapes(rapidxml::xml_node<>* node, ChSystem& system);

    // Get an STL vector from a string, used to make the xml parsing cleaner
    template <typename T>
    static inline std::vector<T> strToSTLVector(const char* string) {
        std::istringstream buf(string);
        std::istream_iterator<T> beg(buf), end;
        return std::vector<T>(beg, end);
    }

    // Convert a space-delimited string into a ChVector
    template <typename T>
    static inline ChVector<T> strToChVector(const char* string) {
        auto elems = strToSTLVector<T>(string);
        return ChVector<T>(elems.at(0), elems.at(1), elems.at(2));
    }

    static inline std::string stringStripCStr(const char* c_str) {
        std::string str(c_str);
        str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
        return str;
    }

    // Convert a lowercase string to a boolean
    static inline bool CStrToBool(const char* string) { return stringStripCStr(string) == std::string("true"); }

    // Maps child fields of a body node to functions that handle said fields
    std::map<std::string, std::function<void(rapidxml::xml_node<>*, std::shared_ptr<ChBodyAuxRef>)>> function_table;

    bool m_verbose;             ///< verbose output
    VisType m_visType;          ///< body visualization type
    bool m_collide;             ///< do bodies have collision shapes?
    int m_family_1;             ///< first collision family
    int m_family_2;             ///< second collision family
    bool m_activate_actuators;  ///< are actuators activated at construction?

    float m_friction;        ///< contact coefficient of friction
    float m_restitution;     ///< contact coefficient of restitution
    float m_young_modulus;   ///< contact material Young modulus
    float m_poisson_ratio;   ///< contact material Poisson ratio
    float m_kn;              ///< normal contact stiffness
    float m_gn;              ///< normal contact damping
    float m_kt;              ///< tangential contact stiffness
    float m_gt;              ///< tangential contact damping
    std::string m_datapath;  ///< path to find visualization meshes, default is "opensim"

    // List of joints in model (loaded in order of outward, base-to-tip, traversal)
    std::vector<std::shared_ptr<ChLink>> m_jointList;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
