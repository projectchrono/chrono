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
#include <tuple>

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

namespace chrono {

namespace utils {

/// @addtogroup chrono_utils
/// @{

/// OpenSim input file parser
class ChApi ChParserOpenSim {
  public:
    enum VisType { PRIMITIVES, MESH, NONE };

    class ChApi ChOpenSimReport {
      public:
        // List of ChBodies read in
        std::vector<std::shared_ptr<ChBody>> bodiesList;
        // List of joints read in, their OpenSim, and Chrono representations
        // Tuples are gross
        std::vector<std::tuple<std::shared_ptr<ChLink>, std::string, std::string>> jointsList;
        void Print() {
            printf("Parsed %u bodies:\n", bodiesList.size());
            for (auto body : bodiesList) {
                std::cout << body->GetNameString() << std::endl;
            }
            printf("Parsed %u bodies:\n", jointsList.size());
            for (auto joint_tuple : jointsList) {
                std::string JointName = std::get<0>(joint_tuple)->GetNameString(),
                            OpenSimType = std::get<1>(joint_tuple), ChronoType = std::get<2>(joint_tuple);
                printf("Joint %25s: OpenSim: %20s, Chrono: %20s\r\n", JointName.c_str(), OpenSimType.c_str(),
                       ChronoType.c_str());
            }
        }
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
    /// Set collision families (to disable collision between a body and its parent).
    void EnableCollision(int family_1 = 1,  ///< [in] First collision family
                         int family_2 = 2   ///< [in] Second collision family
    );

    /// Set body visualization type (default: NONE).
    void SetVisualizationType(VisType val) { m_visType = val; }

    /// Enable/disable verbose parsing output (default: false).
    void SetVerbose(bool val) { m_verbose = val; }

    /// Parse the specified OpenSim input file and create the model in the given system.
    void Parse(ChSystem& system,            ///< [in] containing Chrono system
               const std::string& filename  ///< [in] OpenSim input file name
    );

    /// Parse the specified OpenSim input file and create the model in a new system.
    /// Note that the created system is not deleted in the parser's destructor;
    /// rather, ownership is transferred to the caller.
    ChSystem* Parse(const std::string& filename,  ///< [in] OpenSim input file name
                    ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< [in] contact method
    );

    /// Get the list of bodies in the model.
    const std::vector<std::shared_ptr<ChBodyAuxRef>>& GetBodyList() const { return m_bodyList; }

    /// Get the list of joints in the model.
    const std::vector<std::shared_ptr<ChLink>>& GetJointList() const { return m_jointList; }

    const ChOpenSimReport& GetReport() { return m_report; }
    void PrintReport() { m_report.Print(); }

  private:
    /// Setup lambda table for body parsing
    void initFunctionTable();
    ChOpenSimReport m_report;

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

    // Maps child fields of a body node to functions that handle said fields
    std::map<std::string, std::function<void(rapidxml::xml_node<>*, std::shared_ptr<ChBodyAuxRef>)>> function_table;

    bool m_verbose;     ///< verbose output
    VisType m_visType;  ///< Body visualization type
    bool m_collide;     ///< Do bodies have collision shapes?
    int m_family_1;     ///< First collision family
    int m_family_2;     ///< Second collision family

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping

    std::vector<std::shared_ptr<ChBodyAuxRef>> m_bodyList;  ///< List of bodies in model
    std::vector<std::shared_ptr<ChLink>> m_jointList;       ///< List of joints in model
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
