// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Mangoni, Radu Serban
// =============================================================================
//
// Chrono wrappers to fmu_tools FMU export classes for FMI standard 3.0.
//
// =============================================================================

#ifndef CH_FMU3_TOOLS_EXPORT_H
#define CH_FMU3_TOOLS_EXPORT_H

#include <stack>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <memory>

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/physics/ChAssembly.h"

#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChVisualShapes.h"

// fmu_tools
// #include "rapidxml_ext.hpp"
#include "fmi3/FmuToolsExport.h"

namespace chrono {
namespace fmi3 {

/// @addtogroup chrono_fmi3
/// @{

using FmuVariable = fmu_tools::fmi3::FmuVariable;

// -----------------------------------------------------------------------------

/// Extension of FmuComponentBase class for Chrono FMUs.
class FmuChronoComponentBase : public fmu_tools::fmi3::FmuComponentBase {
  public:
    FmuChronoComponentBase(fmu_tools::fmi3::FmuType fmiInterfaceType,
                           fmi3String instanceName,
                           fmi3String instantiationToken,
                           fmi3String resourcePath,
                           fmi3Boolean visible,
                           fmi3Boolean loggingOn,
                           fmi3InstanceEnvironment instanceEnvironment,
                           fmi3LogMessageCallback logMessage)
        : FmuComponentBase(
              fmiInterfaceType,
              instanceName,
              instantiationToken,
              resourcePath,
              visible,
              loggingOn,
              instanceEnvironment,
              logMessage,
              {{"logEvents", true},
               {"logSingularLinearSystems", true},
               {"logNonlinearSystems", true},
               {"logStatusWarning", true},
               {"logStatusError", true},
               {"logStatusPending", true},
               {"logDynamicStateSelection", true},
               {"logStatusDiscard", true},
               {"logStatusFatal", true},
               {"logAll", true}},
              {"logStatusWarning", "logStatusDiscard", "logStatusError", "logStatusFatal", "logStatusPending"}) {}

    virtual ~FmuChronoComponentBase() {}

    /// Add FMU variables corresponding to the specified ChVector3d.
    /// This function creates 3 FMU variables, one for each component of the ChVector3d, with names "name.x", "name.y",
    /// and "name.z", all of type FmuVariable::Type::Float64.
    void AddFmuVecVariable(ChVector3d& v,
                           const std::string& name,
                           const std::string& unit_name,
                           const std::string& description,
                           FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                           FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous,
                           FmuVariable::InitialType initial = FmuVariable::InitialType::none) {
        addFmuVecVariable(v, name, unit_name, description, causality, variability, initial, true);
    }

    /// Add FMU variables corresponding to the specified ChQuaternion.
    /// This function creates 4 FMU variables, one for each component of the ChVector3d, with names "name.e0",
    /// "name.e1", "name.e2", and "name.e3", all of type FmuVariable::Type::Float64.
    void AddFmuQuatVariable(ChQuaternion<>& q,
                            const std::string& name,
                            const std::string& unit_name,
                            const std::string& description,
                            FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                            FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous,
                            FmuVariable::InitialType initial = FmuVariable::InitialType::none) {
        addFmuQuatVariable(q, name, unit_name, description, causality, variability, initial, true);
    }

    /// Add FMU variables corresponding to the specified ChCoordsys.
    /// This function creates 7 FMU variables, one for each component of the position ChVector3d and one for each
    /// component of the rotation quaternion, all of type FmuVariable::Type::Float64.
    void AddFmuCsysVariable(ChCoordsysd& s,
                            const std::string& name,
                            const std::string& unit_name,
                            const std::string& description,
                            FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                            FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous,
                            FmuVariable::InitialType initial = FmuVariable::InitialType::none) {
        addFmuCsysVariable(s, name, unit_name, description, causality, variability, initial, true);
    }

    /// Add FMU variables corresponding to the specified ChFrame.
    /// This function creates 7 FMU variables, one for each component of the position ChVector3d and one for each
    /// component of the rotation quaternion, all of type FmuVariable::Type::Float64.
    void AddFmuFrameVariable(ChFrame<>& s,
                             const std::string& name,
                             const std::string& unit_name,
                             const std::string& description,
                             FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                             FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous,
                             FmuVariable::InitialType initial = FmuVariable::InitialType::none) {
        addFmuCsysVariable(s.m_csys, name, unit_name, description, causality, variability, initial, true);
    }

    /// Add FMU variables corresponding to the specified ChFrameMoving.
    /// This function creates 7 FMU variables for the pose, one for each component of the position ChVector3d and one
    /// for each component of the rotation quaternion, all of type FmuVariable::Type::Float64.  Additionally, 7 FMU
    /// variables are created to encode the position and orientation time derivatives.
    void AddFmuFrameMovingVariable(ChFrameMoving<>& s,
                                   const std::string& name,
                                   const std::string& unit_name,
                                   const std::string& unit_name_dt,
                                   const std::string& description,
                                   FmuVariable::CausalityType causality = FmuVariable::CausalityType::local,
                                   FmuVariable::VariabilityType variability = FmuVariable::VariabilityType::continuous,
                                   FmuVariable::InitialType initial = FmuVariable::InitialType::none) {
        addFmuFrameMovingVariable(s, name, unit_name, unit_name_dt, description, causality, variability, initial, true);
    }

  protected:
    std::unordered_set<std::string> variables_vec;     ///< list of ChVector3 "variables"
    std::unordered_set<std::string> variables_quat;    ///< list of ChQuaternion "variables"
    std::unordered_set<std::string> variables_csys;    ///< list of ChCoordsys "variables"
    std::unordered_set<std::string> variables_framem;  ///< list of ChFrameMoving "variables"

    void addFmuVecVariable(ChVector3d& v,
                           const std::string& name,
                           const std::string& unit_name,
                           const std::string& description,
                           FmuVariable::CausalityType causality,
                           FmuVariable::VariabilityType variability,
                           FmuVariable::InitialType initial,
                           bool cache) {
        std::string comp[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            AddFmuVariable(&v.data()[i], name + "." + comp[i], FmuVariable::Type::Float64, unit_name,
                           description + " (" + comp[i] + ")", causality, variability, initial);
        }

        if (cache)
            variables_vec.insert(name);
    }

    void addFmuQuatVariable(ChQuaternion<>& q,
                            const std::string& name,
                            const std::string& unit_name,
                            const std::string& description,
                            FmuVariable::CausalityType causality,
                            FmuVariable::VariabilityType variability,
                            FmuVariable::InitialType initial,
                            bool cache) {
        std::string comp[4] = {"e0", "e1", "e2", "e3"};
        for (int i = 0; i < 4; i++) {
            AddFmuVariable(&q.data()[i], name + "." + comp[i], FmuVariable::Type::Float64, unit_name,
                           description + " (" + comp[i] + ")", causality, variability, initial);
        }

        if (cache)
            variables_quat.insert(name);
    }

    void addFmuCsysVariable(ChCoordsysd& s,
                            const std::string& name,
                            const std::string& unit_name,
                            const std::string& description,
                            FmuVariable::CausalityType causality,
                            FmuVariable::VariabilityType variability,
                            FmuVariable::InitialType initial,
                            bool cache) {
        addFmuVecVariable(s.pos, name + ".pos", unit_name, description + " position", causality, variability, initial,
                          false);
        addFmuQuatVariable(s.rot, name + ".rot", "1", description + " orientation", causality, variability, initial,
                           false);

        if (cache)
            variables_csys.insert(name);
    }

    void addFmuFrameMovingVariable(ChFrameMoving<>& s,
                                   const std::string& name,
                                   const std::string& unit_name,
                                   const std::string& unit_name_dt,
                                   const std::string& description,
                                   FmuVariable::CausalityType causality,
                                   FmuVariable::VariabilityType variability,
                                   FmuVariable::InitialType initial,
                                   bool cache) {
        addFmuCsysVariable(s.m_csys, name, unit_name, description, causality, variability, initial, false);
        addFmuVecVariable(s.m_csys_dt.pos, name + ".pos_dt", unit_name_dt, description + " position derivative",
                          causality, variability, initial, false);
        addFmuQuatVariable(s.m_csys_dt.rot, name + ".rot_dt", "1", description + " orientation derivative", causality,
                           variability, initial, false);

        if (cache)
            variables_framem.insert(name);
    }

    /// Add a declaration of a state derivative.
    /// This version accounts for states and state derivatives corresponding to ChVector3 or ChQuaternion types.
    /// These are expanded to the appropriate number of FMI scalar variables.
    /// Dependencies can include ChVector3, ChQuaternion, ChCoordsys, or ChFrameMoving variables.
    virtual void addDerivative(const std::string& derivative_name,
                               const std::string& state_name,
                               const std::vector<std::string>& dependency_names) override {
        std::vector<std::string> derivatives;
        std::vector<std::string> states;
        std::vector<std::string> dependencies;

        // Check if derivative_name corresponds to a ChVector3 or ChQuaternion object
        if (variables_vec.find(derivative_name) != variables_vec.end()) {
            derivatives.push_back(derivative_name + ".x");
            derivatives.push_back(derivative_name + ".y");
            derivatives.push_back(derivative_name + ".z");
        } else if (variables_quat.find(derivative_name) != variables_quat.end()) {
            derivatives.push_back(derivative_name + ".e0");
            derivatives.push_back(derivative_name + ".e1");
            derivatives.push_back(derivative_name + ".e2");
            derivatives.push_back(derivative_name + ".e3");
        } else if (variables_csys.find(derivative_name) != variables_csys.end()) {
            throw std::runtime_error("Derivative of type ChCoordsys not allowed.");
        } else if (variables_framem.find(derivative_name) != variables_framem.end()) {
            throw std::runtime_error("Derivative of type ChFrameMoving not allowed.");
        } else {
            derivatives.push_back(derivative_name);
        }

        // Check if state_name corresponds to a ChVector3 or ChQuaternion object
        if (variables_vec.find(state_name) != variables_vec.end()) {
            derivatives.push_back(state_name + ".x");
            derivatives.push_back(state_name + ".y");
            derivatives.push_back(state_name + ".z");
        } else if (variables_quat.find(state_name) != variables_quat.end()) {
            derivatives.push_back(state_name + ".e0");
            derivatives.push_back(state_name + ".e1");
            derivatives.push_back(state_name + ".e2");
            derivatives.push_back(state_name + ".e3");
        } else if (variables_csys.find(state_name) != variables_csys.end()) {
            throw std::runtime_error("State of type ChCoordsys not allowed.");
        } else if (variables_framem.find(state_name) != variables_framem.end()) {
            throw std::runtime_error("State of type ChFrameMoving not allowed.");
        } else {
            derivatives.push_back(state_name);
        }

        // Sanity check
        auto num_derivatives = derivatives.size();
        if (states.size() != num_derivatives) {
            throw std::runtime_error(
                "Incorrect state derivative declaration (number of derivatives does not match number of states).");
        }

        for (const auto& dependency_name : dependency_names) {
            // Check if dependency_name corresponds to a ChVector3, ChQuaternion, ChCoordsys, or ChFrameMoving object
            if (variables_vec.find(dependency_name) != variables_vec.end()) {
                dependencies.push_back(dependency_name + ".x");
                dependencies.push_back(dependency_name + ".y");
                dependencies.push_back(dependency_name + ".z");
            } else if (variables_quat.find(dependency_name) != variables_quat.end()) {
                dependencies.push_back(dependency_name + ".e0");
                dependencies.push_back(dependency_name + ".e1");
                dependencies.push_back(dependency_name + ".e2");
                dependencies.push_back(dependency_name + ".e3");
            } else if (variables_csys.find(dependency_name) != variables_csys.end()) {
                dependencies.push_back(dependency_name + ".pos.x");
                dependencies.push_back(dependency_name + ".pos.y");
                dependencies.push_back(dependency_name + ".pos.z");
                dependencies.push_back(dependency_name + ".rot.e0");
                dependencies.push_back(dependency_name + ".rot.e1");
                dependencies.push_back(dependency_name + ".rot.e2");
                dependencies.push_back(dependency_name + ".rot.e3");
            } else if (variables_framem.find(dependency_name) != variables_framem.end()) {
                dependencies.push_back(dependency_name + ".pos.x");
                dependencies.push_back(dependency_name + ".pos.y");
                dependencies.push_back(dependency_name + ".pos.z");
                dependencies.push_back(dependency_name + ".rot.e0");
                dependencies.push_back(dependency_name + ".rot.e1");
                dependencies.push_back(dependency_name + ".rot.e2");
                dependencies.push_back(dependency_name + ".rot.e3");
                dependencies.push_back(dependency_name + ".pos_dt.x");
                dependencies.push_back(dependency_name + ".pos_dt.y");
                dependencies.push_back(dependency_name + ".pos_dt.z");
                dependencies.push_back(dependency_name + ".rot_dt.e0");
                dependencies.push_back(dependency_name + ".rot_dt.e1");
                dependencies.push_back(dependency_name + ".rot_dt.e2");
                dependencies.push_back(dependency_name + ".rot_dt.e3");
            } else {
                dependencies.push_back(dependency_name);
            }
        }

        // Now invoke the base class method for each pair of scalar derivative/state
        for (size_t i = 0; i < num_derivatives; i++) {
            FmuComponentBase::addDerivative(derivatives[i], states[i], dependencies);
        }
    }

    /// Include a dependency of "variable_name" on "dependency_name".
    /// This version accounts for variables and/or dependencies corresponding to ChVector3, ChQuaternion, ChCoordsys,
    /// ChFrame, or ChFrameMoving types. These are expanded to the appropriate number of FMI scalar variables.
    virtual void addDependencies(const std::string& variable_name,
                                 const std::vector<std::string>& dependency_names) override {
        std::vector<std::string> variables;
        std::vector<std::string> dependencies;

        // Check if variable_name corresponds to a ChVector3, ChQuaternion, ChCoordsys, or ChFrameMoving object
        if (variables_vec.find(variable_name) != variables_vec.end()) {
            variables.push_back(variable_name + ".x");
            variables.push_back(variable_name + ".y");
            variables.push_back(variable_name + ".z");
        } else if (variables_quat.find(variable_name) != variables_quat.end()) {
            variables.push_back(variable_name + ".e0");
            variables.push_back(variable_name + ".e1");
            variables.push_back(variable_name + ".e2");
            variables.push_back(variable_name + ".e3");
        } else if (variables_csys.find(variable_name) != variables_csys.end()) {
            variables.push_back(variable_name + ".pos.x");
            variables.push_back(variable_name + ".pos.y");
            variables.push_back(variable_name + ".pos.z");
            variables.push_back(variable_name + ".rot.e0");
            variables.push_back(variable_name + ".rot.e1");
            variables.push_back(variable_name + ".rot.e2");
            variables.push_back(variable_name + ".rot.e3");
        } else if (variables_framem.find(variable_name) != variables_framem.end()) {
            variables.push_back(variable_name + ".pos.x");
            variables.push_back(variable_name + ".pos.y");
            variables.push_back(variable_name + ".pos.z");
            variables.push_back(variable_name + ".rot.e0");
            variables.push_back(variable_name + ".rot.e1");
            variables.push_back(variable_name + ".rot.e2");
            variables.push_back(variable_name + ".rot.e3");
            variables.push_back(variable_name + ".pos_dt.x");
            variables.push_back(variable_name + ".pos_dt.y");
            variables.push_back(variable_name + ".pos_dt.z");
            variables.push_back(variable_name + ".rot_dt.e0");
            variables.push_back(variable_name + ".rot_dt.e1");
            variables.push_back(variable_name + ".rot_dt.e2");
            variables.push_back(variable_name + ".rot_dt.e3");
        } else {
            variables.push_back(variable_name);
        }

        for (const auto& dependency_name : dependency_names) {
            // Check if dependency_name corresponds to a ChVector3, ChQuaternion, ChCoordsys, or ChFrameMoving object
            if (variables_vec.find(dependency_name) != variables_vec.end()) {
                dependencies.push_back(dependency_name + ".x");
                dependencies.push_back(dependency_name + ".y");
                dependencies.push_back(dependency_name + ".z");
            } else if (variables_quat.find(dependency_name) != variables_quat.end()) {
                dependencies.push_back(dependency_name + ".e0");
                dependencies.push_back(dependency_name + ".e1");
                dependencies.push_back(dependency_name + ".e2");
                dependencies.push_back(dependency_name + ".e3");
            } else if (variables_csys.find(dependency_name) != variables_csys.end()) {
                dependencies.push_back(dependency_name + ".pos.x");
                dependencies.push_back(dependency_name + ".pos.y");
                dependencies.push_back(dependency_name + ".pos.z");
                dependencies.push_back(dependency_name + ".rot.e0");
                dependencies.push_back(dependency_name + ".rot.e1");
                dependencies.push_back(dependency_name + ".rot.e2");
                dependencies.push_back(dependency_name + ".rot.e3");
            } else if (variables_framem.find(dependency_name) != variables_framem.end()) {
                dependencies.push_back(dependency_name + ".pos.x");
                dependencies.push_back(dependency_name + ".pos.y");
                dependencies.push_back(dependency_name + ".pos.z");
                dependencies.push_back(dependency_name + ".rot.e0");
                dependencies.push_back(dependency_name + ".rot.e1");
                dependencies.push_back(dependency_name + ".rot.e2");
                dependencies.push_back(dependency_name + ".rot.e3");
                dependencies.push_back(dependency_name + ".pos_dt.x");
                dependencies.push_back(dependency_name + ".pos_dt.y");
                dependencies.push_back(dependency_name + ".pos_dt.z");
                dependencies.push_back(dependency_name + ".rot_dt.e0");
                dependencies.push_back(dependency_name + ".rot_dt.e1");
                dependencies.push_back(dependency_name + ".rot_dt.e2");
                dependencies.push_back(dependency_name + ".rot_dt.e3");
            } else {
                dependencies.push_back(dependency_name);
            }
        }

        // Now invoke the base class method for each scalar variable
        for (const auto& v : variables) {
            FmuComponentBase::addDependencies(v, dependencies);
        }
    }
};

/// @} chrono_fmi3

}  // end namespace fmi3
}  // end namespace chrono

#endif
