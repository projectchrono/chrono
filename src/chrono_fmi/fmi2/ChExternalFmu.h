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
// Authors: Radu Serban
// =============================================================================
//
// Chrono physics item that wraps a model exchange FMU.
// 
// =============================================================================

#ifndef CH_EXTERNAL_FMU_H
#define CH_EXTERNAL_FMU_H

#include <functional>
#include <unordered_set>
#include <unordered_map>

#include "chrono_fmi/ChApiFMI.h"
#include "chrono/physics/ChExternalDynamics.h"
#include "fmi2/FmuToolsCommon.h"

class FmuUnit;

namespace chrono {

/// Chrono physics item that wraps a model exchange FMU.
/// This class allows importing the underlying model in the associated FMU and encapsulate it into a ChExternalDynamics
/// physics item which can be added to a Chrono system. The dynamics of the model exchange FMU are integrated at the
/// same time as the Chrono system and its states can be coupled to other components in the system.
class ChApiFMI ChExternalFmu : public ChExternalDynamics {
  public:
    /// Load and instantiate the FMU with given name and wrap it as a Chrono physics item.
    /// If an empty unpack_dir name is specified, the FMU archive is extracted in a temporary directory.
    /// If an empty resources_dir name is specified, the directory in the FMU file is used (if any).
    ChExternalFmu(const std::string& instance_name,      ///< name of the FMU instance
                  const std::string& fmu_filename,       ///< name of the FMU file
                  const std::string& unpack_dir = "",    ///< unpack directory
                  bool logging = false,                  ///< enable FMU logging
                  const std::string& resources_dir = ""  ///< location of FMU resources
    );
    ~ChExternalFmu();

    /// Set FMU debug logging.
    void SetDebugLogging(bool logging, const std::vector<std::string>& log_categories);

    /// Set the initial value for an FMU state.
    /// This function must be called before Initialize() and only for an FMU variable of type="Real" that is declared as
    /// state variable.
    void SetInitialCondition(const std::string& name, double value);

    /// Set the value of a real parameter.
    /// This function must be called before Initialize() and only for an FMU variable of type="Real", with
    /// variability!="constant" that has initial="exact" or causality="input" and is not a state variable.
    void SetRealParameterValue(const std::string& name, double value);

    /// Set the value of an integer parameter.
    /// This function must be called before Initialize() and only for an FMU variable of type="Integer", with
    /// variability!="constant" that has initial="exact" or causality="input" and is not a state variable.
    void SetIntParameterValue(const std::string& name, int value);

    /// Set a continuous input function.
    /// This function will be called automatically during the simulation loop, before interogating the FMU for the model
    /// equations, to return a value that is then used to set the FMU variable with specified name at the current time.
    /// Continuous inputs are FMU variables of type="Real", with causality="input" and variability="continuous".
    void SetRealInputFunction(const std::string& name, std::function<double(double)> function);

    /// Initialize this physics item.
    /// This function initializes the underlying FMU as well as this physcis item.
    virtual void Initialize() override;

    /// Get number of states.
    virtual unsigned int GetNumStates() const override { return m_num_states; }

    /// Get a list of all real states among the FMU variables.
    /// These are variables of type="Real" that are declared as state variables.
    /// The value of such parameters can be set by calling SetRealParameterValue().
    std::unordered_set<std::string> GetStatesList() const;

    /// Get a list of all real "parameters" among the FMU variables.
    /// These are variables of type="Real", with variability!="constant" that have initial="exact" or causality="input"
    /// and are not state variables. The value of such parameters can be set by calling SetRealParameterValue().
    std::unordered_set<std::string> GetRealParametersList() const;

    /// Get a list of all integer "parameters" among the FMU variables.
    /// These are variables of type="Integer", with variability!="constant" that have initial="exact" or
    /// causality="input" and are not state variables. The value of such parameters can be set by calling
    /// SetIntParameterValue().
    std::unordered_set<std::string> GetIntParametersList() const;

    /// Get a list of all continuous real "inputs" among the FMU variables.
    /// These are variables of type="Integer", with causality="input" and variability="continuous".
    std::unordered_set<std::string> GetRealInputsList() const;

    /// Print the list of FMU variables.
    void PrintFmuVariables() const;

  private:
    /// Set initial conditions.
    /// Must load y0 = y(0).
    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override;

    /// Calculate and return the ODE right-hand side at the provided time and states.
    /// Must load rhs = f(t,y).
    virtual void CalculateRHS(double time,                 ///< current time
                              const ChVectorDynamic<>& y,  ///< current ODE states
                              ChVectorDynamic<>& rhs       ///< output ODE right-hand side vector
                              ) override;

    virtual void Update(double time, bool update_assets = true) override;

    bool checkState(const std::string& name, std::string& err_msg) const;
    bool checkParam(const std::string& name, FmuVariable::Type type, std::string& err_msg) const;
    bool checkInput(const std::string& name, FmuVariable::Type type, std::string& err_msg) const;

    std::unique_ptr<FmuUnit> m_fmu;
    bool m_initialized;
    unsigned int m_num_states;

    std::unordered_map<std::string, double> m_initial_conditions;
    std::unordered_map<std::string, double> m_parameters_real;
    std::unordered_map<std::string, int> m_parameters_int;
    std::unordered_map<std::string, std::function<double(double)>> m_inputs_real;
};

}  // end namespace chrono

#endif
