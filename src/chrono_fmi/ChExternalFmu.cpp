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

#include "chrono_fmi/ChExternalFmu.h"

#include "fmi2/FmuToolsImport.h"
#include "fmi3/FmuToolsImport.h"

namespace chrono {

// =============================================================================
// Definition of the interface to an FMI 2.0 model exchange FMU

class ChFmu2Wrapper : public ChFmuWrapper {
  public:
    ChFmu2Wrapper(const std::string& instance_name,  // name of the FMU instance
                  const std::string& fmu_filename,   // name of the FMU file
                  const std::string& unpack_dir,     // unpack directory
                  bool logging,                      // enable FMU logging
                  const std::string& resources_dir,  // location of FMU resources
                  bool verbose                       // verbose output
    );

  private:
    using FmuVariable = fmu_tools::fmi2::FmuVariable;

    virtual void SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) override;
    virtual unsigned int GetNumStates() const override;
    virtual std::unordered_set<std::string> GetStatesList() const override;
    virtual std::unordered_set<std::string> GetRealParametersList() const override;
    virtual std::unordered_set<std::string> GetIntParametersList() const override;
    virtual std::unordered_set<std::string> GetRealInputsList() const override;
    virtual void Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                            const std::unordered_map<std::string, double>& parameters_real,
                            const std::unordered_map<std::string, int>& parameters_int) override;
    virtual bool checkState(const std::string& name, std::string& err_msg) const override;
    virtual bool checkInput(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamReal(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamInt(const std::string& name, std::string& err_msg) const override;
    virtual void SetInputs(const std::unordered_map<std::string, double>& inputs_real) override;
    virtual void SetContinuousStates(const std::vector<double>& states) override;
    virtual void GetContinuousStates(std::vector<double>& states) override;
    virtual void GetContinuousDerivatives(std::vector<double>& derivs) override;
    virtual void PrintFmuVariables() const override;

    bool checkParam(const std::string& name, FmuVariable::Type type, std::string& err_msg) const;

    fmu_tools::fmi2::FmuUnit m_fmu;
};

// =============================================================================
// Definition of the interface to an FMI 3.0 model exchange FMU

class ChFmu3Wrapper : public ChFmuWrapper {
  public:
    ChFmu3Wrapper(const std::string& instance_name,  // name of the FMU instance
                  const std::string& fmu_filename,   // name of the FMU file
                  const std::string& unpack_dir,     // unpack directory
                  bool logging,                      // enable FMU logging
                  const std::string& resources_dir,  // location of FMU resources
                  bool verbose                       // verbose output
    );

  private:
    using FmuVariable = fmu_tools::fmi3::FmuVariable;

    virtual void SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) override;
    virtual unsigned int GetNumStates() const override;
    virtual std::unordered_set<std::string> GetStatesList() const override;
    virtual std::unordered_set<std::string> GetRealParametersList() const override;
    virtual std::unordered_set<std::string> GetIntParametersList() const override;
    virtual std::unordered_set<std::string> GetRealInputsList() const override;
    virtual void Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                            const std::unordered_map<std::string, double>& parameters_real,
                            const std::unordered_map<std::string, int>& parameters_int) override;
    virtual bool checkState(const std::string& name, std::string& err_msg) const override;
    virtual bool checkInput(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamReal(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamInt(const std::string& name, std::string& err_msg) const override;
    virtual void SetInputs(const std::unordered_map<std::string, double>& inputs_real) override;
    virtual void SetContinuousStates(const std::vector<double>& states) override;
    virtual void GetContinuousStates(std::vector<double>& states) override;
    virtual void GetContinuousDerivatives(std::vector<double>& derivs) override;
    virtual void PrintFmuVariables() const override;

    fmu_tools::fmi3::FmuUnit m_fmu;
};

// =============================================================================
// Implementation of a ChExternalDynamics component that wraps an FMU

ChExternalFmu::ChExternalFmu() : m_verbose(false), m_initialized(false), m_num_states(0) {}

void ChExternalFmu::Load(const std::string& instance_name,  // name of the FMU instance
                         const std::string& fmu_filename,   // name of the FMU file
                         const std::string& unpack_dir,     // unpack directory
                         bool logging,                      // enable FMU logging
                         const std::string& resources_dir   // location of FMU resources
) {
    // Peek in FMU model description file to get FMI version
    auto fmi_version = fmu_tools::GetFmuVersion(fmu_filename);

    // Create an FMU wrapper of appropriate type
    try {
        switch (fmi_version) {
            case fmu_tools::FmuVersion::FMI2:
                if (m_verbose)
                    std::cout << "\nLoading FMI 2.0 FMU: " << fmu_filename << "\n" << std::endl;
                m_wrapper = chrono_types::make_unique<ChFmu2Wrapper>(instance_name, fmu_filename, unpack_dir, logging,
                                                                     resources_dir, m_verbose);
                break;
            case fmu_tools::FmuVersion::FMI3:
                if (m_verbose)
                    std::cout << "\nLoading FMI 3.0 FMU: " << fmu_filename << "\n" << std::endl;
                m_wrapper = chrono_types::make_unique<ChFmu3Wrapper>(instance_name, fmu_filename, unpack_dir, logging,
                                                                     resources_dir, m_verbose);
                break;
        }
    } catch (std::exception&) {
        throw;
    }

    m_num_states = m_wrapper->GetNumStates();
}

void ChExternalFmu::SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) {
    m_wrapper->SetDebugLogging(logging, log_categories);
}

std::unordered_set<std::string> ChExternalFmu::GetStatesList() const {
    return m_wrapper->GetStatesList();
}

std::unordered_set<std::string> ChExternalFmu::GetRealParametersList() const {
    return m_wrapper->GetRealParametersList();
}

std::unordered_set<std::string> ChExternalFmu::GetIntParametersList() const {
    return m_wrapper->GetIntParametersList();
}

std::unordered_set<std::string> ChExternalFmu::GetRealInputsList() const {
    return m_wrapper->GetRealInputsList();
}

void ChExternalFmu::SetInitialCondition(const std::string& name, double value) {
    if (m_initialized) {
        std::cerr << "SetInitialCondition cannot be called after Initialize()";
        throw std::runtime_error("SetInitialCondition cannot be called after Initialize()");
    }

    std::string err_msg;
    bool ok = m_wrapper->checkState(name, err_msg);

    if (!ok) {
        std::cerr << "SetInitialCondition: " + err_msg << std::endl;
        throw std::runtime_error("SetInitialCondition: " + err_msg);
    }

    m_initial_conditions.insert({name, value});
}

void ChExternalFmu::SetRealParameterValue(const std::string& name, double value) {
    if (m_initialized) {
        std::cerr << "SetRealParameterValue cannot be called after Initialize()";
        throw std::runtime_error("SetRealParameterValue cannot be called after Initialize()");
    }

    std::string err_msg;
    bool ok = m_wrapper->checkParamReal(name, err_msg);

    if (!ok) {
        std::cerr << "SetRealParameterValue: " + err_msg << std::endl;
        throw std::runtime_error("SetRealParameterValue: " + err_msg);
    }

    m_parameters_real.insert({name, value});
}

void ChExternalFmu::SetIntParameterValue(const std::string& name, int value) {
    if (m_initialized) {
        std::cerr << "SetIntParameterValue cannot be called after Initialize()";
        throw std::runtime_error("SetIntParameterValue cannot be called after Initialize()");
    }

    std::string err_msg;
    bool ok = m_wrapper->checkParamInt(name, err_msg);

    if (!ok) {
        std::cerr << "SetIntParameterValue: " + err_msg << std::endl;
        throw std::runtime_error("SetIntParameterValue: " + err_msg);
    }

    m_parameters_int.insert({name, value});
}

void ChExternalFmu::SetRealInputFunction(const std::string& name, std::function<double(double)> function) {
    if (m_initialized) {
        std::cerr << "SetRealInputFunction cannot be called after Initialize()";
        throw std::runtime_error("SetRealInputFunction cannot be called after Initialize()");
    }

    std::string err_msg;
    bool ok = m_wrapper->checkInput(name, err_msg);

    if (!ok) {
        std::cerr << "SetRealInputFunction: " + err_msg << std::endl;
        throw std::runtime_error("SetRealInputFunction: " + err_msg);
    }

    m_inputs_real.insert({name, function});
}

void ChExternalFmu::Initialize() {
    // Let the wrapper initialize its underlying FMU
    m_wrapper->Initialize(m_initial_conditions, m_parameters_real, m_parameters_int);

    // Initialize the base class
    ChExternalDynamics::Initialize();

    m_initialized = true;
}

// -----------------

void ChExternalFmu::SetInitialConditions(ChVectorDynamic<>& y0) {
    std::vector<double> states(m_num_states);
    m_wrapper->GetContinuousStates(states);

    // Load initial conditions for the ChExternalDynamics component
    for (unsigned int i = 0; i < m_num_states; i++)
        y0(i) = states[i];
}

void ChExternalFmu::CalculateRHS(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& rhs) {
    // Load provided states and pass them to the FMU
    std::vector<double> states(m_num_states);
    for (unsigned int i = 0; i < m_num_states; i++)
        states[i] = y(i);
    m_wrapper->SetContinuousStates(states);

    // Get the RHS from the FMU and load for the ChExternalDynamics component
    std::vector<double> derivs(m_num_states);
    m_wrapper->GetContinuousDerivatives(derivs);
    for (unsigned int i = 0; i < m_num_states; i++)
        rhs(i) = derivs[i];
}

void ChExternalFmu::Update(double time, bool update_assets) {
    // Collect input values at current time
    std::unordered_map<std::string, double> inputs_real;
    for (const auto& v : m_inputs_real) {
        double value = v.second(time);
        inputs_real.insert({v.first, value});
    }

    m_wrapper->SetInputs(inputs_real);

    // Invoke base class Update
    ChExternalDynamics::Update(time, update_assets);
}

void ChExternalFmu::PrintFmuVariables() const {
    std::cout << "FMU variables" << std::endl;
    m_wrapper->PrintFmuVariables();
}

// =============================================================================
// Implementation of the interface to an FMI 2.0 model exchange FMU

ChFmu2Wrapper::ChFmu2Wrapper(const std::string& instance_name,
                             const std::string& fmu_filename,
                             const std::string& unpack_dir,
                             bool logging,
                             const std::string& resources_dir,
                             bool verbose) {
    // Create the underlying FMU
    m_fmu.SetVerbose(verbose);

    // Load the FMU from the specified file
    try {
        if (unpack_dir.empty())
            m_fmu.Load(fmi2Type::fmi2ModelExchange, fmu_filename);
        else
            m_fmu.Load(fmi2Type::fmi2ModelExchange, fmu_filename, unpack_dir);
    } catch (std::exception&) {
        throw;
    }

    // Instantiate the FMU
    try {
        if (resources_dir.empty())
            m_fmu.Instantiate(instance_name, logging, false);
        else
            m_fmu.Instantiate(instance_name, resources_dir, logging, false);
    } catch (std::exception&) {
        throw;
    }

    // Set up an experiment
    m_fmu.SetupExperiment(fmi2False,  // no tolerance defined
                           0.0,        // tolerance (dummy)
                           0.0,        // start time
                           fmi2False,  // do not use stop time
                           1.0         // stop time (dummy)
    );
}

void ChFmu2Wrapper::SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) {
    if (logging)
        m_fmu.SetDebugLogging(fmi2True, log_categories);
}

unsigned int ChFmu2Wrapper::GetNumStates() const {
    return (unsigned int)m_fmu.GetNumStates();
}

std::unordered_set<std::string> ChFmu2Wrapper::GetStatesList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkState(v.first, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

std::unordered_set<std::string> ChFmu2Wrapper::GetRealParametersList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkParam(v.first, FmuVariable::Type::Real, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

std::unordered_set<std::string> ChFmu2Wrapper::GetIntParametersList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkParam(v.first, FmuVariable::Type::Integer, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

std::unordered_set<std::string> ChFmu2Wrapper::GetRealInputsList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkInput(v.first, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

// Initialize underlying FMU
void ChFmu2Wrapper::Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                               const std::unordered_map<std::string, double>& parameters_real,
                               const std::unordered_map<std::string, int>& parameters_int) {
    m_fmu.EnterInitializationMode();

    // Set initial conditions
    for (const auto& v : initial_conditions)
        m_fmu.SetVariable(v.first, v.second, FmuVariable::Type::Real);

    // Set real and integer parameters
    for (const auto& v : parameters_real)
        m_fmu.SetVariable(v.first, v.second, FmuVariable::Type::Real);
    for (const auto& v : parameters_int)
        m_fmu.SetVariable(v.first, v.second, FmuVariable::Type::Integer);

    m_fmu.ExitInitializationMode();
}

// Check that the FMU variable with given name is a state
bool ChFmu2Wrapper::checkState(const std::string& name, std::string& err_msg) const {
    const auto& variables = m_fmu.GetVariablesList();
    auto search = variables.find(name);
    if (search == variables.end()) {
        err_msg = "[ChFmu2Wrapper::checkState] variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = search->second;
    bool ok = var.GetType() == FmuVariable::Type::Real && var.IsState();

    if (!ok) {
        err_msg = "[ChFmu2Wrapper::checkState] illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Check that the FMU variable with given name is a floating point "input"
bool ChFmu2Wrapper::checkInput(const std::string& name, std::string& err_msg) const {
    const auto& variables = m_fmu.GetVariablesList();
    auto search = variables.find(name);
    if (search == variables.end()) {
        err_msg = "[ChFmu2Wrapper::checkInput] variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = search->second;
    bool ok = var.GetType() == FmuVariable::Type::Real && var.GetCausality() == FmuVariable::CausalityType::input &&
              var.GetVariability() == FmuVariable::VariabilityType::continuous;

    if (!ok) {
        err_msg = "[ChFmu2Wrapper::checkInput] illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Check that the FMU variable with given name is a floating point "parameter"
bool ChFmu2Wrapper::checkParamReal(const std::string& name, std::string& err_msg) const {
    return checkParam(name, FmuVariable::Type::Real, err_msg);
}

// Check that the FMU variable with given name is an integral "parameter"
bool ChFmu2Wrapper::checkParamInt(const std::string& name, std::string& err_msg) const {
    return checkParam(name, FmuVariable::Type::Integer, err_msg);
}

// Check that an FMU variable with given name is a "parameter" with specified type
bool ChFmu2Wrapper::checkParam(const std::string& name, FmuVariable::Type type, std::string& err_msg) const {
    const auto& variables = m_fmu.GetVariablesList();
    auto search = variables.find(name);
    if (search == variables.end()) {
        err_msg = "[ChFmu2Wrapper::checkParam] variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = search->second;
    bool ok = var.GetType() == type && !var.IsState() &&
              var.GetVariability() != FmuVariable::VariabilityType::constant &&
              (var.GetInitial() == FmuVariable::InitialType::exact ||
               var.GetCausality() == FmuVariable::CausalityType::input);

    if (!ok) {
        err_msg = "[ChFmu2Wrapper::checkParam] illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

void ChFmu2Wrapper::SetInputs(const std::unordered_map<std::string, double>& inputs_real) {
    for (const auto& v : inputs_real) {
        m_fmu.SetVariable(v.first, v.second, FmuVariable::Type::Real);
    }
}

void ChFmu2Wrapper::SetContinuousStates(const std::vector<double>& states) {
    m_fmu.SetContinuousStates(states.data(), states.size());
}

void ChFmu2Wrapper::GetContinuousStates(std::vector<double>& states) {
    m_fmu.GetContinuousStates(states.data(), states.size());
}

void ChFmu2Wrapper::GetContinuousDerivatives(std::vector<double>& derivs) {
    m_fmu.GetDerivatives(derivs.data(), derivs.size());
}

void ChFmu2Wrapper::PrintFmuVariables() const {
    const auto& variables = m_fmu.GetVariablesList();
    for (const auto& v : variables) {
        std::cout << v.first << "\t";
        std::cout << "type: " << FmuVariable::Type_toString(v.second.GetType()) << "  index: " << v.second.GetIndex()
                  << "\t";
        std::cout << "state? " << v.second.IsState() << "  deriv? " << v.second.IsDeriv() << "\t";
        std::cout << std::endl;
    }
}

// =============================================================================
// Implementation of the interface to an FMI 3.0 model exchange FMU
ChFmu3Wrapper::ChFmu3Wrapper(const std::string& instance_name,
                             const std::string& fmu_filename,
                             const std::string& unpack_dir,
                             bool logging,
                             const std::string& resources_dir,
                             bool verbose) {
    m_fmu.SetVerbose(verbose);

    // Load the FMU from the specified file
    try {
        if (unpack_dir.empty())
            m_fmu.Load(fmu_tools::fmi3::FmuType::MODEL_EXCHANGE, fmu_filename);
        else
            m_fmu.Load(fmu_tools::fmi3::FmuType::MODEL_EXCHANGE, fmu_filename, unpack_dir);
    } catch (std::exception&) {
        throw;
    }

    // Instantiate the FMU
    try {
        if (resources_dir.empty())
            m_fmu.Instantiate(instance_name, logging, false);
        else
            m_fmu.Instantiate(instance_name, resources_dir, logging, false);
    } catch (std::exception&) {
        throw;
    }
}

void ChFmu3Wrapper::SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) {
    if (logging)
        m_fmu.SetDebugLogging(fmi2True, log_categories);
}

unsigned int ChFmu3Wrapper::GetNumStates() const {
    return (unsigned int)m_fmu.GetNumStates();
}

std::unordered_set<std::string> ChFmu3Wrapper::GetStatesList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkState(v.second.GetName(), err_msg);
        if (ok)
            list.insert(v.second.GetName());
    }

    return list;
}

std::unordered_set<std::string> ChFmu3Wrapper::GetRealParametersList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkParamReal(v.second.GetName(), err_msg);
        if (ok)
            list.insert(v.second.GetName());
    }

    return list;
}

std::unordered_set<std::string> ChFmu3Wrapper::GetIntParametersList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkParamInt(v.second.GetName(), err_msg);
        if (ok)
            list.insert(v.second.GetName());
    }

    return list;
}

std::unordered_set<std::string> ChFmu3Wrapper::GetRealInputsList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu.GetVariablesList()) {
        bool ok = checkInput(v.second.GetName(), err_msg);
        if (ok)
            list.insert(v.second.GetName());
    }

    return list;
}

// Initialize underlying FMU
void ChFmu3Wrapper::Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                               const std::unordered_map<std::string, double>& parameters_real,
                               const std::unordered_map<std::string, int>& parameters_int) {
    m_fmu.EnterInitializationMode(fmi3False,  // no tolerance defined
                                   0.0,        // tolerance (dummy)
                                   0.0,        // start time
                                   fmi3False,  // do not use stop time
                                   1.0         // stop time (dummy)
    );

    // Set initial conditions
    for (const auto& v : initial_conditions)
        m_fmu.SetVariable(v.first, v.second);

    // Set real and integer parameters
    for (const auto& v : parameters_real)
        m_fmu.SetVariable(v.first, v.second);
    for (const auto& v : parameters_int)
        m_fmu.SetVariable(v.first, v.second);

    m_fmu.ExitInitializationMode();
}

// Check that the FMU variable with given name is a state
bool ChFmu3Wrapper::checkState(const std::string& name, std::string& err_msg) const {
    fmi3ValueReference vr;
    if (!m_fmu.GetValueReference(name, vr)) {
        err_msg = "[ChFmu3Wrapper::checkState] variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = m_fmu.GetVariablesList().at(vr);
    auto var_type = var.GetType();

    bool ok = (var_type == FmuVariable::Type::Float32 || var_type == FmuVariable::Type::Float64) &&  //
              var.IsState();

    if (!ok) {
        err_msg = "[ChFmu3Wrapper::checkState] illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Check that the FMU variable with given name is a floating point "input"
bool ChFmu3Wrapper::checkInput(const std::string& name, std::string& err_msg) const {
    fmi3ValueReference vr;
    if (!m_fmu.GetValueReference(name, vr)) {
        err_msg = "[ChFmu3Wrapper::checkInput] variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = m_fmu.GetVariablesList().at(vr);
    auto var_type = var.GetType();

    bool ok = (var_type == FmuVariable::Type::Float32 || var_type == FmuVariable::Type::Float64) &&  //
              var.GetCausality() == FmuVariable::CausalityType::input &&                             //
              var.GetVariability() == FmuVariable::VariabilityType::continuous;

    if (!ok) {
        err_msg = "[ChFmu3Wrapper::checkInput] illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Check that an FMU variable with given name is a "parameter" with specified type
bool ChFmu3Wrapper::checkParamReal(const std::string& name, std::string& err_msg) const {
    fmi3ValueReference vr;
    if (!m_fmu.GetValueReference(name, vr)) {
        err_msg = "[ChFmu3Wrapper::checkParamReal] variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = m_fmu.GetVariablesList().at(vr);
    auto var_type = var.GetType();

    bool ok = (var_type == FmuVariable::Type::Float32 || var_type == FmuVariable::Type::Float64) &&  //
              !var.IsState() &&                                                                      //
              var.GetVariability() != FmuVariable::VariabilityType::constant &&                      //
              (var.GetInitial() == FmuVariable::InitialType::exact ||
               var.GetCausality() == FmuVariable::CausalityType::input);

    if (!ok) {
        err_msg = "[ChFmu3Wrapper::checkParamReal] illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Check that an FMU variable with given name is a "parameter" with specified type
bool ChFmu3Wrapper::checkParamInt(const std::string& name, std::string& err_msg) const {
    fmi3ValueReference vr;
    if (!m_fmu.GetValueReference(name, vr)) {
        err_msg = "[ChFmu3Wrapper::checkParamInt] variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = m_fmu.GetVariablesList().at(vr);
    auto var_type = var.GetType();

    bool ok = (var_type == FmuVariable::Type::Int32 || var_type == FmuVariable::Type::UInt32 ||
               var_type == FmuVariable::Type::Int64 || var_type == FmuVariable::Type::UInt64) &&  //
              !var.IsState() &&                                                                   //
              var.GetVariability() != FmuVariable::VariabilityType::constant &&                   //
              (var.GetInitial() == FmuVariable::InitialType::exact ||
               var.GetCausality() == FmuVariable::CausalityType::input);

    if (!ok) {
        err_msg = "[ChFmu3Wrapper::checkParamInt] illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

void ChFmu3Wrapper::SetInputs(const std::unordered_map<std::string, double>& inputs_real) {
    for (const auto& v : inputs_real) {
        m_fmu.SetVariable(v.first, v.second);
    }
}

void ChFmu3Wrapper::SetContinuousStates(const std::vector<double>& states) {
    m_fmu.SetContinuousStates(states.data(), states.size());
}

void ChFmu3Wrapper::GetContinuousStates(std::vector<double>& states) {
    m_fmu.GetContinuousStates(states.data(), states.size());
}

void ChFmu3Wrapper::GetContinuousDerivatives(std::vector<double>& derivs) {
    m_fmu.GetContinuousStateDerivatives(derivs.data(), derivs.size());
}

void ChFmu3Wrapper::PrintFmuVariables() const {
    const auto& variables = m_fmu.GetVariablesList();
    for (const auto& v : variables) {
        std::cout << v.second.GetName() << "\t";
        std::cout << "type: " << FmuVariable::Type_toString(v.second.GetType()) << "  val ref: " << v.first << "\t";
        std::cout << "state? " << v.second.IsState() << "  deriv? " << v.second.IsDeriv() << "\t";
        std::cout << std::endl;
    }
}

}  // end namespace chrono
