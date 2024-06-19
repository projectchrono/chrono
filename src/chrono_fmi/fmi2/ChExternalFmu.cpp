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

#include "chrono_fmi/fmi2/ChExternalFmu.h"

#include "fmi2/FmuToolsImport.h"

namespace chrono {

ChExternalFmu::ChExternalFmu(const std::string& instance_name,
                             const std::string& fmu_filename,
                             const std::string& unpack_dir,
                             bool logging,
                             const std::string& resources_dir)
    : m_initialized(false), m_num_states(0) {
    // Create the underlying FMU
    m_fmu = chrono_types::make_unique<FmuUnit>();
    ////m_fmu->SetVerbose(true);

    // Load the FMU from the specified file
    try {
        if (unpack_dir.empty())
            m_fmu->Load(fmi2Type::fmi2ModelExchange, fmu_filename);
        else
            m_fmu->Load(fmi2Type::fmi2ModelExchange, fmu_filename, unpack_dir);
    } catch (std::exception& my_exception) {
        std::cout << "ERROR loading FMU: " << my_exception.what() << "\n";
    }

    // Instantiate the FMU
    if (resources_dir.empty())
        m_fmu->Instantiate(instance_name, logging, false);
    else
        m_fmu->Instantiate(instance_name, resources_dir, logging, false);

    // Set up an experiment
    m_fmu->SetupExperiment(fmi2False,  // no tolerance defined
                           0.0,        // tolerance (dummy)
                           0.0,        // start time
                           fmi2False,  // do not use stop time
                           1.0         // stop time (dummy)
    );

    // Extract the number of states
    m_num_states = (unsigned int)m_fmu->GetNumStates();
}

ChExternalFmu::~ChExternalFmu() {}

void ChExternalFmu::SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) {
    if (logging)
        m_fmu->SetDebugLogging(fmi2True, log_categories);
}

// Check that an FMU variable with given name is a state
bool ChExternalFmu::checkState(const std::string& name, std::string& err_msg) const {
    if (m_initialized) {
        err_msg = "cannot be called after Initialize()";
        return false;
    }

    const auto& variables = m_fmu->GetVariablesList();
    auto search = variables.find(name);
    if (search == variables.end()) {
        err_msg = "variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = search->second;
    bool ok = var.GetType() == FmuVariable::Type::Real && var.IsState();

    if (!ok) {
        err_msg = "illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Check that an FMU variable with given name is a "parameter"
bool ChExternalFmu::checkParam(const std::string& name, FmuVariable::Type type, std::string& err_msg) const {
    if (m_initialized) {
        err_msg = "cannot be called after Initialize()";
        return false;
    }

    const auto& variables = m_fmu->GetVariablesList();
    auto search = variables.find(name);
    if (search == variables.end()) {
        err_msg = "variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = search->second;
    bool ok = var.GetType() == type && !var.IsState() &&
              var.GetVariability() != FmuVariable::VariabilityType::constant &&
              (var.GetInitial() == FmuVariable::InitialType::exact ||
               var.GetCausality() == FmuVariable::CausalityType::input);

    if (!ok) {
        err_msg = "illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Check that an FMU variable with given name is an "input"
bool ChExternalFmu::checkInput(const std::string& name, FmuVariable::Type type, std::string& err_msg) const {
    if (m_initialized) {
        err_msg = "cannot be called after Initialize()";
        return false;
    }

    const auto& variables = m_fmu->GetVariablesList();
    auto search = variables.find(name);
    if (search == variables.end()) {
        err_msg = "variable '" + name + "' does not exist";
        return false;
    }

    const auto& var = search->second;
    bool ok = var.GetType() == type && var.GetCausality() == FmuVariable::CausalityType::input &&
              var.GetVariability() == FmuVariable::VariabilityType::continuous;

    if (!ok) {
        err_msg = "illegal to set variable '" + name + "'";
        return false;
    }

    return true;
}

// Extract names of all FMU state variables
std::unordered_set<std::string> ChExternalFmu::GetStatesList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu->GetVariablesList()) {
        bool ok = checkState(v.first, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

// Extract names of all FMU real "parameters"
std::unordered_set<std::string> ChExternalFmu::GetRealParametersList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu->GetVariablesList()) {
        bool ok = checkParam(v.first, FmuVariable::Type::Real, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

// Extract names of all FMU integer "parameters"
std::unordered_set<std::string> ChExternalFmu::GetIntParametersList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu->GetVariablesList()) {
        bool ok = checkParam(v.first, FmuVariable::Type::Integer, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

// Extract names of all FMU real "inputs"
std::unordered_set<std::string> ChExternalFmu::GetRealInputsList() const {
    std::unordered_set<std::string> list;
    std::string err_msg;

    for (const auto& v : m_fmu->GetVariablesList()) {
        bool ok = checkInput(v.first, FmuVariable::Type::Real, err_msg);
        if (ok)
            list.insert(v.first);
    }

    return list;
}

void ChExternalFmu::SetInitialCondition(const std::string& name, double value) {
    std::string err_msg;
    bool ok = checkState(name, err_msg);
    if (!ok) {
        std::cerr << "SetInitialCondition: " + err_msg << std::endl;
        throw std::runtime_error("SetInitialCondition: " + err_msg);
    }

    m_initial_conditions.insert({name, value});
}

void ChExternalFmu::SetRealParameterValue(const std::string& name, double value) {
    std::string err_msg;
    bool ok = checkParam(name, FmuVariable::Type::Real, err_msg);
    if (!ok) {
        std::cerr << "SetRealParameterValue: " + err_msg << std::endl;
        throw std::runtime_error("SetRealParameterValue: " + err_msg);
    }

    m_parameters_real.insert({name, value});
}

void ChExternalFmu::SetIntParameterValue(const std::string& name, int value) {
    std::string err_msg;
    bool ok = checkParam(name, FmuVariable::Type::Integer, err_msg);
    if (!ok) {
        std::cerr << "SetIntParameterValue: " + err_msg << std::endl;
        throw std::runtime_error("SetIntParameterValue: " + err_msg);
    }

    m_parameters_int.insert({name, value});
}

void ChExternalFmu::SetRealInputFunction(const std::string& name, std::function<double(double)> function) {
    std::string err_msg;
    bool ok = checkInput(name, FmuVariable::Type::Real, err_msg);
    if (!ok) {
        std::cerr << "SetContinuousInputFunction: " + err_msg << std::endl;
        throw std::runtime_error("SetContinuousInputFunction: " + err_msg);
    }

    m_inputs_real.insert({name, function});
}

void ChExternalFmu::PrintFmuVariables() const {
    std::cout << "FMU variables" << std::endl;
    const auto& variables = m_fmu->GetVariablesList();
    for (const auto& v : variables) {
        std::cout << v.first << "\t";
        std::cout << "type: " << FmuVariable::Type_toString(v.second.GetType()) << "  index: " << v.second.GetIndex()
                  << "\t";
        std::cout << "state? " << v.second.IsState() << "  deriv? " << v.second.IsDeriv() << "\t";
        std::cout << std::endl;
    }
}

void ChExternalFmu::Initialize() {
    m_fmu->EnterInitializationMode();

    // Set initial conditions
    for (const auto& v : m_initial_conditions)
        m_fmu->SetVariable(v.first, v.second, FmuVariable::Type::Real);

    // Set real and integer parameters
    for (const auto& v : m_parameters_real)
        m_fmu->SetVariable(v.first, v.second, FmuVariable::Type::Real);
    for (const auto& v : m_parameters_int)
        m_fmu->SetVariable(v.first, v.second, FmuVariable::Type::Integer);

    m_fmu->ExitInitializationMode();

    // Initialize the base class
    ChExternalDynamics::Initialize();

    m_initialized = true;
}

void ChExternalFmu::SetInitialConditions(ChVectorDynamic<>& y0) {
    // Get initial conditions from the FMU
    std::vector<fmi2Real> states(m_num_states);
    m_fmu->GetContinuousStates(states.data(), m_num_states);

    // Load initial conditions for the ChExternalDynamics component
    for (unsigned int i = 0; i < m_num_states; i++)
        y0(i) = states[i];
}

void ChExternalFmu::CalculateRHS(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& rhs) {
    // Set the states in the FMU
    std::vector<fmi2Real> states(m_num_states);
    for (unsigned int i = 0; i < m_num_states; i++)
        states[i] = y(i);
    m_fmu->SetContinuousStates(states.data(), m_num_states);

    // Get the RHS from the FMU
    std::vector<fmi2Real> derivs(m_num_states);
    m_fmu->GetDerivatives(derivs.data(), m_num_states);

    // Load RHS for the ChExternalDynamics component
    for (unsigned int i = 0; i < m_num_states; i++)
        rhs(i) = derivs[i];
}

void ChExternalFmu::Update(double time, bool update_assets) {
    // Set FMU inputs at current time
    for (const auto& v : m_inputs_real) {
        double value = v.second(time);
        m_fmu->SetVariable(v.first, value, FmuVariable::Type::Real);
    }

    // Invoke base class Update
    ChExternalDynamics::Update(time, update_assets);
}

}  // end namespace chrono
