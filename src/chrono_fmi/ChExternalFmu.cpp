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

#include "chrono_fmi/fmi2/ChFmuToolsImport.h"
#include "chrono_fmi/fmi3/ChFmuToolsImport.h"

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
    using FmuVariable = fmu_forge::fmi2::FmuVariable;

    virtual void SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) override;
    virtual void SetTime(double time) override;
    virtual unsigned int GetNumStates() const override;
    virtual std::unordered_set<std::string> GetStatesList() const override;
    virtual std::unordered_set<std::string> GetRealParametersList() const override;
    virtual std::unordered_set<std::string> GetIntParametersList() const override;
    virtual std::unordered_set<std::string> GetRealInputsList() const override;
    virtual double GetRealVariable(const std::string& name) override;
    virtual int GetIntVariable(const std::string& name) override;
    virtual ChVector3d GetVecVariable(const std::string& name) override;
    virtual ChQuaterniond GetQuatVariable(const std::string& name) override;
    virtual ChCoordsysd GetCoordsysVariable(const std::string& name) override;
    virtual ChFrame<> GetFrameVariable(const std::string& name) override;
    virtual ChFrameMoving<> GetFrameMovingVariable(const std::string& name) override;
    virtual void SetRealVariable(const std::string& name, double val) override;
    virtual void SetIntVariable(const std::string& name, int val) override;
    virtual void SetVecVariable(const std::string& name, const ChVector3d& val) override;
    virtual void SetQuatVariable(const std::string& name, const ChQuaterniond& val) override;
    virtual void SetCoordsysVariable(const std::string& name, const ChCoordsysd& val) override;
    virtual void SetFrameVariable(const std::string& name, const ChFrame<>& val) override;
    virtual void SetFrameMovingVariable(const std::string& name, const ChFrameMoving<>& val) override;
    virtual void Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                            const std::unordered_map<std::string, double>& parameters_real,
                            const std::unordered_map<std::string, int>& parameters_int,
                            const std::unordered_map<std::string, std::string>& parameters_string) override;
    virtual bool checkState(const std::string& name, std::string& err_msg) const override;
    virtual bool checkInput(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamReal(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamInt(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamString(const std::string& name, std::string& err_msg) const override;
    virtual void SetInputs(const std::unordered_map<std::string, double>& inputs_real) override;
    virtual void SetContinuousStates(const std::vector<double>& states) override;
    virtual void GetContinuousStates(std::vector<double>& states) override;
    virtual void GetContinuousDerivatives(std::vector<double>& derivs) override;
    virtual void PrintFmuVariables() const override;

    bool checkParam(const std::string& name, FmuVariable::Type type, std::string& err_msg) const;

    fmi2::FmuChronoUnit m_fmu;
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
    using FmuVariable = fmu_forge::fmi3::FmuVariable;

    virtual void SetDebugLogging(bool logging, const std::vector<std::string>& log_categories) override;
    virtual void SetTime(double time) override;
    virtual unsigned int GetNumStates() const override;
    virtual std::unordered_set<std::string> GetStatesList() const override;
    virtual std::unordered_set<std::string> GetRealParametersList() const override;
    virtual std::unordered_set<std::string> GetIntParametersList() const override;
    virtual std::unordered_set<std::string> GetRealInputsList() const override;
    virtual double GetRealVariable(const std::string& name) override;
    virtual int GetIntVariable(const std::string& name) override;
    virtual ChVector3d GetVecVariable(const std::string& name) override;
    virtual ChQuaterniond GetQuatVariable(const std::string& name) override;
    virtual ChCoordsysd GetCoordsysVariable(const std::string& name) override;
    virtual ChFrame<> GetFrameVariable(const std::string& name) override;
    virtual ChFrameMoving<> GetFrameMovingVariable(const std::string& name) override;
    virtual void SetRealVariable(const std::string& name, double val) override;
    virtual void SetIntVariable(const std::string& name, int val) override;
    virtual void SetVecVariable(const std::string& name, const ChVector3d& val) override;
    virtual void SetQuatVariable(const std::string& name, const ChQuaterniond& val) override;
    virtual void SetCoordsysVariable(const std::string& name, const ChCoordsysd& val) override;
    virtual void SetFrameVariable(const std::string& name, const ChFrame<>& val) override;
    virtual void SetFrameMovingVariable(const std::string& name, const ChFrameMoving<>& val) override;
    virtual void Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                            const std::unordered_map<std::string, double>& parameters_real,
                            const std::unordered_map<std::string, int>& parameters_int,
                            const std::unordered_map<std::string, std::string>& parameters_string) override;
    virtual bool checkState(const std::string& name, std::string& err_msg) const override;
    virtual bool checkInput(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamReal(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamInt(const std::string& name, std::string& err_msg) const override;
    virtual bool checkParamString(const std::string& name, std::string& err_msg) const override;
    virtual void SetInputs(const std::unordered_map<std::string, double>& inputs_real) override;
    virtual void SetContinuousStates(const std::vector<double>& states) override;
    virtual void GetContinuousStates(std::vector<double>& states) override;
    virtual void GetContinuousDerivatives(std::vector<double>& derivs) override;
    virtual void PrintFmuVariables() const override;

    fmi3::FmuChronoUnit m_fmu;
};

// =============================================================================
// Implementation of a ChExternalDynamicsODE component that wraps an FMU

ChExternalFmu::ChExternalFmu() : m_verbose(false), m_initialized(false), m_num_states(0) {}

void ChExternalFmu::Load(const std::string& instance_name,  // name of the FMU instance
                         const std::string& fmu_filename,   // name of the FMU file
                         const std::string& unpack_dir,     // unpack directory
                         bool logging,                      // enable FMU logging
                         const std::string& resources_dir   // location of FMU resources
) {
    // Peek in FMU model description file to get FMI version
    auto fmi_version = fmu_forge::GetFmuVersion(fmu_filename);

    // Create an FMU wrapper of appropriate type
    try {
        switch (fmi_version) {
            case fmu_forge::FmuVersion::FMI2:
                if (m_verbose)
                    std::cout << "\nLoading FMI 2.0 FMU: " << fmu_filename << "\n" << std::endl;
                m_wrapper = chrono_types::make_unique<ChFmu2Wrapper>(instance_name, fmu_filename, unpack_dir, logging,
                                                                     resources_dir, m_verbose);
                break;
            case fmu_forge::FmuVersion::FMI3:
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

double ChExternalFmu::GetRealVariable(const std::string& name) const {
    return m_wrapper->GetRealVariable(name);
}

int ChExternalFmu::GetIntVariable(const std::string& name) const {
    return m_wrapper->GetIntVariable(name);
}

ChVector3d ChExternalFmu::GetVecVariable(const std::string& name) const {
    return m_wrapper->GetVecVariable(name);
}

ChQuaterniond ChExternalFmu::GetQuatVariable(const std::string& name) const {
    return m_wrapper->GetQuatVariable(name);
}

ChCoordsysd ChExternalFmu::GetCoordsysVariable(const std::string& name) const {
    return m_wrapper->GetCoordsysVariable(name);
}

ChFrame<> ChExternalFmu::GetFrameVariable(const std::string& name) const {
    return m_wrapper->GetFrameVariable(name);
}

ChFrameMoving<> ChExternalFmu::GetFrameMovingVariable(const std::string& name) const {
    return m_wrapper->GetFrameMovingVariable(name);
}

void ChExternalFmu::SetRealVariable(const std::string& name, double val) {
    m_wrapper->SetRealVariable(name, val);
}

void ChExternalFmu::SetIntVariable(const std::string& name, double val) {
    m_wrapper->SetIntVariable(name, val);
}

void ChExternalFmu::SetVecVariable(const std::string& name, const ChVector3d& val) {
    m_wrapper->SetVecVariable(name, val);
}

void ChExternalFmu::SetQuatVariable(const std::string& name, const ChQuaterniond& val) {
    m_wrapper->SetQuatVariable(name, val);
}

void ChExternalFmu::SetCoordsysVariable(const std::string& name, const ChCoordsysd& val) {
    m_wrapper->SetCoordsysVariable(name, val);
}

void ChExternalFmu::SetFrameVariable(const std::string& name, const ChFrame<>& val) {
    m_wrapper->SetFrameVariable(name, val);
}

void ChExternalFmu::SetFrameMovingVariable(const std::string& name, const ChFrameMoving<>& val) {
    m_wrapper->SetFrameMovingVariable(name, val);
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

void ChExternalFmu::SetStringParameterValue(const std::string& name, const std::string& value) {
    if (m_initialized) {
        std::cerr << "SetStringParameterValue cannot be called after Initialize()";
        throw std::runtime_error("SetStringParameterValue cannot be called after Initialize()");
    }

    std::string err_msg;
    bool ok = m_wrapper->checkParamString(name, err_msg);

    if (!ok) {
        std::cerr << "SetStringParameterValue: " + err_msg << std::endl;
        throw std::runtime_error("SetStringParameterValue: " + err_msg);
    }

    m_parameters_string.insert({name, value});
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

    m_input_functions.insert({name, function});
}

void ChExternalFmu::SetRealInputChFunction(const std::string& name, std::shared_ptr<ChFunction> function) {
    if (m_initialized) {
        std::cerr << "SetRealInputChFunction cannot be called after Initialize()";
        throw std::runtime_error("SetRealInputChFunction cannot be called after Initialize()");
    }

    std::string err_msg;
    bool ok = m_wrapper->checkInput(name, err_msg);

    if (!ok) {
        std::cerr << "SetRealInputChFunction: " + err_msg << std::endl;
        throw std::runtime_error("SetRealInputChFunction: " + err_msg);
    }

    m_input_chfunctions.insert({name, function});
}

void ChExternalFmu::Initialize() {
    // Let the wrapper initialize its underlying FMU
    m_wrapper->Initialize(m_initial_conditions, m_parameters_real, m_parameters_int, m_parameters_string);

    // Initialize the base class
    ChExternalDynamicsODE::Initialize();

    m_initialized = true;
}

// -----------------

void ChExternalFmu::SetInitialConditions(ChVectorDynamic<>& y0) {
    std::vector<double> states(m_num_states);
    m_wrapper->GetContinuousStates(states);

    // Load initial conditions for the ChExternalDynamicsODE component
    for (unsigned int i = 0; i < m_num_states; i++)
        y0(i) = states[i];
}

void ChExternalFmu::CalculateRHS(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& rhs) {
    // Load provided states and pass them to the FMU
    std::vector<double> states(m_num_states);
    for (unsigned int i = 0; i < m_num_states; i++)
        states[i] = y(i);
    m_wrapper->SetContinuousStates(states);

    // Get the RHS from the FMU and load for the ChExternalDynamicsODE component
    std::vector<double> derivs(m_num_states);
    m_wrapper->GetContinuousDerivatives(derivs);
    for (unsigned int i = 0; i < m_num_states; i++)
        rhs(i) = derivs[i];
}

void ChExternalFmu::Update(double time, bool update_assets) {
    m_wrapper->SetTime(time);

    // Collect input values at current time
    std::unordered_map<std::string, double> inputs_real;
    for (const auto& v : m_input_functions) {
        double value = v.second(time);
        inputs_real.insert({v.first, value});
    }
    for (const auto& v : m_input_chfunctions) {
        double value = v.second->GetVal(time);
        inputs_real.insert({v.first, value});
    }

    m_wrapper->SetInputs(inputs_real);

    // Invoke base class Update
    ChExternalDynamicsODE::Update(time, update_assets);
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

void ChFmu2Wrapper::SetTime(double time) {
    m_fmu.SetTime(time);
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

double ChFmu2Wrapper::GetRealVariable(const std::string& name) {
    double val;
    m_fmu.GetVariable(name, val, FmuVariable::Type::Real);
    return val;
}

int ChFmu2Wrapper::GetIntVariable(const std::string& name) {
    int val;
    m_fmu.GetVariable(name, val, FmuVariable::Type::Integer);
    return val;
}

ChVector3d ChFmu2Wrapper::GetVecVariable(const std::string& name) {
    ChVector3d val;
    m_fmu.GetVecVariable(name, val);
    return val;
}

ChQuaterniond ChFmu2Wrapper::GetQuatVariable(const std::string& name) {
    ChQuaterniond val;
    m_fmu.GetQuatVariable(name, val);
    return val;
}

ChCoordsysd ChFmu2Wrapper::GetCoordsysVariable(const std::string& name) {
    ChCoordsysd val;
    m_fmu.GetCoordsysVariable(name, val);
    return val;
}

ChFrame<> ChFmu2Wrapper::GetFrameVariable(const std::string& name) {
    ChFrame<> val;
    m_fmu.GetFrameVariable(name, val);
    return val;
}

ChFrameMoving<> ChFmu2Wrapper::GetFrameMovingVariable(const std::string& name) {
    ChFrameMoving<> val;
    m_fmu.GetFrameMovingVariable(name, val);
    return val;
}

void ChFmu2Wrapper::SetRealVariable(const std::string& name, double val) {
    m_fmu.SetVariable(name, val, FmuVariable::Type::Real);
}

void ChFmu2Wrapper::SetIntVariable(const std::string& name, int val) {
    m_fmu.SetVariable(name, val, FmuVariable::Type::Integer);
}

void ChFmu2Wrapper::SetVecVariable(const std::string& name, const ChVector3d& val) {
    m_fmu.SetVecVariable(name, val);
}

void ChFmu2Wrapper::SetQuatVariable(const std::string& name, const ChQuaterniond& val) {
    m_fmu.SetQuatVariable(name, val);
}

void ChFmu2Wrapper::SetCoordsysVariable(const std::string& name, const ChCoordsysd& val) {
    m_fmu.SetCoordsysVariable(name, val);
}

void ChFmu2Wrapper::SetFrameVariable(const std::string& name, const ChFrame<>& val) {
    m_fmu.SetFrameVariable(name, val);
}

void ChFmu2Wrapper::SetFrameMovingVariable(const std::string& name, const ChFrameMoving<>& val) {
    m_fmu.SetFrameMovingVariable(name, val);
}

// Initialize underlying FMU
void ChFmu2Wrapper::Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                               const std::unordered_map<std::string, double>& parameters_real,
                               const std::unordered_map<std::string, int>& parameters_int,
                               const std::unordered_map<std::string, std::string>& parameters_string) {
    m_fmu.EnterInitializationMode();

    // Set initial conditions
    for (const auto& v : initial_conditions)
        m_fmu.SetVariable(v.first, v.second, FmuVariable::Type::Real);

    // Set real and integer parameters
    for (const auto& v : parameters_real)
        m_fmu.SetVariable(v.first, v.second, FmuVariable::Type::Real);
    for (const auto& v : parameters_int)
        m_fmu.SetVariable(v.first, v.second, FmuVariable::Type::Integer);
    for (const auto& v : parameters_string)
        m_fmu.SetVariable(v.first, v.second);

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

// Check that the FMU variable with given name is a string "parameter"
bool ChFmu2Wrapper::checkParamString(const std::string& name, std::string& err_msg) const {
    return checkParam(name, FmuVariable::Type::String, err_msg);
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
            m_fmu.Load(fmu_forge::fmi3::FmuType::MODEL_EXCHANGE, fmu_filename);
        else
            m_fmu.Load(fmu_forge::fmi3::FmuType::MODEL_EXCHANGE, fmu_filename, unpack_dir);
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

void ChFmu3Wrapper::SetTime(double time) {
    m_fmu.SetTime(time);
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

double ChFmu3Wrapper::GetRealVariable(const std::string& name) {
    double val;
    m_fmu.GetVariable(name, val);
    return val;
}

int ChFmu3Wrapper::GetIntVariable(const std::string& name) {
    int val;
    m_fmu.GetVariable(name, val);
    return val;
}

ChVector3d ChFmu3Wrapper::GetVecVariable(const std::string& name) {
    ChVector3d val;
    m_fmu.GetVecVariable(name, val);
    return val;
}

ChQuaterniond ChFmu3Wrapper::GetQuatVariable(const std::string& name) {
    ChQuaterniond val;
    m_fmu.GetQuatVariable(name, val);
    return val;
}

ChCoordsysd ChFmu3Wrapper::GetCoordsysVariable(const std::string& name) {
    ChCoordsysd val;
    m_fmu.GetCoordsysVariable(name, val);
    return val;
}

ChFrame<> ChFmu3Wrapper::GetFrameVariable(const std::string& name) {
    ChFrame<> val;
    m_fmu.GetFrameVariable(name, val);
    return val;
}

ChFrameMoving<> ChFmu3Wrapper::GetFrameMovingVariable(const std::string& name) {
    ChFrameMoving<> val;
    m_fmu.GetFrameMovingVariable(name, val);
    return val;
}

void ChFmu3Wrapper::SetRealVariable(const std::string& name, double val) {
    m_fmu.SetVariable(name, val);
}

void ChFmu3Wrapper::SetIntVariable(const std::string& name, int val) {
    m_fmu.SetVariable(name, val);
}

void ChFmu3Wrapper::SetVecVariable(const std::string& name, const ChVector3d& val) {
    m_fmu.SetVecVariable(name, val);
}

void ChFmu3Wrapper::SetQuatVariable(const std::string& name, const ChQuaterniond& val) {
    m_fmu.SetQuatVariable(name, val);
}

void ChFmu3Wrapper::SetCoordsysVariable(const std::string& name, const ChCoordsysd& val) {
    m_fmu.SetCoordsysVariable(name, val);
}

void ChFmu3Wrapper::SetFrameVariable(const std::string& name, const ChFrame<>& val) {
    m_fmu.SetFrameVariable(name, val);
}

void ChFmu3Wrapper::SetFrameMovingVariable(const std::string& name, const ChFrameMoving<>& val) {
    m_fmu.SetFrameMovingVariable(name, val);
}

// Initialize underlying FMU
void ChFmu3Wrapper::Initialize(const std::unordered_map<std::string, double>& initial_conditions,
                               const std::unordered_map<std::string, double>& parameters_real,
                               const std::unordered_map<std::string, int>& parameters_int,
                               const std::unordered_map<std::string, std::string>& parameters_string) {
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
    for (const auto& v : parameters_string)
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

// Check that an FMU variable with given name is a real type "parameter"
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

// Check that an FMU variable with given name is an integer type "parameter"
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

// Check that the FMU variable with given name is a string type "parameter"
bool ChFmu3Wrapper::checkParamString(const std::string& name, std::string& err_msg) const {
    //// TODO
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
