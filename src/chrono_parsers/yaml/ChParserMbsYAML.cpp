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

//// TODO
//// - perform checks during loading of YAML file or use a 3rd party YAML schema validation?
//// - add definition functions for link-type components using 2 local frames (alternative ChLink initialization)
//// - add support for other constraints (composite joints: rev-sph and rev-prismatic)
//// - add support for point-point actuators (hydraulic, FMU, external)
//// - what is the best way to deal with collision families?
//// - complete FEA parsing

#include <algorithm>
#include <filesystem>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono/physics//ChSystemNSC.h"
#include "chrono/physics//ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#ifdef CHRONO_FEA
    #include "chrono/fea/ChBuilderBeam.h"

    #include "chrono/fea/ChBeamSectionEuler.h"
    #include "chrono/fea/ChBeamSectionCableANCF.h"
    #include "chrono/fea/ChBeamSectionCosserat.h"
    #include "chrono/fea/ChBeamSectionTaperedTimoshenko.h"

    #include "chrono/fea/ChMaterialBeamANCF.h"
    #include "chrono/fea/ChMaterialHexaANCF.h"
    #include "chrono/fea/ChMaterialShellANCF.h"
    #include "chrono/fea/ChMaterialShellKirchhoff.h"
    #include "chrono/fea/ChMaterialShellReissner.h"

    #include "chrono/fea/ChLinkNodeFrame.h"
    #include "chrono/fea/ChLinkNodeSlopeFrame.h"
    #include "chrono/fea/ChLinkNodeNode.h"
    #include "chrono/fea/ChLinkNodeFace.h"
#endif

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChForceFunctors.h"

#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_parsers/yaml/ChParserMbsYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserMbsYAML::ChParserMbsYAML(bool verbose) : ChParserYAML(), m_loaded(false), m_solver_loaded(false), m_model_loaded(false), m_crt_instance(-1) {
    SetVerbose(verbose);
}

ChParserMbsYAML::ChParserMbsYAML(const std::string& yaml_filename, bool verbose) : ChParserYAML(), m_solver_loaded(false), m_model_loaded(false), m_crt_instance(-1) {
    SetVerbose(verbose);
    LoadFile(yaml_filename);
}

ChParserMbsYAML::~ChParserMbsYAML() {}

// -----------------------------------------------------------------------------

void ChParserMbsYAML::LoadFile(const std::string& yaml_filename) {
    YAML::Node yaml;

    // Load MBS YAML file
    yaml = YAML::LoadFile(yaml_filename);
    m_file_handler.SetReferenceDirectory(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check the YAML file if of type "MBS"
    ChAssertAlways(yaml["type"]);
    auto type = ReadYamlFileType(yaml["type"]);
    ChAssertAlways(type == ChParserYAML::YamlFileType::MBS);

    // Load simulation, output, and run-time visualization data
    LoadSimData(yaml);

    // Load MBS model YAML file
    {
        ChAssertAlways(yaml["model"]);
        auto model_fname = yaml["model"].as<std::string>();
        auto model_filename = m_file_handler.GetReferenceDirectory() + "/" + model_fname;
        auto path = std::filesystem::path(model_filename);
        if (!exists(path) || !is_regular_file(path)) {
            cerr << "Error: file '" << model_filename << "' not found." << endl;
            throw std::runtime_error("File not found");
        }
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\n[ChParserMbsYAML] Loading Chrono MBS model from: '" << model_filename << "'\n" << endl;
        }
        auto model = YAML::LoadFile(model_filename);
        ChAssertAlways(model["chrono-version"]);
        CheckVersion(model["chrono-version"]);
        LoadModelData(model);
    }

    // Load solver YAML file
    {
        ChAssertAlways(yaml["solver"]);
        auto solver_fname = yaml["solver"].as<std::string>();
        auto solver_filename = m_file_handler.GetReferenceDirectory() + "/" + solver_fname;
        auto path = std::filesystem::path(solver_filename);
        if (!exists(path) || !is_regular_file(path)) {
            cerr << "Error: file '" << solver_filename << "' not found." << endl;
            throw std::runtime_error("File not found");
        }
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\n[ChParserMbsYAML] Loading Chrono MBS solver from: " << solver_filename << "\n" << endl;
        }
        auto solver = YAML::LoadFile(solver_filename);
        ChAssertAlways(solver["chrono-version"]);
        CheckVersion(solver["chrono-version"]);
        LoadSolverData(solver);
    }

    if (m_verbose) {
        m_sim.PrintInfo();
        cout << endl;
        m_vis_settings.PrintInfo();
        cout << "body visualization type: " << utils::ChBodyGeometry::GetVisualizationTypeAsString(m_vis_type) << endl;
        cout << endl;
        m_output_settings.PrintInfo();
    }

    m_loaded = true;
}

void ChParserMbsYAML::LoadSimData(const YAML::Node& yaml) {
    // Read common simulation settings
    ChParserYAML::LoadSimData(yaml);

    // Simulation settings (required)
    {
        ChAssertAlways(yaml["simulation"]);
        auto sim = yaml["simulation"];
        if (sim["end_time"])
            m_sim.end_time = sim["end_time"].as<double>();
        if (sim["enforce_realtime"])
            m_sim.enforce_realtime = sim["enforce_realtime"].as<bool>();
        if (sim["gravity"])
            m_sim.gravity = ReadVector(sim["gravity"]);
    }

    // MBS-specific run-time visualization settings (optional)
    if (yaml["visualization"]) {
        if (yaml["visualization"]["type"])
            m_vis_type = ReadVisualizationType(yaml["visualization"]["type"]);
    }
}

void ChParserMbsYAML::LoadSolverData(const YAML::Node& yaml) {
    // Mandatory
    ChAssertAlways(yaml["contact_method"]);
    auto contact_method = ChToUpper(yaml["contact_method"].as<std::string>());
    if (contact_method == "SMC") {
        m_sim.contact_method = ChContactMethod::SMC;
    } else if (contact_method == "NSC") {
        m_sim.contact_method = ChContactMethod::NSC;
    } else {
        cerr << "Incorrect contact method: " << yaml["contact_method"].as<std::string>() << endl;
        throw std::runtime_error("Incorrect contact method");
    }

    // Integrator parameters (optional)
    if (yaml["integrator"]) {
        auto intgr = yaml["integrator"];
        ChAssertAlways(intgr["type"]);
        ChAssertAlways(intgr["time_step"]);
        m_sim.integrator.type = ReadIntegratorType(intgr["type"]);
        m_sim.integrator.time_step = intgr["time_step"].as<double>();
        switch (m_sim.integrator.type) {
            case ChTimestepper::Type::HHT:
                if (intgr["rel_tolerance"])
                    m_sim.integrator.rtol = intgr["rel_tolerance"].as<double>();
                if (intgr["abs_tolerance_states"])
                    m_sim.integrator.atol_states = intgr["abs_tolerance_states"].as<double>();
                if (intgr["abs_tolerance_multipliers"])
                    m_sim.integrator.atol_multipliers = intgr["abs_tolerance_multipliers"].as<double>();
                if (intgr["max_iterations"])
                    m_sim.integrator.max_iterations = intgr["max_iterations"].as<double>();
                if (intgr["use_stepsize_control"])
                    m_sim.integrator.use_stepsize_control = intgr["use_stepsize_control"].as<bool>();
                if (intgr["use_modified_newton"])
                    m_sim.integrator.use_modified_newton = intgr["use_modified_newton"].as<bool>();
                break;
            case ChTimestepper::Type::EULER_IMPLICIT:
                if (intgr["rel_tolerance"])
                    m_sim.integrator.rtol = intgr["rel_tolerance"].as<double>();
                if (intgr["abs_tolerance_states"])
                    m_sim.integrator.atol_states = intgr["abs_tolerance_states"].as<double>();
                if (intgr["abs_tolerance_multipliers"])
                    m_sim.integrator.atol_multipliers = intgr["abs_tolerance_multipliers"].as<double>();
                if (intgr["max_iterations"])
                    m_sim.integrator.max_iterations = intgr["max_iterations"].as<double>();
                break;
        }
    }

    // Solver parameters (optional)
    if (yaml["solver"]) {
        auto slvr = yaml["solver"];
        ChAssertAlways(slvr["type"]);
        m_sim.solver.type = ReadSolverType(slvr["type"]);
        switch (m_sim.solver.type) {
            case ChSolver::Type::SPARSE_LU:
            case ChSolver::Type::SPARSE_QR:
                if (slvr["lock_sparsity_pattern"])
                    m_sim.solver.lock_sparsity_pattern = slvr["lock_sparsity_pattern"].as<bool>();
                if (slvr["use_sparsity_pattern_learner"])
                    m_sim.solver.use_sparsity_pattern_learner = slvr["use_sparsity_pattern_learner"].as<bool>();
                break;
            case ChSolver::Type::BARZILAIBORWEIN:
            case ChSolver::Type::APGD:
            case ChSolver::Type::PSOR:
                if (slvr["max_iterations"])
                    m_sim.solver.max_iterations = slvr["max_iterations"].as<int>();
                if (slvr["overrelaxation_factor"])
                    m_sim.solver.overrelaxation_factor = slvr["overrelaxation_factor"].as<double>();
                if (slvr["sharpness_factor"])
                    m_sim.solver.sharpness_factor = slvr["sharpness_factor"].as<double>();
                if (slvr["enable_diagonal_preconditioner"])
                    m_sim.solver.enable_diagonal_preconditioner = slvr["enable_diagonal_preconditioner"].as<bool>();
                if (slvr["warm_start"])
                    m_sim.solver.warm_start = slvr["warm_start"].as<bool>();
                break;
            case ChSolver::Type::BICGSTAB:
            case ChSolver::Type::MINRES:
            case ChSolver::Type::GMRES:
                if (slvr["max_iterations"])
                    m_sim.solver.max_iterations = slvr["max_iterations"].as<int>();
                if (slvr["tolerance"])
                    m_sim.solver.tolerance = slvr["tolerance"].as<double>();
                if (slvr["enable_diagonal_preconditioner"])
                    m_sim.solver.enable_diagonal_preconditioner = slvr["enable_diagonal_preconditioner"].as<bool>();
                if (slvr["warm_start"])
                    m_sim.solver.warm_start = slvr["warm_start"].as<bool>();
                break;
        }
    }

    m_solver_loaded = true;
}

void ChParserMbsYAML::LoadModelData(const YAML::Node& yaml) {
    // Check a model object exists
    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    if (model["name"])
        m_name = model["name"].as<std::string>();

    if (model["angle_degrees"])
        m_use_degrees = model["angle_degrees"].as<bool>();

    m_file_handler.Read(model);

    if (m_verbose) {
        cout << "model name: '" << m_name << "'" << endl;
        cout << "angles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
        m_file_handler.PrintInfo();
    }

    if (model["bodies"])
        LoadBodies(model["bodies"]);

    if (model["joints"])
        LoadJoints(model["joints"]);

    if (model["constraints"])
        LoadConstraints(model["constraints"]);

    if (model["tsdas"])
        LoadTSDAs(model["tsdas"]);

    if (model["rsdas"])
        LoadRSDAs(model["rsdas"]);

    if (model["body_loads"])
        LoadBodyLoads(model["body_loads"]);

    if (model["load_controllers"])
        LoadControllers(model["load_controllers"]);

    if (model["motors"])
        LoadMotors(model["motors"]);

    if (model["FEA"]) {
#ifdef CHRONO_FEA
        auto fea = model["FEA"];
        if (fea["beams"])
            LoadFEABeams(fea);
        if (fea["shells"])
            LoadFEAShells(fea);
        if (fea["constraints"])
            LoadFEAConstraints(fea);
#else
        cerr << "Chrono::FEA not enabled. All FEA components will be ignored." << endl;
#endif
    }

    m_model_loaded = true;
}

void ChParserMbsYAML::LoadBodies(const YAML::Node& bodies) {
    ChAssertAlways(bodies.IsSequence());
    if (m_verbose)
        cout << "\nbodies: " << bodies.size() << endl;

    for (size_t i = 0; i < bodies.size(); i++) {
        ChAssertAlways(bodies[i]["name"]);
        auto name = bodies[i]["name"].as<std::string>();

        BodyParams body;

        if (bodies[i]["fixed"])
            body.is_fixed = bodies[i]["fixed"].as<bool>();

        ChAssertAlways(bodies[i]["location"]);
        ChAssertAlways(body.is_fixed || bodies[i]["mass"]);
        ChAssertAlways(body.is_fixed || bodies[i]["inertia"]);
        ChAssertAlways(body.is_fixed || bodies[i]["inertia"]["moments"]);

        body.pos = ReadVector(bodies[i]["location"]);
        if (bodies[i]["orientation"])
            body.rot = ReadRotation(bodies[i]["orientation"], m_use_degrees);
        if (bodies[i]["initial_linear_velocity"])
            body.lin_vel = ReadVector(bodies[i]["initial_linear_velocity"]);
        if (bodies[i]["initial_angular_velocity"])
            body.ang_vel = ReadVector(bodies[i]["initial_angular_velocity"]);

        if (!body.is_fixed)
            body.mass = bodies[i]["mass"].as<double>();
        if (bodies[i]["com"]) {
            ChVector3d com_pos = VNULL;
            ChQuaterniond com_rot = QUNIT;
            if (bodies[i]["com"]["location"])
                com_pos = ReadVector(bodies[i]["com"]["location"]);
            if (bodies[i]["com"]["orientation"])
                com_rot = ReadRotation(bodies[i]["com"]["orientation"], m_use_degrees);
            body.com = ChFramed(com_pos, com_rot);
        }
        if (!body.is_fixed) {
            body.inertia_moments = ReadVector(bodies[i]["inertia"]["moments"]);
            if (bodies[i]["inertia"]["products"])
                body.inertia_products = ReadVector(bodies[i]["inertia"]["products"]);
        }
        body.geometry = ReadBodyGeometry(bodies[i], m_file_handler, m_use_degrees);

        if (m_verbose)
            body.PrintInfo(name);

        m_body_params.insert({name, body});
    }
}

void ChParserMbsYAML::LoadJoints(const YAML::Node& joints) {
    ChAssertAlways(joints.IsSequence());
    if (m_verbose)
        cout << "\njoints: " << joints.size() << endl;

    for (size_t i = 0; i < joints.size(); i++) {
        ChAssertAlways(joints[i]["name"]);
        ChAssertAlways(joints[i]["type"]);
        ChAssertAlways(joints[i]["body1"]);
        ChAssertAlways(joints[i]["body2"]);
        ChAssertAlways(joints[i]["location"]);

        auto name = joints[i]["name"].as<std::string>();

        JointParams joint;
        joint.type = ReadJointType(joints[i]["type"]);
        joint.body1 = joints[i]["body1"].as<std::string>();
        joint.body2 = joints[i]["body2"].as<std::string>();

        std::shared_ptr<ChJoint::BushingData> bushing_data = nullptr;
        if (joints[i]["bushing_data"]) {
            joint.bdata = ReadBushingData(joints[i]["bushing_data"]);
            joint.is_kinematic = false;
        }

        joint.frame = ReadJointFrame(joints[i]);

        if (m_verbose)
            joint.PrintInfo(name);

        m_joint_params.insert({name, joint});
    }
}

void ChParserMbsYAML::LoadConstraints(const YAML::Node& constraints) {
    ChAssertAlways(constraints.IsSequence());
    if (m_verbose)
        cout << "\nconstraints: " << constraints.size() << endl;

    for (size_t i = 0; i < constraints.size(); i++) {
        ChAssertAlways(constraints[i]["name"]);
        ChAssertAlways(constraints[i]["type"]);
        ChAssertAlways(constraints[i]["body1"]);
        ChAssertAlways(constraints[i]["body2"]);
        auto name = constraints[i]["name"].as<std::string>();
        auto type = ChToUpper(constraints[i]["type"].as<std::string>());

        if (type == "DISTANCE") {
            DistanceConstraintParams dist;
            ChAssertAlways(constraints[i]["point1"]);
            ChAssertAlways(constraints[i]["point2"]);
            dist.body1 = constraints[i]["body1"].as<std::string>();
            dist.body2 = constraints[i]["body2"].as<std::string>();
            dist.point1 = ReadVector(constraints[i]["point1"]);
            dist.point2 = ReadVector(constraints[i]["point2"]);

            if (m_verbose)
                dist.PrintInfo(name);

            m_distcnstr_params.insert({name, dist});

        } else if (type == "REVOLUTE-SPHERICAL") {
            //// TODO
        } else if (type == "REVOLUTE-TRANSLATIONAL") {
            //// TODO
        }
    }
}

void ChParserMbsYAML::LoadTSDAs(const YAML::Node& tsdas) {
    ChAssertAlways(tsdas.IsSequence());
    if (m_verbose)
        cout << "\nTSDA (translational spring dampers): " << tsdas.size() << endl;

    for (size_t i = 0; i < tsdas.size(); i++) {
        ChAssertAlways(tsdas[i]["name"]);
        ChAssertAlways(tsdas[i]["body1"]);
        ChAssertAlways(tsdas[i]["body2"]);
        ChAssertAlways(tsdas[i]["point1"]);
        ChAssertAlways(tsdas[i]["point2"]);

        auto name = tsdas[i]["name"].as<std::string>();

        TsdaParams tsda;
        tsda.body1 = tsdas[i]["body1"].as<std::string>();
        tsda.body2 = tsdas[i]["body2"].as<std::string>();
        tsda.point1 = ReadVector(tsdas[i]["point1"]);
        tsda.point2 = ReadVector(tsdas[i]["point2"]);
        tsda.force = ReadTSDAFunctor(tsdas[i], tsda.free_length);
        tsda.geometry = ReadTSDAGeometry(tsdas[i]);

        if (m_verbose)
            tsda.PrintInfo(name);

        m_tsda_params.insert({name, tsda});
    }
}

void ChParserMbsYAML::LoadRSDAs(const YAML::Node& rsdas) {
    ChAssertAlways(rsdas.IsSequence());
    if (m_verbose)
        cout << "\nRSDA (rotational spring dampers): " << rsdas.size() << endl;

    for (size_t i = 0; i < rsdas.size(); i++) {
        ChAssertAlways(rsdas[i]["name"]);
        ChAssertAlways(rsdas[i]["body1"]);
        ChAssertAlways(rsdas[i]["body2"]);
        ChAssertAlways(rsdas[i]["axis"]);

        auto name = rsdas[i]["name"].as<std::string>();

        RsdaParams rsda;
        rsda.body1 = rsdas[i]["body1"].as<std::string>();
        rsda.body2 = rsdas[i]["body2"].as<std::string>();
        rsda.axis = ReadVector(rsdas[i]["axis"]);
        rsda.axis.Normalize();
        if (rsdas[i]["location"])
            rsda.pos = ReadVector(rsdas[i]["location"]);
        rsda.torque = ReadRSDAFunctor(rsdas[i], rsda.free_angle);
        if (m_use_degrees)
            rsda.free_angle *= CH_DEG_TO_RAD;

        if (m_verbose)
            rsda.PrintInfo(name);

        m_rsda_params.insert({name, rsda});
    }
}

void ChParserMbsYAML::LoadBodyLoads(const YAML::Node& loads) {
    ChAssertAlways(loads.IsSequence());
    if (m_verbose)
        cout << "\nbody loads: " << loads.size() << endl;

    for (size_t i = 0; i < loads.size(); i++) {
        ChAssertAlways(loads[i]["name"]);
        ChAssertAlways(loads[i]["type"]);
        ChAssertAlways(loads[i]["body"]);

        auto name = loads[i]["name"].as<std::string>();

        BodyLoadParams load;
        load.type = ReadBodyLoadType(loads[i]["type"]);
        load.body = loads[i]["body"].as<std::string>();
        if (loads[i]["local_load"])
            load.local_load = loads[i]["local_load"].as<bool>();
        if (loads[i]["load"])
            load.value = ReadVector(loads[i]["load"]);
        if (load.type == BodyLoadType::FORCE) {
            load.local_point = loads[i]["local_point"].as<bool>();
            load.point = ReadVector(loads[i]["point"]);
        }
        if (loads[i]["modulation_function"])
            load.modulation = ReadFunction(loads[i]["modulation_function"], m_use_degrees);

        if (m_verbose)
            load.PrintInfo(name);

        m_bodyload_params.insert({name, load});
    }
}

void ChParserMbsYAML::LoadControllers(const YAML::Node& controllers) {
    ChAssertAlways(controllers.IsSequence());
    if (m_verbose)
        cout << "\nexternal load controllers: " << controllers.size() << endl;

    for (size_t i = 0; i < controllers.size(); i++) {
        ChAssertAlways(controllers[i]["name"]);
        ChAssertAlways(controllers[i]["type"]);
        ChAssertAlways(controllers[i]["body"]);

        auto name = controllers[i]["name"].as<std::string>();

        BodyLoadParams load;
        load.type = ReadBodyLoadType(controllers[i]["type"]);
        load.body = controllers[i]["body"].as<std::string>();
        if (controllers[i]["local_load"])
            load.local_load = controllers[i]["local_load"].as<bool>();
        if (load.type == BodyLoadType::FORCE) {
            load.local_point = controllers[i]["local_point"].as<bool>();
            load.point = ReadVector(controllers[i]["point"]);
        }
        load.value = 0;

        if (m_verbose)
            load.PrintInfo(name);

        m_load_controller_params.insert({name, load});
    }
}

void ChParserMbsYAML::LoadMotors(const YAML::Node& motors) {
    ChAssertAlways(motors.IsSequence());
    if (m_verbose) {
        cout << "\nmotors: " << motors.size() << endl;
    }

    for (size_t i = 0; i < motors.size(); i++) {
        ChAssertAlways(motors[i]["name"]);
        ChAssertAlways(motors[i]["type"]);
        ChAssertAlways(motors[i]["body1"]);
        ChAssertAlways(motors[i]["body2"]);
        ChAssertAlways(motors[i]["location"]);
        ChAssertAlways(motors[i]["axis"]);
        ChAssertAlways(motors[i]["actuation_type"]);
        ChAssertAlways(motors[i]["actuation_function"]);

        auto name = motors[i]["name"].as<std::string>();

        MotorParams motor;
        motor.type = ReadMotorType(motors[i]["type"]);
        motor.body1 = motors[i]["body1"].as<std::string>();
        motor.body2 = motors[i]["body2"].as<std::string>();
        motor.pos = ReadVector(motors[i]["location"]);
        motor.axis = ReadVector(motors[i]["axis"]);
        motor.actuation_type = ReadMotorActuationType(motors[i]["actuation_type"]);
        motor.actuation_function = ReadFunction(motors[i]["actuation_function"], m_use_degrees);

        switch (motor.type) {
            case MotorType::LINEAR:
                if (motors[i]["guide"])
                    motor.guide = ReadMotorGuideType(motors[i]["guide"]);
                break;
            case MotorType::ROTATION:
                if (motors[i]["spindle"])
                    motor.spindle = ReadMotorSpindleType(motors[i]["spindle"]);
                break;
        }

        // A ChFunctionSetpoint actuation function indicates an external controller
        if (std::dynamic_pointer_cast<ChFunctionSetpoint>(motor.actuation_function))
            motor.has_controller = true;

        if (m_verbose)
            motor.PrintInfo(name);

        m_motor_params.insert({name, motor});
    }
}

#ifdef CHRONO_FEA

using MaterialsMap = std::unordered_map<std::string, size_t>;
using SectionsMap = std::unordered_map<std::string, size_t>;

// Find the given FEA material name in the provided map and return the associated index
static int FindMaterialFEA(const std::string& name, const MaterialsMap materials) {
    auto m = materials.find(name);
    if (m == materials.end()) {
        cerr << "Cannot find the FEA material with name '" << name << "' in the provided map" << endl;
        throw std::runtime_error("Invalid FEA material name in map");
    }
    return (int)m->second;
}

// Find the given FEA beam section in the provided map and return the associated index
static int FindBeamSection(const std::string& name, const SectionsMap sections) {
    auto m = sections.find(name);
    if (m == sections.end()) {
        cerr << "Cannot find the beam section with name '" << name << "' in the provided map" << endl;
        throw std::runtime_error("Invalid beam section name in map");
    }
    return (int)m->second;
}

void ChParserMbsYAML::LoadFEABeams(const YAML::Node& fea) {
    // Maps from a material or beam section name to the index in the corresponding array
    MaterialsMap materials_map;
    SectionsMap sections_map;

    // Read beam sections
    if (fea["beam_sections"]) {
        auto sections = fea["beam_sections"];
        ChAssertAlways(sections.IsSequence());
        size_t num_sections = sections.size();
        if (m_verbose)
            cout << "\nFEA beam sections: " << num_sections << endl;

        for (size_t i = 0; i < num_sections; i++) {
            ChAssertAlways(sections[i]["name"]);
            ChAssertAlways(sections[i]["type"]);
            auto section_name = sections[i]["name"].as<std::string>();
            auto section_type = ReadFEABeamSectionType(sections[i]["type"]);
            std::shared_ptr<fea::ChBeamSection> section;
            switch (section_type) {
                case FEABeamSectionType::EULER_SIMPLE: {
                  auto section_beam = chrono_types::make_shared<fea::ChBeamSectionEulerSimple>();
                  //// TODO
                  section = section_beam;
                  throw std::runtime_error("NOT YET IMPLEMENTED");
                    break;
                }
                case FEABeamSectionType::ANCF_CABLE: {
                    auto diameter = sections[i]["diameter"].as<double>();
                    auto modulus = sections[i]["Young_modulus"].as<double>();
                    auto density = sections[i]["density"].as<double>();
                    auto damping = sections[i]["Rayleight_damping"].as<double>();
                    auto section_cable = chrono_types::make_shared<fea::ChBeamSectionCableANCF>();
                    section_cable->SetDiameter(diameter);
                    section_cable->SetYoungModulus(modulus);
                    section_cable->SetDensity(density);
                    section_cable->SetRayleighDamping(damping);
                    section = section_cable;
                    break;
                }
                case FEABeamSectionType::COSSERAT: {
                    //// TODO
                    throw std::runtime_error("NOT YET IMPLEMENTED");
                    break;
                }
                case FEABeamSectionType::TIMOSHENKO: {
                    //// TODO
                    throw std::runtime_error("NOT YET IMPLEMENTED");
                    break;
                }
            }
            m_beam_sections.push_back(section);
            sections_map.insert({section_name, i});
        }
    }

    // Read FEA materials - process only beam materials
    if (fea["materials"]) {
        auto materials = fea["materials"];
        ChAssertAlways(materials.IsSequence());
        size_t num_materials = materials.size();
        if (m_verbose)
            cout << "\nFEA materials: " << num_materials << endl;

        for (int i = 0; i < num_materials; i++) {
            ChAssertAlways(materials[i]["name"]);
            ChAssertAlways(materials[i]["type"]);
            auto material_name = materials[i]["name"].as<std::string>();
            auto material_type = ReadFEAMaterialType(materials[i]["type"]);
            std::shared_ptr<fea::ChMaterialFEA> material;
            switch (material_type) {
                case FEAMaterialType::BEAM_ANCF: {
                    //// TODO
                    throw std::runtime_error("NOT YET IMPLEMENTED");
                    break;
                }
            }
            m_fea_materials.push_back(material);
            materials_map.insert({material_name, i});
        }
    }

    // Read FEA beams
    auto beams = fea["beams"];

    ChAssertAlways(beams.IsSequence());
    if (m_verbose)
        cout << "\nFEA beams: " << beams.size() << endl;

    for (size_t i = 0; i < beams.size(); i++) {
        ChAssertAlways(beams[i]["name"]);
        ChAssertAlways(beams[i]["type"]);
        ChAssertAlways(beams[i]["num_elements"]);
        ChAssertAlways(beams[i]["start_point"]);
        ChAssertAlways(beams[i]["end_point"]);

        auto name = beams[i]["name"].as<std::string>();

        FEABeamParams beam;
        beam.type = ReadFEABeamType(beams[i]["type"]);
        beam.num_elements = beams[i]["num_elements"].as<int>();
        beam.start = ReadVector(beams[i]["start_point"]);
        beam.end = ReadVector(beams[i]["end_point"]);

        //// TODO - more comprehensive error checking
        switch (beam.type) {
            case FEABeamType::EULER: {
                ChAssertAlways(beams[i]["section"]);
                auto section_name = beams[i]["section"].as<std::string>();
                auto section_index = FindBeamSection(section_name, sections_map);
                beam.section = m_beam_sections[section_index];
                beam.material = nullptr;
                ChAssertAlways(beams[i]["up"]);
                beam.up = ReadVector(beams[i]["up"]);
                break;
            }
            case FEABeamType::ANCF_CABLE: {
                ChAssertAlways(beams[i]["section"]);
                auto section_name = beams[i]["section"].as<std::string>();
                auto section_index = FindBeamSection(section_name, sections_map);
                beam.section = m_beam_sections[section_index];
                beam.material = nullptr;
                break;
            }
            case FEABeamType::ANCF_3243: {
                ChAssertAlways(beams[i]["material"]);
                auto material_name = beams[i]["material"].as<std::string>();
                auto material_index = FindMaterialFEA(material_name, materials_map);
                beam.material = m_fea_materials[material_index];
                beam.section = nullptr;
                break;
            }
            case FEABeamType::ANCF_3333: {
                ChAssertAlways(beams[i]["material"]);
                auto material_name = beams[i]["material"].as<std::string>();
                auto material_index = FindMaterialFEA(material_name, materials_map);
                beam.material = m_fea_materials[material_index];
                beam.section = nullptr;
                break;
            }
            case FEABeamType::IGA: {
                ChAssertAlways(beams[i]["section"]);
                auto section_name = beams[i]["section"].as<std::string>();
                auto section_index = FindBeamSection(section_name, sections_map);
                beam.section = m_beam_sections[section_index];
                beam.material = nullptr;
                break;
            }
            case FEABeamType::TIMOSHENKO: {
                ChAssertAlways(beams[i]["section"]);
                auto section_name = beams[i]["section"].as<std::string>();
                auto section_index = FindBeamSection(section_name, sections_map);
                beam.section = m_beam_sections[section_index];
                beam.material = nullptr;
                break;
            }
        }

        if (beams[i]["visualization"])
            beam.visualization = ChVisualShapeFEA::Settings::Read(beams[i]["visualization"]);

        if (m_verbose)
            beam.PrintInfo(name);

        m_beam_params.insert({name, beam});
    }
}

void ChParserMbsYAML::LoadFEAShells(const YAML::Node& fea) {
    MaterialsMap materials_map;

    // Read FEA materials - process only shell materials
    if (fea["materials"]) {
        auto materials = fea["materials"];
        ChAssertAlways(materials.IsSequence());
        size_t num_materials = materials.size();
        if (m_verbose)
            cout << "\nFEA materials: " << num_materials << endl;

        for (int i = 0; i < num_materials; i++) {
            ChAssertAlways(materials[i]["name"]);
            ChAssertAlways(materials[i]["type"]);
            auto material_name = materials[i]["name"].as<std::string>();
            auto material_type = ReadFEAMaterialType(materials[i]["type"]);
            std::shared_ptr<fea::ChMaterialFEA> material;
            switch (material_type) {
                case FEAMaterialType::SHELL_ANCF: {
                    //// TODO
                    throw std::runtime_error("NOT YET IMPLEMENTED");
                    break;
                }
                case FEAMaterialType::SHELL_KIRCHHOFF: {
                    //// TODO
                    throw std::runtime_error("NOT YET IMPLEMENTED");
                    break;
                }
                case FEAMaterialType::SHELL_REISSNER: {
                    //// TODO
                    throw std::runtime_error("NOT YET IMPLEMENTED");
                    break;
                }
            }
            m_fea_materials.push_back(material);
            materials_map.insert({material_name, i});
        }
    }

    // Read FEA shells
    auto shells = fea["shells"];

    ChAssertAlways(shells.IsSequence());
    if (m_verbose)
        cout << "\nFEA shells: " << shells.size() << endl;

    //// TODO
    throw std::runtime_error("NOT YET IMPLEMENTED");
}

void ChParserMbsYAML::LoadFEAConstraints(const YAML::Node& fea) {
    auto constraints = fea["constraints"];
    ChAssertAlways(constraints.IsSequence());
    size_t num_constraints = constraints.size();
    if (m_verbose)
        cout << "\nFEA constraints: " << num_constraints << endl;

    for (size_t i = 0; i < num_constraints; i++) {
        ChAssertAlways(constraints[i]["name"]);
        ChAssertAlways(constraints[i]["type"]);

        auto name = constraints[i]["name"].as<std::string>();

        FEAConstraintParams constraint;
        constraint.type = ReadFEAConstraintType(constraints[i]["type"]);

        switch (constraint.type) {
            case FEAConstraintType::NODE_FRAME: {
                ChAssertAlways(constraints[i]["node"]);
                ChAssertAlways(constraints[i]["body"]);
                auto node_mesh = constraints[i]["node"]["mesh"].as<std::string>();
                auto node_id = constraints[i]["node"]["index"].as<int>();
                constraint.node1 = {node_mesh, node_id};
                constraint.body = constraints[i]["body"].as<std::string>();
                break;
            }
            case FEAConstraintType::NODESLOPE_FRAME: {
                ChAssertAlways(constraints[i]["node"]);
                ChAssertAlways(constraints[i]["body"]);
                ChAssertAlways(constraints[i]["body_direction"]);
                auto node_mesh = constraints[i]["node"]["mesh"].as<std::string>();
                auto node_id = constraints[i]["node"]["index"].as<int>();
                constraint.node1 = {node_mesh, node_id};
                constraint.body = constraints[i]["body"].as<std::string>();
                constraint.direction = ReadVector(constraints[i]["body_direction"]);
                break;
            }
            case FEAConstraintType::NODE_NODE: {
                ChAssertAlways(constraints[i]["node1"]);
                ChAssertAlways(constraints[i]["node2"]);
                auto node1_mesh = constraints[i]["node1"]["mesh"].as<std::string>();
                auto node1_id = constraints[i]["node1"]["index"].as<int>();
                constraint.node1 = {node1_mesh, node1_id};
                auto node2_mesh = constraints[i]["node2"]["mesh"].as<std::string>();
                auto node2_id = constraints[i]["node2"]["index"].as<int>();
                constraint.node2 = {node2_mesh, node2_id};
                break;
            }
            case FEAConstraintType::NODE_FACE: {
                //// TODO
                throw std::runtime_error("NOT YET IMPLEMENTED");
                break;
            }
        }

        if (m_verbose)
            constraint.PrintInfo(name);

        m_fea_constraint_params.insert({name, constraint});
    }
}

#endif

// -----------------------------------------------------------------------------

void ChParserMbsYAML::SetSolver(ChSystem& sys, const SolverParams& params, int num_threads_pardiso) {
    if (params.type == ChSolver::Type::PARDISO_MKL) {
#ifdef CHRONO_PARDISO_MKL
        auto solver = chrono_types::make_shared<ChSolverPardisoMKL>(num_threads_pardiso);
        solver->LockSparsityPattern(params.lock_sparsity_pattern);
        sys.SetSolver(solver);
#else
        cerr << "Chrono::PardisoMKL module not enabled. PARDISO_MKL solver not available." << endl;
        throw std::runtime_error("Chrono::PardisoMKL module not enabled");
#endif
    } else if (params.type == ChSolver::Type::MUMPS) {
#ifdef CHRONO_MUMPS
        auto solver = chrono_types::make_shared<ChSolverMumps>();
        solver->LockSparsityPattern(params.lock_sparsity_pattern);
        solver->EnableNullPivotDetection(true);
        solver->GetMumpsEngine().SetICNTL(14, 50);
        sys.SetSolver(solver);
#else
        cerr << "Chrono::MUMPS module not enabled. MUMPS solver not available." << endl;
        throw std::runtime_error("Chrono::MUMPS module not enabled");
#endif
    } else {
        sys.SetSolverType(params.type);
        switch (params.type) {
            case ChSolver::Type::SPARSE_LU:
            case ChSolver::Type::SPARSE_QR: {
                auto solver = std::static_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
                solver->LockSparsityPattern(params.lock_sparsity_pattern);
                solver->UseSparsityPatternLearner(params.use_sparsity_pattern_learner);
                break;
            }
            case ChSolver::Type::BARZILAIBORWEIN:
            case ChSolver::Type::APGD:
            case ChSolver::Type::PSOR: {
                auto solver = std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver());
                solver->SetMaxIterations(params.max_iterations);
                solver->SetOmega(params.overrelaxation_factor);
                solver->SetSharpnessLambda(params.sharpness_factor);
                solver->EnableDiagonalPreconditioner(params.enable_diagonal_preconditioner);
                solver->EnableWarmStart(params.warm_start);
                break;
            }
            case ChSolver::Type::BICGSTAB:
            case ChSolver::Type::MINRES:
            case ChSolver::Type::GMRES: {
                auto solver = std::static_pointer_cast<ChIterativeSolverLS>(sys.GetSolver());
                solver->SetMaxIterations(params.max_iterations);
                solver->SetTolerance(params.tolerance);
                solver->EnableDiagonalPreconditioner(params.enable_diagonal_preconditioner);
                solver->EnableWarmStart(params.warm_start);
                break;
            }
            default:
                break;
        }
    }
}

void ChParserMbsYAML::SetIntegrator(ChSystem& sys, const IntegratorParams& params) {
    sys.SetTimestepperType(params.type);

    switch (params.type) {
        case ChTimestepper::Type::HHT: {
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxIters(params.max_iterations);
            integrator->SetRelTolerance(params.rtol);
            integrator->SetAbsTolerances(params.atol_states, params.atol_multipliers);
            integrator->SetStepControl(params.use_stepsize_control);
            integrator->SetJacobianUpdateMethod(params.use_modified_newton ? ChTimestepperImplicit::JacobianUpdate::EVERY_STEP
                                                                           : ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT: {
            auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
            integrator->SetMaxIters(params.max_iterations);
            integrator->SetRelTolerance(params.rtol);
            integrator->SetAbsTolerances(params.atol_states, params.atol_multipliers);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
        default:
            break;
    }
}

void ChParserMbsYAML::SetSimulationParameters(ChSystem& sys) {
    if (!m_solver_loaded) {
        cerr << "[ChParserMbsYAML::SetSimulationParameters] Warning: no YAML simulation file loaded." << endl;
        cerr << "No changes applied to system." << endl;
        return;
    }

    if (sys.GetContactMethod() != m_sim.contact_method) {
        cerr << "[ChParserMbsYAML::SetSimulationParameters] Warning: contact method mismatch." << endl;
    }

    sys.SetGravitationalAcceleration(m_sim.gravity);
    sys.SetNumThreads(m_sim.num_threads_chrono, m_sim.num_threads_collision, m_sim.num_threads_eigen);

    SetSolver(sys, m_sim.solver, m_sim.num_threads_pardiso);
    SetIntegrator(sys, m_sim.integrator);
}

std::shared_ptr<ChSystem> ChParserMbsYAML::CreateSystem() {
    if (!m_solver_loaded) {
        cerr << "[ChParserMbsYAML::CreateSystem] Warning: no YAML simulation file loaded." << endl;
        cerr << "Returning a default ChSystemNSC." << endl;
        return chrono_types::make_shared<ChSystemNSC>();
    }

    // Create a Chrono system of specified type
    m_sys = ChSystem::Create(m_sim.contact_method);
    m_sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(0.2);

    // Set solver and integrator parameters
    SetSimulationParameters(*m_sys);

    return m_sys;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChBodyAuxRef> ChParserMbsYAML::FindBodyByName(const std::string& name) const {
    return FindBodyByName(name, m_crt_instance);
}

std::shared_ptr<ChBodyAuxRef> ChParserMbsYAML::FindBodyByName(const std::string& name, int model_instance) const {
    if (model_instance >= GetNumInstances()) {
        cerr << "[ChParserMbsYAML::FindBodyByName] Error: incorrect model instance number" << endl;
        throw std::runtime_error("Incorrect model instance number");
    }

    auto b = m_body_params.find(name);
    if (b != m_body_params.end())
        return b->second.body[model_instance];

    cerr << "[ChParserMbsYAML::FindBodyByName] Error: Cannot find body with name: " << name << endl;
    throw std::runtime_error("Invalid body name");
}

std::vector<std::shared_ptr<ChBodyAuxRef>> ChParserMbsYAML::FindBodiesByName(const std::string& name) const {
    auto b = m_body_params.find(name);
    if (b != m_body_params.end())
        return b->second.body;

    std::vector<std::shared_ptr<ChBodyAuxRef>> empty;
    return empty;
}

std::shared_ptr<ChLinkMotor> ChParserMbsYAML::FindMotorByName(const std::string& name) const {
    return FindMotorByName(name, m_crt_instance);
}

std::shared_ptr<ChLinkMotor> ChParserMbsYAML::FindMotorByName(const std::string& name, int model_instance) const {
    if (model_instance >= GetNumInstances()) {
        cerr << "[ChParserMbsYAML::FindMotorByName] Error: incorrect model instance number" << endl;
        throw std::runtime_error("Incorrect model instance number");
    }

    auto m = m_motor_params.find(name);
    if (m != m_motor_params.end())
        return m->second.motor[model_instance];

    cerr << "[ChParserMbsYAML::FindMotorByName] Error: Cannot find motor with name: " << name << endl;
    throw std::runtime_error("Invalid motor name");
}

std::vector<std::shared_ptr<ChLinkMotor>> ChParserMbsYAML::FindMotorsByName(const std::string& name) const {
    auto m = m_motor_params.find(name);
    if (m != m_motor_params.end())
        return m->second.motor;

    std::vector<std::shared_ptr<ChLinkMotor>> empty;
    return empty;
}

#ifdef CHRONO_FEA

std::shared_ptr<fea::ChMesh> ChParserMbsYAML::FindMeshByName(const std::string& name) const {
    return FindMeshByName(name, m_crt_instance);
}

std::shared_ptr<fea::ChMesh> ChParserMbsYAML::FindMeshByName(const std::string& name, int model_instance) const {
    if (model_instance >= GetNumInstances()) {
        cerr << "[ChParserMbsYAML::FindMeshByName] Error: incorrect model instance number" << endl;
        throw std::runtime_error("Incorrect model instance number");
    }

    auto b = m_beam_params.find(name);
    if (b != m_beam_params.end())
        return b->second.mesh[model_instance];
    //// TODO
    ////auto s = m_shell_params.find(name);
    ////if (s != m_shell_params.end())
    ////    return s->second.mesh[model_instance];

    cerr << "[ChParserMbsYAML::FindMeshByName] Error: Cannot find FEA mesh with name: " << name << endl;
    throw std::runtime_error("Invalid body name");
}

std::vector<std::shared_ptr<fea::ChMesh>> ChParserMbsYAML::FindMeshesByName(const std::string& name) const {
    auto b = m_beam_params.find(name);
    if (b != m_beam_params.end())
        return b->second.mesh;
    //// TODO
    ////auto s = m_shell_params.find(name);
    ////if (s != m_shell_params.end())
    ////    return s->second.mesh;

    std::vector<std::shared_ptr<fea::ChMesh>> empty;
    return empty;
}

std::shared_ptr<fea::ChNodeFEAxyz> ChParserMbsYAML::FindNodeXYZ(const std::string& mesh_name, int node_index) const {
    return FindNodeXYZ(mesh_name, node_index, m_crt_instance);
}

std::shared_ptr<fea::ChNodeFEAxyz> ChParserMbsYAML::FindNodeXYZ(const std::string& mesh_name, int node_index, int model_instance) const {
    if (model_instance >= GetNumInstances()) {
        cerr << "[ChParserMbsYAML::FindNodeXYZ] Error: incorrect model instance number" << endl;
        throw std::runtime_error("Incorrect model instance number");
    }
    auto b = m_beam_params.find(mesh_name);
    if (b != m_beam_params.end())
        return b->second.nodesXYZ[node_index];
    cerr << "[ChParserMbsYAML::FindNodeXYZ] Error: Cannot find FEA mesh with name: " << mesh_name << endl;
    throw std::runtime_error("Invalid body name");
}

std::shared_ptr<fea::ChNodeFEAxyzrot> ChParserMbsYAML::FindNodeXYZrot(const std::string& mesh_name, int node_index) const {
    return FindNodeXYZrot(mesh_name, node_index, m_crt_instance);
}

std::shared_ptr<fea::ChNodeFEAxyzrot> ChParserMbsYAML::FindNodeXYZrot(const std::string& mesh_name, int node_index, int model_instance) const {
    if (model_instance >= GetNumInstances()) {
        cerr << "[ChParserMbsYAML::FindNodeXYZrot] Error: incorrect model instance number" << endl;
        throw std::runtime_error("Incorrect model instance number");
    }
    auto b = m_beam_params.find(mesh_name);
    if (b != m_beam_params.end())
        return b->second.nodesXYZrot[node_index];
    cerr << "[ChParserMbsYAML::FindNodeXYZrot] Error: Cannot find FEA mesh with name: " << mesh_name << endl;
    throw std::runtime_error("Invalid body name");
}

#endif

// -----------------------------------------------------------------------------

int ChParserMbsYAML::Populate(ChSystem& sys, const ChFramed& model_frame, const std::string& model_prefix) {
    m_crt_instance++;

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserMbsYAML] Populate ChSystem\n" << endl;
    }

    if (!m_model_loaded) {
        cerr << "[ChParserMbsYAML::Populate] Error: no YAML model loaded." << endl;
        throw std::runtime_error("No YAML model loaded");
    }

    // Create a load container for bushings and body forces
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    // Create bodies
    if (m_verbose)
        cout << "create bodies...                         " << m_body_params.size() << endl;
    for (auto& item : m_body_params) {
        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        body->SetName(model_prefix + item.first);
        body->SetFixed(item.second.is_fixed);
        body->SetMass(item.second.mass);
        body->SetFrameCOMToRef(item.second.com);
        body->SetInertiaXX(item.second.inertia_moments);
        body->SetInertiaXY(item.second.inertia_products);
        body->SetFrameRefToAbs(model_frame * ChFramed(item.second.pos, item.second.rot));
        body->SetLinVel(item.second.lin_vel);
        body->SetAngVelLocal(item.second.ang_vel);
        sys.AddBody(body);
        item.second.body.push_back(body);
        m_output_components.bodies.push_back(body);
    }

    // Create joints (kinematic or bushings)
    if (m_verbose)
        cout << "create joints...                         " << m_joint_params.size() << endl;
    for (auto& item : m_joint_params) {
        auto body1 = FindBodyByName(item.second.body1);
        auto body2 = FindBodyByName(item.second.body2);
        auto joint = chrono_types::make_shared<ChJoint>(item.second.type,                 //
                                                        model_prefix + item.first,        //
                                                        body1,                            //
                                                        body2,                            //
                                                        model_frame * item.second.frame,  //
                                                        item.second.bdata);
        if (joint->IsKinematic()) {
            sys.AddLink(joint->GetAsLink());
            m_output_components.joints.push_back(joint->GetAsLink());
        } else {
            load_container->Add(joint->GetAsBushing());
            m_output_components.bushings.push_back(joint->GetAsBushing());
        }
        item.second.joint.push_back(joint);
    }

    // Create distance constraints
    if (m_verbose)
        cout << "create distance constraints...           " << m_distcnstr_params.size() << endl;
    for (auto& item : m_distcnstr_params) {
        auto body1 = FindBodyByName(item.second.body1);
        auto body2 = FindBodyByName(item.second.body2);

        auto dist = chrono_types::make_shared<ChLinkDistance>();
        dist->SetName(model_prefix + item.first);
        dist->Initialize(body1, body2, false, model_frame * item.second.point1, model_frame * item.second.point2);
        sys.AddLink(dist);
        item.second.dist.push_back(dist);
        m_output_components.constraints.push_back(dist);
    }

    // Create TSDAs
    if (m_verbose)
        cout << "create TSDAs...                          " << m_tsda_params.size() << endl;
    for (auto& item : m_tsda_params) {
        auto body1 = FindBodyByName(item.second.body1);
        auto body2 = FindBodyByName(item.second.body2);
        auto tsda = chrono_types::make_shared<ChLinkTSDA>();
        tsda->SetName(model_prefix + item.first);
        tsda->Initialize(body1, body2, false, model_frame * item.second.point1, model_frame * item.second.point2);
        tsda->SetRestLength(item.second.free_length);
        tsda->RegisterForceFunctor(item.second.force);
        sys.AddLink(tsda);
        item.second.tsda.push_back(tsda);
        m_output_components.tsdas.push_back(tsda);
    }

    // Create RSDAs
    if (m_verbose)
        cout << "create RSDAs...                          " << m_rsda_params.size() << endl;
    for (auto& item : m_rsda_params) {
        auto body1 = FindBodyByName(item.second.body1);
        auto body2 = FindBodyByName(item.second.body2);

        ChMatrix33d rot;
        rot.SetFromAxisX(item.second.axis);
        ChQuaterniond quat = rot.GetQuaternion() * QuatFromAngleY(CH_PI_2);

        auto rsda = chrono_types::make_shared<ChLinkRSDA>();
        rsda->SetName(model_prefix + item.first);
        rsda->Initialize(body1, body2, model_frame * ChFramed(item.second.pos, quat));
        rsda->SetRestAngle(item.second.free_angle);
        rsda->RegisterTorqueFunctor(item.second.torque);
        sys.AddLink(rsda);
        item.second.rsda.push_back(rsda);
        m_output_components.rsdas.push_back(rsda);
    }

    // Create body loads
    if (m_verbose)
        cout << "create body loads...                     " << m_bodyload_params.size() << endl;
    for (auto& item : m_bodyload_params) {
        auto body = FindBodyByName(item.second.body);
        std::shared_ptr<ChLoadCustom> load;
        switch (item.second.type) {
            case BodyLoadType::FORCE: {
                auto loadF = chrono_types::make_shared<ChLoadBodyForce>(body, item.second.value, item.second.local_load, item.second.point, item.second.local_point);
                if (item.second.modulation)
                    loadF->SetModulationFunction(item.second.modulation);
                load = loadF;
                break;
            }
            case BodyLoadType::TORQUE: {
                auto loadT = chrono_types::make_shared<ChLoadBodyTorque>(body, item.second.value, item.second.local_load);
                if (item.second.modulation)
                    loadT->SetModulationFunction(item.second.modulation);
                load = loadT;
                break;
            }
        }
        load->SetName(model_prefix + item.first);
        load_container->Add(load);
        item.second.load.push_back(load);
        m_output_components.loads.push_back(load);
    }

    // Create external body load controllers
    if (m_verbose)
        cout << "create external body load controllers... " << m_load_controller_params.size() << endl;
    for (auto& item : m_load_controller_params) {
        auto body = FindBodyByName(item.second.body);
        std::shared_ptr<ChLoadCustom> load;
        switch (item.second.type) {
            case BodyLoadType::FORCE:
                load = chrono_types::make_shared<ChLoadBodyForce>(body, 0.0, item.second.local_load, item.second.point, item.second.local_point);
                break;
            case BodyLoadType::TORQUE:
                load = chrono_types::make_shared<ChLoadBodyTorque>(body, 0.0, item.second.local_load);
                break;
        }
        load->SetName(model_prefix + item.first);
        load_container->Add(load);
        item.second.load.push_back(load);
        m_output_components.loads.push_back(load);
    }

    // Create motors
    if (m_verbose && !m_motor_params.empty())
        cout << "create motors...                         " << m_motor_params.size() << endl;
    for (auto& item : m_motor_params) {
        auto body1 = FindBodyByName(item.second.body1);
        auto body2 = FindBodyByName(item.second.body2);

        ChMatrix33d rot;
        rot.SetFromAxisX(item.second.axis);
        ChQuaterniond quat = rot.GetQuaternion() * QuatFromAngleY(CH_PI_2);

        switch (item.second.type) {
            case MotorType::LINEAR: {
                std::shared_ptr<ChLinkMotorLinear> motor;
                switch (item.second.actuation_type) {
                    case MotorActuation::POSITION:
                        motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
                        break;
                    case MotorActuation::SPEED:
                        motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
                        break;
                    case MotorActuation::FORCE:
                        motor = chrono_types::make_shared<ChLinkMotorLinearForce>();
                        break;
                }
                motor->SetName(model_prefix + item.first);
                motor->SetGuideConstraint(item.second.guide);
                motor->SetMotorFunction(item.second.actuation_function);
                motor->Initialize(body1, body2, model_frame * ChFramed(item.second.pos, quat));
                sys.AddLink(motor);
                item.second.motor.push_back(motor);
                m_output_components.lin_motors.push_back(motor);

                break;
            }

            case MotorType::ROTATION: {
                std::shared_ptr<ChLinkMotorRotation> motor;
                switch (item.second.actuation_type) {
                    case MotorActuation::POSITION:
                        motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
                        break;
                    case MotorActuation::SPEED:
                        motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
                        break;
                    case MotorActuation::FORCE:
                        motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
                        break;
                }
                motor->SetName(model_prefix + item.first);
                motor->SetSpindleConstraint(item.second.spindle);
                motor->SetMotorFunction(item.second.actuation_function);
                motor->Initialize(body1, body2, model_frame * ChFramed(item.second.pos, quat));
                sys.AddLink(motor);
                item.second.motor.push_back(motor);
                m_output_components.rot_motors.push_back(motor);

                break;
            }
        }
    }

#ifdef CHRONO_FEA

    // Create FEA beams
    if (m_verbose && !m_beam_params.empty())
        cout << "create FEA beams...                      " << m_beam_params.size() << endl;

    for (auto& item : m_beam_params) {
        auto mesh = chrono_types::make_shared<fea::ChMesh>();

        switch (item.second.type) {
            case FEABeamType::EULER: {
                fea::ChBuilderBeamEuler builder;
                builder.BuildBeam(mesh,                                                                    // container FEA mesh
                                  std::static_pointer_cast<fea::ChBeamSectionEuler>(item.second.section),  // cable section
                                  item.second.num_elements,                                                // number of cable elements
                                  item.second.start,                                                       // point A (start of beam)
                                  item.second.end,                                                         // point B (end of beam)
                                  item.second.up                                                           // beam up direction
                );
                for (const auto& n : builder.GetLastBeamNodes())
                    item.second.nodesXYZrot.push_back(n);
                for (const auto& e : builder.GetLastBeamElements())
                    item.second.elements.push_back(e);
                break;
            }
            case FEABeamType::ANCF_CABLE: {
                fea::ChBuilderCableANCF builder;
                builder.BuildBeam(mesh,                                                                        // container FEA mesh
                                  std::static_pointer_cast<fea::ChBeamSectionCableANCF>(item.second.section),  // cable section
                                  item.second.num_elements,                                                    // number of cable elements
                                  item.second.start,                                                           // point A (start of beam)
                                  item.second.end                                                              // point B (end of beam)
                );
                for (const auto& n : builder.GetLastBeamNodes())
                    item.second.nodesXYZ.push_back(n);
                for (const auto& e : builder.GetLastBeamElements())
                    item.second.elements.push_back(e);
                break;
            }
            case FEABeamType::ANCF_3243: {
                //// TODO
                throw std::runtime_error("NOT YET IMPLEMENTED");
                break;
            }
            case FEABeamType::ANCF_3333: {
                //// TODO
                throw std::runtime_error("NOT YET IMPLEMENTED");
                break;
            }
            case FEABeamType::IGA: {
                //// TODO
                throw std::runtime_error("NOT YET IMPLEMENTED");
                break;
            }
            case FEABeamType::TIMOSHENKO: {
                //// TODO
                throw std::runtime_error("NOT YET IMPLEMENTED");
                break;
            }
        }

        sys.AddMesh(mesh);
        item.second.mesh.push_back(mesh);
    }

    // Create FEA shells
    //// TODO
    ////if (m_verbose && !m_shell_params.empty())
    ////    cout << "create FEA shells...                     " << m_shell_params.size() << endl;
    ////
    ////for (auto& item : m_beam_params) {
    ////    auto mesh = chrono_types::make_shared<fea::ChMesh>();
    ////
    ////    switch (item.second.type) {
    ////        //// TODO
    ////    }
    ////
    ////    sys.AddMesh(mesh);
    ////    item.second.mesh.push_back(mesh);
    ////}

    // Create FEA constraints
    if (m_verbose && !m_fea_constraint_params.empty())
        cout << "create FEA constraints...                " << m_beam_params.size() << endl;

    for (auto& item : m_fea_constraint_params) {
        switch (item.second.type) {
            case FEAConstraintType::NODE_FRAME: {
                auto node = FindNodeXYZ(item.second.node1.first, item.second.node1.second - 1);
                auto body = FindBodyByName(item.second.body);
                auto constraint = chrono_types::make_shared<fea::ChLinkNodeFrame>();
                constraint->Initialize(std::static_pointer_cast<fea::ChNodeFEAxyz>(node), body);
                sys.Add(constraint);
                break;
            }
            case FEAConstraintType::NODESLOPE_FRAME: {
                auto node = FindNodeXYZ(item.second.node1.first, item.second.node1.second - 1);
                auto body = FindBodyByName(item.second.body);
                const auto& dir = item.second.direction;
                auto constraint = chrono_types::make_shared<fea::ChLinkNodeSlopeFrame>();
                constraint->Initialize(std::static_pointer_cast<fea::ChNodeFEAxyzD>(node), body);
                constraint->SetDirectionInBodyCoords(dir);
                sys.Add(constraint);
                break;
            }
            case FEAConstraintType::NODE_NODE: {
                //// TODO
                throw std::runtime_error("NOT YET IMPLEMENTED");
                break;
            }
            case FEAConstraintType::NODE_FACE: {
                //// TODO
                throw std::runtime_error("NOT YET IMPLEMENTED");
                break;
            }
        }
    }

#endif

    if (m_verbose)
        cout << endl;

    // Create body collision models
    for (auto& item : m_body_params) {
        if (item.second.geometry->HasCollision())
            item.second.geometry->CreateCollisionShapes(item.second.body[m_crt_instance], 0, sys.GetContactMethod());
    }

    // Create visualization assets
    for (auto& item : m_body_params)
        item.second.geometry->CreateVisualizationAssets(item.second.body[m_crt_instance], m_vis_type);
    for (auto& item : m_tsda_params)
        item.second.geometry->CreateVisualizationAssets(item.second.tsda[m_crt_instance]);
    for (auto& item : m_distcnstr_params)
        item.second.dist[m_crt_instance]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
#ifdef CHRONO_FEA
    CreateFEAVisualizationAssets();
#endif

    return m_crt_instance;
}

void ChParserMbsYAML::Depopulate(ChSystem& sys, int instance_index) {
    for (auto& item : m_body_params) {
        ChAssertAlways(item.second.body.size() > instance_index);
        sys.Remove(item.second.body[instance_index]);
        item.second.body.erase(item.second.body.begin() + instance_index);
    }

    for (auto& item : m_joint_params) {
        ChAssertAlways(item.second.joint.size() > instance_index);
        ChJoint::Remove(item.second.joint[instance_index]);
        item.second.joint.erase(item.second.joint.begin() + instance_index);
    }

    for (auto& item : m_distcnstr_params) {
        ChAssertAlways(item.second.dist.size() > instance_index);
        sys.Remove(item.second.dist[instance_index]);
        item.second.dist.erase(item.second.dist.begin() + instance_index);
    }

    for (auto& item : m_tsda_params) {
        ChAssertAlways(item.second.tsda.size() > instance_index);
        sys.Remove(item.second.tsda[instance_index]);
        item.second.tsda.erase(item.second.tsda.begin() + instance_index);
    }

    for (auto& item : m_rsda_params) {
        ChAssertAlways(item.second.rsda.size() > instance_index);
        sys.Remove(item.second.rsda[instance_index]);
        item.second.rsda.erase(item.second.rsda.begin() + instance_index);
    }
}

// -----------------------------------------------------------------------------

void ChParserMbsYAML::AttachLoadController(std::shared_ptr<ChLoadController> controller, const std::string& name, int model_instance) {
    if (!m_model_loaded) {
        cerr << "[ChParserMbsYAML::AttachLoadController] Error: No MBS model loaded" << endl;
        throw std::runtime_error("No MBS model loaded");
    }

    // Check that parameters for a controller with this base name were specified in the input YAML file
    auto c = m_load_controller_params.find(name);
    if (c == m_load_controller_params.end()) {
        cerr << "[ChParserMbsYAML::AttachLoadController] Error: cannot find controller with name: " << name << endl;
        throw std::runtime_error("Invalid controller name");
    }

    // Check the model instance number
    if (model_instance >= GetNumInstances()) {
        cerr << "[ChParserMbsYAML::AttachLoadController] Error: incorrect model instance number" << endl;
        throw std::runtime_error("Incorrect model instance number");
    }

    // Initialize the provided controller
    controller->Initialize(*this, model_instance);

    LoadController load_controller;
    load_controller.controller = controller;
    load_controller.model_instance = model_instance;

    m_load_controllers.insert({name, load_controller});
}

void ChParserMbsYAML::AttachMotorController(std::shared_ptr<ChMotorController> controller, const std::string& name, int model_instance) {
    if (!m_model_loaded) {
        cerr << "[ChParserMbsYAML::AttachMotorController] Error: No MBS model loaded" << endl;
        throw std::runtime_error("No MBS model loaded");
    }

    // Check that a motor with this base name was specified in the input YAML file
    auto c = m_motor_params.find(name);
    if (c == m_motor_params.end()) {
        cerr << "[ChParserMbsYAML::AttachMotorController] Error: cannot find motor with name: " << name << endl;
        throw std::runtime_error("Invalid motor name");
    }

    // Check that the motor was flag as externally actuated
    if (!c->second.has_controller) {
        cerr << "[ChParserMbsYAML::AttachMotorController] Error: the motor " << name << " is not externally controlled" << endl;
        throw std::runtime_error("Invalid motor");
    }

    // Check the model instance number
    if (model_instance >= GetNumInstances()) {
        cerr << "[ChParserMbsYAML::AttachMotorController] Error: incorrect model instance number" << endl;
        throw std::runtime_error("Incorrect model instance number");
    }

    // Initialize the provided controller
    controller->Initialize(*this, model_instance);

    MotorController motor_controller;
    motor_controller.controller = controller;
    motor_controller.model_instance = model_instance;
    motor_controller.motor = FindMotorByName(name);
    m_motor_controllers.insert({name, motor_controller});
}

void ChParserMbsYAML::ApplyLoadControllerLoads(const LoadControllerLoads& controller_loads) {
    for (const auto& controller_load : controller_loads) {
        const auto& name = controller_load.first;
        const auto& load = controller_load.second;

        // Find the controllers with this base name
        auto c = m_load_controller_params.find(name);
        if (c == m_load_controller_params.end()) {
            cerr << "[ChParserMbsYAML::ApplyLoadControllerLoads] Error: cannot find controller with name: " << name << endl;
            throw std::runtime_error("Invalid controller name");
        }
        auto type = c->second.type;
        bool local_load = c->second.local_load;
        auto& body_loads = c->second.load;

        // Set the load to controllers from all model instances
        for (auto& body_load : body_loads) {
            switch (type) {
                case BodyLoadType::FORCE: {
                    auto body_load_F = std::dynamic_pointer_cast<ChLoadBodyForce>(body_load);
                    body_load_F->SetForce(load, local_load);
                    break;
                }
                case BodyLoadType::TORQUE: {
                    auto body_load_T = std::dynamic_pointer_cast<ChLoadBodyTorque>(body_load);
                    body_load_T->SetTorque(load, local_load);
                    break;
                }
            }
        }
    }
}

void ChParserMbsYAML::ApplyMotorControllerActuations(const MotorControllerActuations& controller_actuations) {
    double time = m_sys->GetChTime();

    for (const auto& controller_actuation : controller_actuations) {
        const auto& name = controller_actuation.first;
        double actuation = controller_actuation.second;

        // Find the motor with this base name
        auto c = m_motor_params.find(name);
        if (c == m_motor_params.end()) {
            cerr << "[ChParserMbsYAML::ApplyMotorControllerLoads] Error: cannot find motor with name: " << name << endl;
            throw std::runtime_error("Invalid motor name");
        }

        // Ensure this is a controlled motor
        if (!c->second.has_controller) {
            cerr << "[ChParserMbsYAML::ApplyMotorControllerLoads] Error: the motor " << name << " is not externally controlled" << endl;
            throw std::runtime_error("Invalid motor");
        }

        // Set the actuation function value for the named motor in all model instances
        for (auto& motor : c->second.motor) {
            auto function = std::static_pointer_cast<ChFunctionSetpoint>(motor->GetMotorFunction());
            function->SetSetpoint(time, actuation);
        }
    }
}

void ChParserMbsYAML::DoStepDynamics() {
    double time = m_sys->GetChTime();
    double time_step = m_sim.integrator.time_step;

    // Process load controllers
    for (auto& load_controller : m_load_controllers) {
        // Model instance
        int model_instance = load_controller.second.model_instance;

        // Find parameters for this controller
        const auto& name = load_controller.first;
        auto c = m_load_controller_params.find(name);
        ChAssertAlways(c != m_load_controller_params.end());
        auto type = c->second.type;
        bool local_load = c->second.local_load;
        auto& body_load = c->second.load[model_instance];

        // Synchronize at current time and advance controller dynamics
        load_controller.second.controller->Synchronize(time);
        load_controller.second.controller->Advance(time_step);

        // Apply controller loads
        auto load = load_controller.second.controller->GetLoad();

        switch (type) {
            case BodyLoadType::FORCE: {
                auto body_load_F = std::dynamic_pointer_cast<ChLoadBodyForce>(body_load);
                body_load_F->SetForce(load, local_load);
                break;
            }
            case BodyLoadType::TORQUE: {
                auto body_load_T = std::dynamic_pointer_cast<ChLoadBodyTorque>(body_load);
                body_load_T->SetTorque(load, local_load);
                break;
            }
        }
    }

    // Process motor controllers
    for (auto& motor_controller : m_motor_controllers) {
        motor_controller.second.controller->Synchronize(time);
        motor_controller.second.controller->Advance(time_step);

        // Set controller actuation
        auto actuation = motor_controller.second.controller->GetActuation();
        auto function = std::static_pointer_cast<ChFunctionSetpoint>(motor_controller.second.motor->GetMotorFunction());
        function->SetSetpoint(actuation, time);
    }

    // Advance multibody system dynamics
    m_sys->DoStepDynamics(time_step);

    // Enforce soft real time (if requested)
    if (m_sim.enforce_realtime)
        m_rt_timer.Spin(time_step);
}

void ChParserMbsYAML::WriteOutput(int frame, double time) {
    ChParserYAML::WriteOutput(frame, time);
    m_output_db->Write(frame, time, m_output_components);
}

// -----------------------------------------------------------------------------

ChParserMbsYAML::SolverParams::SolverParams()
    : type(ChSolver::Type::BARZILAIBORWEIN),
      lock_sparsity_pattern(false),
      use_sparsity_pattern_learner(true),
      tolerance(0.0),
      max_iterations(100),
      enable_diagonal_preconditioner(false),
      overrelaxation_factor(1.0),
      sharpness_factor(1.0),
      warm_start(false) {}

ChParserMbsYAML::IntegratorParams::IntegratorParams()
    : type(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED),
      time_step(1e-3),
      rtol(1e-4),
      atol_states(1e-4),
      atol_multipliers(1e2),
      max_iterations(50),
      use_stepsize_control(false),
      use_modified_newton(false) {}

ChParserMbsYAML::SimParams::SimParams()
    : gravity({0, 0, -9.8}),
      contact_method(ChContactMethod::NSC),
      num_threads_chrono(1),
      num_threads_collision(1),
      num_threads_eigen(1),
      num_threads_pardiso(1),
      end_time(-1),
      enforce_realtime(false) {}

void ChParserMbsYAML::SolverParams::PrintInfo() const {
    cout << "solver" << endl;
    cout << "  type:                         " << ChSolver::GetTypeAsString(type) << endl;
    switch (type) {
        case ChSolver::Type::SPARSE_LU:
        case ChSolver::Type::SPARSE_QR:
            cout << "  lock sparsity pattern?        " << std::boolalpha << lock_sparsity_pattern << endl;
            cout << "  use sparsity pattern learner? " << std::boolalpha << use_sparsity_pattern_learner << endl;
            break;
        case ChSolver::Type::BARZILAIBORWEIN:
        case ChSolver::Type::APGD:
        case ChSolver::Type::PSOR:
            cout << "  max iterations:               " << max_iterations << endl;
            cout << "  over-relaxation factor:       " << overrelaxation_factor << endl;
            cout << "  sharpness factor:             " << sharpness_factor << endl;
            cout << "  warm start?                   " << (warm_start ? "true" : "false");
            break;
        case ChSolver::Type::BICGSTAB:
        case ChSolver::Type::MINRES:
        case ChSolver::Type::GMRES:
            cout << "  max iterations:               " << max_iterations << endl;
            cout << "  tolerance:                    " << tolerance << endl;
            cout << "  use diagonal preconditioner?  " << std::boolalpha << enable_diagonal_preconditioner << endl;
            cout << "  warm start?                   " << (warm_start ? "true" : "false");
            break;
        case ChSolver::Type::PARDISO_MKL:
        case ChSolver::Type::MUMPS:
            cout << "  lock sparsity pattern?        " << std::boolalpha << lock_sparsity_pattern << endl;
            break;
    }
}

void ChParserMbsYAML::IntegratorParams::PrintInfo() const {
    cout << "integrator" << endl;
    cout << "  time step:                    " << time_step << endl;
    cout << "  type:                         " << ChTimestepper::GetTypeAsString(type) << endl;
    switch (type) {
        case ChTimestepper::Type::HHT:
            cout << "  max iterations:               " << max_iterations << endl;
            cout << "  rel tolerance:                " << rtol << endl;
            cout << "  abs tolerance (states):       " << atol_states << endl;
            cout << "  abs tolerance (multipliers):  " << atol_multipliers << endl;
            cout << "  use step-size control?        " << std::boolalpha << use_stepsize_control << endl;
            cout << "  use modified Newton?          " << std::boolalpha << use_modified_newton << endl;
            break;

        case ChTimestepper::Type::EULER_IMPLICIT:
            cout << "  max iterations:               " << max_iterations << endl;
            cout << "  rel tolerance:                " << rtol << endl;
            cout << "  abs tolerance (states):       " << atol_states << endl;
            cout << "  abs tolerance (multipliers):  " << atol_multipliers << endl;
            break;
    }
}

void ChParserMbsYAML::SimParams::PrintInfo() const {
    cout << "contact method:         " << (contact_method == ChContactMethod::NSC ? "NSC" : "SMC") << endl;
    cout << endl;
    cout << "simulation end time:    " << (end_time < 0 ? "infinite" : std::to_string(end_time)) << endl;
    cout << "enforce real time?      " << std::boolalpha << enforce_realtime << endl;
    cout << endl;
    cout << "num threads Chrono:     " << num_threads_chrono << endl;
    cout << "num threads collision:  " << num_threads_collision << endl;
    cout << "num threads Eigen:      " << num_threads_eigen << endl;
    cout << "num threads Pardiso:    " << num_threads_pardiso << endl;
    cout << endl;
    solver.PrintInfo();
    cout << endl;
    integrator.PrintInfo();
}

// -----------------------------------------------------------------------------

ChParserMbsYAML::BodyParams::BodyParams()
    : pos(VNULL),
      rot(QUNIT),
      lin_vel(VNULL),
      ang_vel(VNULL),
      is_fixed(false),
      mass(1),
      com(ChFramed(VNULL, QUNIT)),
      inertia_moments(ChVector3d(1)),
      inertia_products(ChVector3d(0)) {}

ChParserMbsYAML::JointParams::JointParams() : type(ChJoint::Type::LOCK), body1(""), body2(""), frame(ChFramed(VNULL, QUNIT)), bdata(nullptr), is_kinematic(true) {}

ChParserMbsYAML::DistanceConstraintParams::DistanceConstraintParams() : body1(""), body2(""), point1(VNULL), point2(VNULL) {}

ChParserMbsYAML::TsdaParams::TsdaParams() : body1(""), body2(""), point1(VNULL), point2(VNULL), free_length(0), force(nullptr) {}

ChParserMbsYAML::RsdaParams::RsdaParams() : body1(""), body2(""), pos(VNULL), axis(ChVector3d(0, 0, 1)), free_angle(0), torque(nullptr) {}

ChParserMbsYAML::BodyLoadParams::BodyLoadParams() : type(BodyLoadType::FORCE), body(""), local_load(true), local_point(true), value(VNULL), point(VNULL) {}

ChParserMbsYAML::MotorParams::MotorParams()
    : type(MotorType::ROTATION),
      actuation_type(MotorActuation::NONE),
      actuation_function(nullptr),
      has_controller(false),
      body1(""),
      body2(""),
      pos(VNULL),
      axis(ChVector3d(0, 0, 1)),
      guide(ChLinkMotorLinear::GuideConstraint::PRISMATIC),
      spindle(ChLinkMotorRotation::SpindleConstraint::REVOLUTE) {}

std::string ChParserMbsYAML::GetMotorActuationTypeString(MotorActuation type) {
    switch (type) {
        case MotorActuation::NONE:
            return "none";
        case MotorActuation::POSITION:
            return "position";
        case MotorActuation::SPEED:
            return "speed";
        case MotorActuation::FORCE:
            return "force";
        default:
            return "unknown";
    }
}

static void PrintGeometry(const utils::ChBodyGeometry& geometry) {
    bool collision = geometry.HasCollision();
    bool vis_prims = geometry.HasVisualizationPrimitives();
    bool vis_mesh = geometry.HasVisualizationMesh();

    cout << "      collision?                " << (collision ? "yes" : "no") << endl;
    cout << "      visualization primitives? " << (vis_prims ? "yes" : "no") << endl;
    cout << "      visualization mesh?       " << (vis_mesh ? "yes" : "no") << endl;

    //// TODO
}

void ChParserMbsYAML::BodyParams::PrintInfo(const std::string& name) const {
    cout << "  name:        " << name << endl;
    cout << "    pos:       " << pos << endl;
    cout << "    rot:       " << rot << endl;
    cout << "    fixed?     " << (is_fixed ? "true" : "false") << endl;
    cout << "    mass:      " << mass << endl;
    cout << "    com frame: " << com.GetPos() << " | " << com.GetRot() << endl;
    cout << "    I_xx:      " << inertia_moments << endl;
    cout << "    I_xy:      " << inertia_products << endl;
    cout << "    geometry:  " << endl;
    PrintGeometry(*geometry);
}

void ChParserMbsYAML::JointParams::PrintInfo(const std::string& name) const {
    cout << "  name:           " << name << endl;
    cout << "     type:        " << ChJoint::GetTypeString(type);
    cout << " (" << (is_kinematic ? "kinematic joint" : "bushing") << ")" << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     joint frame: " << frame.GetPos() << " | " << frame.GetRot() << endl;
}

void ChParserMbsYAML::DistanceConstraintParams::PrintInfo(const std::string& name) const {
    cout << "  name:           " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     point1:      " << point1 << endl;
    cout << "     point2:      " << point2 << endl;
}

void ChParserMbsYAML::TsdaParams::PrintInfo(const std::string& name) const {
    cout << "  name:           " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     point1:      " << point1 << endl;
    cout << "     point2:      " << point2 << endl;
    cout << "     free_length: " << free_length << endl;
}

void ChParserMbsYAML::RsdaParams::PrintInfo(const std::string& name) const {
    cout << "  name:           " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     pos:         " << pos << endl;
    cout << "     axis:        " << axis << endl;
    cout << "     free_angle:  " << free_angle << endl;
}

void ChParserMbsYAML::BodyLoadParams::PrintInfo(const std::string& name) const {
    std::string type_str = "force";
    if (type == BodyLoadType::TORQUE)
        type_str = "torque";

    cout << "  name:          " << name << endl;
    cout << "     type:       " << type_str << endl;
    cout << "     body:       " << body << endl;
    switch (type) {
        case BodyLoadType::FORCE:
            cout << "     force:      " << value << "  " << (local_load ? " (local)" : " (absolute)") << endl;
            cout << "     point:      " << point << "  " << (local_point ? " (local)" : " (absolute)") << endl;
            break;
        case BodyLoadType::TORQUE:
            cout << "     torque:     " << value << "  " << (local_load ? " (local)" : " (absolute)") << endl;
    }
}

void ChParserMbsYAML::MotorParams::PrintInfo(const std::string& name) const {
    std::string type_str = "linear";
    if (type == MotorType::ROTATION)
        type_str = "rotation";

    cout << "  name:           " << name << endl;
    cout << "     type:        " << type_str << endl;
    cout << "     actuation:   " << GetMotorActuationTypeString(actuation_type) << endl;
    cout << "     controller:  " << has_controller << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     pos:         " << pos << endl;
    cout << "     axis:        " << axis << endl;
    switch (type) {
        case MotorType::LINEAR:
            cout << "     guide:       " << ChLinkMotorLinear::GetGuideTypeString(guide) << endl;
            break;
        case MotorType::ROTATION:
            cout << "     spindle:     " << ChLinkMotorRotation::GetSpindleTypeString(spindle) << endl;
            break;
    }
}

#ifdef CHRONO_FEA

ChParserMbsYAML::FEABeamParams::FEABeamParams() : type(FEABeamType::EULER), num_elements(0) {}

ChParserMbsYAML::FEAConstraintParams::FEAConstraintParams() : type(FEAConstraintType::NODE_NODE) {}

void ChParserMbsYAML::FEABeamParams::PrintInfo(const std::string& name) const {
    std::string type_str;
    switch (type) {
        case FEABeamType::EULER:
            type_str = "Euler";
            break;
        case FEABeamType::ANCF_CABLE:
            type_str = "ANCF_cable";
            break;
        case FEABeamType::ANCF_3243:
            type_str = "ANCF_3243";
            break;
        case FEABeamType::ANCF_3333:
            type_str = "ANCF_3333";
            break;
        case FEABeamType::IGA:
            type_str = "IGA";
            break;
        case FEABeamType::TIMOSHENKO:
            type_str = "Timoshenko";
            break;
        default:
            type_str = "UNKNOWN";
            break;
    }

    cout << "  name:            " << name << endl;
    cout << "     type:         " << type_str << endl;
    cout << "     start:        " << start << endl;
    cout << "     end:          " << end << endl;
    cout << "     num elements: " << num_elements << endl;
}

void ChParserMbsYAML::FEAConstraintParams::PrintInfo(const std::string& name) const {
    std::string type_str;
    switch (type) {
        case FEAConstraintType::NODE_FRAME:
            type_str = "node - body";
            break;
        case FEAConstraintType::NODESLOPE_FRAME:
            type_str = "node slope - body direction";
            break;
        case FEAConstraintType::NODE_NODE:
            type_str = "node - node";
            break;
        case FEAConstraintType::NODE_FACE:
            type_str = "node - face";
            break;
    }

    cout << "  name:            " << name << endl;
    cout << "     type:         " << type_str << endl;

    switch (type) {
        case FEAConstraintType::NODE_FRAME:
            type_str = "node - body";
            cout << "     node:         " << node1.second << " on " << node1.first << endl;
            cout << "     body:         " << body << endl;
            break;
        case FEAConstraintType::NODESLOPE_FRAME:
            cout << "     node:         " << node1.second << " on " << node1.first << endl;
            cout << "     body:         " << body << endl;
            cout << "     body_dir:     " << direction << endl;
            break;
        case FEAConstraintType::NODE_NODE:
            cout << "     node1:        " << node1.second << " on " << node1.first << endl;
            cout << "     node2:        " << node2.second << " on " << node2.first << endl;
            break;
        case FEAConstraintType::NODE_FACE:
            cout << "     node:         " << node1.second << " on " << node1.first << endl;
            break;
    }
}

#endif

// =============================================================================

ChJoint::Type ChParserMbsYAML::ReadJointType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "LOCK") {
        return ChJoint::Type::LOCK;
    } else if (type == "POINT_LINE") {
        return ChJoint::Type::POINTLINE;
    } else if (type == "POINT_PLANE") {
        return ChJoint::Type::POINTPLANE;
    } else if (type == "REVOLUTE") {
        return ChJoint::Type::REVOLUTE;
    } else if (type == "SPHERICAL") {
        return ChJoint::Type::SPHERICAL;
    } else if (type == "PRISMATIC") {
        return ChJoint::Type::PRISMATIC;
    } else if (type == "UNIVERSAL") {
        return ChJoint::Type::UNIVERSAL;
    } else {
        return ChJoint::Type::LOCK;
    }
}

ChFramed ChParserMbsYAML::ReadJointFrame(const YAML::Node& a) {
    ChAssertAlways(a["location"]);
    ChVector3d pos = ReadVector(a["location"]);
    ChQuaterniond rot = QUNIT;

    switch (ReadJointType(a["type"])) {
        case ChJoint::Type::LOCK:
        case ChJoint::Type::SPHERICAL:
            rot = QUNIT;
            break;
        case ChJoint::Type::REVOLUTE:
        case ChJoint::Type::PRISMATIC: {
            ChAssertAlways(a["axis"]);
            auto axis = ReadVector(a["axis"]);
            axis.Normalize();
            ChMatrix33d R;
            R.SetFromAxisZ(axis);
            rot = R.GetQuaternion();
            break;
        }
        case ChJoint::Type::UNIVERSAL: {
            ChAssertAlways(a["axis1"]);
            ChAssertAlways(a["axis2"]);
            auto axis_x = ReadVector(a["axis1"]);
            auto axis_y = ReadVector(a["axis2"]);
            auto axis_z = Vcross(axis_x, axis_y);
            axis_y = Vcross(axis_z, axis_x);
            axis_x.Normalize();
            axis_y.Normalize();
            axis_z.Normalize();
            ChMatrix33d R(axis_x, axis_y, axis_z);
            rot = R.GetQuaternion();
            break;
        }
    }

    //// TODO - POINTLINE, POINTPLANE

    return ChFramed(pos, rot);
}

std::shared_ptr<ChJoint::BushingData> ChParserMbsYAML::ReadBushingData(const YAML::Node& bd) {
    auto bushing_data = chrono_types::make_shared<ChJoint::BushingData>();

    bushing_data->K_lin = bd["stiffness_linear"].as<double>();
    bushing_data->D_lin = bd["damping_linear"].as<double>();
    bushing_data->K_rot = bd["stiffness_rotational"].as<double>();
    bushing_data->D_rot = bd["damping_rotational"].as<double>();

    if (bd["DOF"]) {
        bushing_data->K_lin_dof = bd["DOF"]["stiffness_linear"].as<double>();
        bushing_data->D_lin_dof = bd["DOF"]["damping_linear"].as<double>();
        bushing_data->K_rot_dof = bd["DOF"]["stiffness_rotational"].as<double>();
        bushing_data->D_rot_dof = bd["DOF"]["damping_rotational"].as<double>();
    }

    return bushing_data;
}

// -----------------------------------------------------------------------------

std::shared_ptr<utils::ChTSDAGeometry> ChParserMbsYAML::ReadTSDAGeometry(const YAML::Node& d) {
    auto geometry = chrono_types::make_shared<utils::ChTSDAGeometry>();

    if (d["visualization"]) {
        ChAssertAlways(d["visualization"]["type"]);
        std::string type = ChToUpper(d["visualization"]["type"].as<std::string>());
        if (type == "SEGMENT") {
            geometry->vis_segment = chrono_types::make_shared<utils::ChTSDAGeometry::SegmentShape>();
        } else if (type == "SPRING") {
            // the default below are copied from the actual class, and since there
            // are no setters I can't just take the default constructor and override
            // the properties the user specifies (my only alternative would be to
            // construct and read from a prototype instance)
            double radius = 0.05;
            int resolution = 65;
            double turns = 5;
            if (d["visualization"]["radius"])
                radius = d["visualization"]["radius"].as<double>();
            if (d["visualization"]["resolution"])
                resolution = d["visualization"]["resolution"].as<int>();
            if (d["visualization"]["turns"])
                turns = d["visualization"]["turns"].as<double>();
            geometry->vis_spring = chrono_types::make_shared<utils::ChTSDAGeometry::SpringShape>(radius, resolution, turns);
        } else {
            cerr << "Incorrect TSDA visualization shape type: " << d["visualization"]["type"].as<std::string>() << endl;
            throw std::runtime_error("Incorrect TSDA visualization shape type");
        }
    }

    return geometry;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChLinkTSDA::ForceFunctor> ChParserMbsYAML::ReadTSDAFunctor(const YAML::Node& tsda, double& free_length) {
    enum class FunctorType { LinearSpring, NonlinearSpring, LinearDamper, NonlinearDamper, DegressiveDamper, LinearSpringDamper, NonlinearSpringDamper, MapSpringDamper, Unknown };

    // Determine type of functor to be created (based on specified keys)
    FunctorType type = FunctorType::Unknown;
    free_length = 0;

    if (tsda["spring_coefficient"]) {
        if (tsda["damping_coefficient"])
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    } else if (tsda["damping_coefficient"]) {
        if (tsda["degressivity_compression"] && tsda["degressivity_expansion"])
            type = FunctorType::DegressiveDamper;
        else
            type = FunctorType::LinearDamper;
    }

    if (tsda["spring_curve_data"]) {
        if (tsda["damping_curve_data"])
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    } else if (tsda["damping_curve_data"]) {
        type = FunctorType::NonlinearDamper;
    }

    if (tsda["map_data"])
        type = FunctorType::MapSpringDamper;

    // Read preload (if specified)
    double preload = 0;
    if (tsda["preload"])
        preload = tsda["preload"].as<double>();

    // Construct functor of appropriate type
    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported TSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            ChAssertAlways(tsda["free_length"]);
            ChAssertAlways(tsda["spring_coefficient"]);
            free_length = tsda["free_length"].as<double>();
            double k = tsda["spring_coefficient"].as<double>();

            auto forceCB = chrono_types::make_shared<utils::LinearSpringForce>(k, preload);
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::NonlinearSpring: {
            ChAssertAlways(tsda["free_length"]);
            free_length = tsda["free_length"].as<double>();

            auto forceCB = chrono_types::make_shared<utils::NonlinearSpringForce>(preload);

            ChAssertAlways(tsda["spring_curve_data"].IsSequence() && tsda["spring_curve_data"][0].size() == 2);
            int num_defs = tsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["spring_curve_data"][i][0].as<double>();
                double force = tsda["spring_curve_data"][i][1].as<double>();
                forceCB->add_pointK(def, force);
            }
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::LinearDamper: {
            ChAssertAlways(tsda["damping_coefficient"]);
            double c = tsda["damping_coefficient"].as<double>();

            return chrono_types::make_shared<utils::LinearDamperForce>(c);
        }

        case FunctorType::DegressiveDamper: {
            ChAssertAlways(tsda["damping_coefficient"]);
            ChAssertAlways(tsda["degressivity_compression"]);
            ChAssertAlways(tsda["degressivity_expansion"]);
            double c = tsda["damping_coefficient"].as<double>();
            double dc = tsda["degressivity_compression"].as<double>();
            double de = tsda["degressivity_expansion"].as<double>();

            return chrono_types::make_shared<utils::DegressiveDamperForce>(c, dc, de);
        }

        case FunctorType::NonlinearDamper: {
            auto forceCB = chrono_types::make_shared<utils::NonlinearDamperForce>();

            ChAssertAlways(tsda["damping_curve_data"].IsSequence() && tsda["damping_curve_data"][0].size() == 2);
            int num_speeds = tsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["damping_curve_data"][i][0].as<double>();
                double force = tsda["damping_curve_data"][i][1].as<double>();
                forceCB->add_pointC(vel, force);
            }

            return forceCB;
        }

        case FunctorType::LinearSpringDamper: {
            ChAssertAlways(tsda["free_length"]);
            ChAssertAlways(tsda["spring_coefficient"]);
            ChAssertAlways(tsda["damping_coefficient"]);
            free_length = tsda["free_length"].as<double>();
            double k = tsda["spring_coefficient"].as<double>();
            double c = tsda["damping_coefficient"].as<double>();

            auto forceCB = chrono_types::make_shared<utils::LinearSpringDamperForce>(k, c, preload);
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::NonlinearSpringDamper: {
            ChAssertAlways(tsda["free_length"]);
            free_length = tsda["free_length"].as<double>();

            auto forceCB = chrono_types::make_shared<utils::NonlinearSpringDamperForce>(preload);

            ChAssertAlways(tsda["spring_curve_data"].IsSequence() && tsda["spring_curve_data"][0].size() == 2);
            int num_defs = tsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["spring_curve_data"][i][0].as<double>();
                double force = tsda["spring_curve_data"][i][1].as<double>();
                forceCB->add_pointK(def, force);
            }
            ChAssertAlways(tsda["damping_curve_data"].IsSequence() && tsda["damping_curve_data"][0].size() == 2);
            int num_speeds = tsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["damping_curve_data"][i][0].as<double>();
                double force = tsda["damping_curve_data"][i][1].as<double>();
                forceCB->add_pointC(vel, force);
            }
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::MapSpringDamper: {
            auto forceCB = chrono_types::make_shared<utils::MapSpringDamperForce>(preload);

            ChAssertAlways(tsda["deformation"]);
            ChAssertAlways(tsda["map_data"]);
            ChAssertAlways(tsda["deformation"].IsSequence());
            ChAssertAlways(tsda["map_data"].IsSequence() && tsda["map_data"][0].size() == tsda["deformation"].size() + 1);
            int num_defs = tsda["deformation"].size();
            int num_speeds = tsda["map_data"].size();
            std::vector<double> defs(num_defs);
            for (int j = 0; j < num_defs; j++)
                defs[j] = tsda["deformation"][j].as<double>();
            forceCB->set_deformations(defs);
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["map_data"][i][0].as<double>();
                std::vector<double> force(num_defs);
                for (int j = 0; j < num_defs; j++)
                    force[j] = tsda["map_data"][i][j + 1].as<double>();
                forceCB->add_pointC(vel, force);
            }
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }
    }
}

std::shared_ptr<ChLinkRSDA::TorqueFunctor> ChParserMbsYAML::ReadRSDAFunctor(const YAML::Node& rsda, double& free_angle) {
    enum class FunctorType { LinearSpring, NonlinearSpring, LinearDamper, NonlinearDamper, LinearSpringDamper, NonlinearSpringDamper, Unknown };

    // Determine type of functor to be created (based on specified keys)
    FunctorType type = FunctorType::Unknown;
    free_angle = 0;

    if (rsda["spring_coefficient"])
        if (rsda["damping_coefficient"])
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    else if (rsda["damping_coefficient"])
        type = FunctorType::LinearDamper;

    if (rsda["spring_curve_data"])
        if (rsda["damping_curve_data"])
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    else if (rsda["damping_curve_data"])
        type = FunctorType::NonlinearDamper;

    // Read preload (if specified)
    double preload = 0;
    if (rsda["preload"])
        preload = rsda["preload"].as<double>();

    // Construct functor of appropriate type
    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported RSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            ChAssertAlways(rsda["free_angle"]);
            ChAssertAlways(rsda["spring_coefficient"]);
            free_angle = rsda["free_angle"].as<double>();
            double k = rsda["spring_coefficient"].as<double>();

            return chrono_types::make_shared<utils::LinearSpringTorque>(k, preload);
        }

        case FunctorType::NonlinearSpring: {
            ChAssertAlways(rsda["free_angle"]);
            free_angle = rsda["free_angle"].as<double>();

            auto torqueCB = chrono_types::make_shared<utils::NonlinearSpringTorque>(preload);

            ChAssertAlways(rsda["spring_curve_data"].IsSequence() && rsda["spring_curve_data"][0].size() == 2);
            int num_defs = rsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = rsda["spring_curve_data"][i][0].as<double>();
                double force = rsda["spring_curve_data"][i][1].as<double>();
                torqueCB->add_pointK(def, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearDamper: {
            ChAssertAlways(rsda["damping_coefficient"]);
            double c = rsda["damping_coefficient"].as<double>();

            return chrono_types::make_shared<utils::LinearDamperTorque>(c);
        }

        case FunctorType::NonlinearDamper: {
            auto torqueCB = chrono_types::make_shared<utils::NonlinearDamperTorque>();

            ChAssertAlways(rsda["damping_curve_data"].IsSequence() && rsda["damping_curve_data"][0].size() == 2);
            int num_speeds = rsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = rsda["damping_curve_data"][i][0].as<double>();
                double force = rsda["damping_curve_data"][i][1].as<double>();
                torqueCB->add_pointC(vel, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearSpringDamper: {
            ChAssertAlways(rsda["free_angle"]);
            ChAssertAlways(rsda["spring_coefficient"]);
            ChAssertAlways(rsda["damping_coefficient"]);
            free_angle = rsda["free_angle"].as<double>();
            double k = rsda["spring_coefficient"].as<double>();
            double c = rsda["damping_coefficient"].as<double>();

            return chrono_types::make_shared<utils::LinearSpringDamperTorque>(k, c, preload);
        }

        case FunctorType::NonlinearSpringDamper: {
            ChAssertAlways(rsda["free_angle"]);
            free_angle = rsda["free_angle"].as<double>();

            auto torqueCB = chrono_types::make_shared<utils::NonlinearSpringDamperTorque>(preload);

            ChAssertAlways(rsda["spring_curve_data"].IsSequence() && rsda["spring_curve_data"][0].size() == 2);
            int num_defs = rsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = rsda["spring_curve_data"][i][0].as<double>();
                double force = rsda["spring_curve_data"][i][1].as<double>();
                torqueCB->add_pointK(def, force);
            }
            int num_speeds = rsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = rsda["damping_curve_data"][i][0].as<double>();
                double force = rsda["damping_curve_data"][i][1].as<double>();
                torqueCB->add_pointC(vel, force);
            }

            return torqueCB;
        }
    }
}

ChParserMbsYAML::BodyLoadType ChParserMbsYAML::ReadBodyLoadType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if ((type == "FORCE"))
        return BodyLoadType::FORCE;
    else if (type == "TORQUE")
        return BodyLoadType::TORQUE;

    throw std::runtime_error("Unknown body load type");
}

ChParserMbsYAML::MotorType ChParserMbsYAML::ReadMotorType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "ROTATION")
        return MotorType::ROTATION;
    else if (type == "LINEAR")
        return MotorType::LINEAR;

    throw std::runtime_error("Unknown motor type");
}

ChParserMbsYAML::MotorActuation ChParserMbsYAML::ReadMotorActuationType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "POSITION")
        return MotorActuation::POSITION;
    else if (type == "SPEED")
        return MotorActuation::SPEED;
    else if (type == "FORCE")
        return MotorActuation::FORCE;

    return MotorActuation::NONE;
}

ChLinkMotorLinear::GuideConstraint ChParserMbsYAML::ReadMotorGuideType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "FREE")
        return ChLinkMotorLinear::GuideConstraint::FREE;
    else if (type == "PRISMATIC")
        return ChLinkMotorLinear::GuideConstraint::PRISMATIC;
    else if (type == "SPHERICAL")
        return ChLinkMotorLinear::GuideConstraint::SPHERICAL;

    throw std::runtime_error("Unknown motor guide constraint type");
}

ChLinkMotorRotation::SpindleConstraint ChParserMbsYAML::ReadMotorSpindleType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "FREE")
        return ChLinkMotorRotation::SpindleConstraint::FREE;
    else if (type == "REVOLUTE")
        return ChLinkMotorRotation::SpindleConstraint::REVOLUTE;
    else if (type == "CYLINDRICAL")
        return ChLinkMotorRotation::SpindleConstraint::CYLINDRICAL;

    throw std::runtime_error("Unknown motor spindle constraint type");
}

#ifdef CHRONO_FEA

ChParserMbsYAML::FEAMaterialType ChParserMbsYAML::ReadFEAMaterialType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "BEAM_ANCF")
        return FEAMaterialType::BEAM_ANCF;
    else if (type == "HEXA_ANCF")
        return FEAMaterialType::HEXA_ANCF;
    else if (type == "SHELL_ANCF")
        return FEAMaterialType::SHELL_ANCF;
    else if (type == "SHELL_KIRCHHOFF")
        return FEAMaterialType::SHELL_KIRCHHOFF;
    else if (type == "SHELL_REISSNER")
        return FEAMaterialType::SHELL_REISSNER;

    throw std::runtime_error("Unknown FEA element material type");
}

ChParserMbsYAML::FEABeamSectionType ChParserMbsYAML::ReadFEABeamSectionType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "EULER_SIMPLE")
        return FEABeamSectionType::EULER_SIMPLE;
    else if (type == "ANCF_CABLE")
        return FEABeamSectionType::ANCF_CABLE;
    else if (type == "COSSERAT")
        return FEABeamSectionType::COSSERAT;
    else if (type == "TIMOSHENKO")
        return FEABeamSectionType::TIMOSHENKO;

    throw std::runtime_error("Unknown FEA beam section type");
}

ChParserMbsYAML::FEABeamType ChParserMbsYAML::ReadFEABeamType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "EULER")
        return FEABeamType::EULER;
    else if (type == "ANCF_CABLE")
        return FEABeamType::ANCF_CABLE;
    else if (type == "ANCF_3243")
        return FEABeamType::ANCF_3243;
    else if (type == "ANCF_3333")
        return FEABeamType::ANCF_3333;
    else if (type == "IGA")
        return FEABeamType::IGA;
    else if (type == "TIMOSHENKO")
        return FEABeamType::TIMOSHENKO;

    throw std::runtime_error("Unknown FEA beam type");
}

ChParserMbsYAML::FEAConstraintType ChParserMbsYAML::ReadFEAConstraintType(const YAML::Node& a) {
    std::string type = ChToUpper(a.as<std::string>());
    if (type == "NODE_FRAME")
        return FEAConstraintType::NODE_FRAME;
    else if (type == "NODESLOPE_FRAME")
        return FEAConstraintType::NODESLOPE_FRAME;
    else if (type == "NODE_NODE")
        return FEAConstraintType::NODE_NODE;
    else if (type == "NODE_FACE")
        return FEAConstraintType::NODE_FACE;

    throw std::runtime_error("Unknown FEA constraint type");
}

void ChParserMbsYAML::CreateFEAVisualizationAssets() {
    // Visualization of FEA beams
    for (auto& item : m_beam_params) {
        const auto& vis_settings = item.second.visualization;

        if (vis_settings.data_type != ChVisualShapeFEA::DataType::NONE) {
            auto vis = chrono_types::make_shared<ChVisualShapeFEA>();
            vis->SetFEMdataType(vis_settings.data_type);
            vis->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NONE);
            vis->SetColormap(vis_settings.colormap);
            vis->SetColormapRange(vis_settings.data_range);
            vis->SetSmoothFaces(vis_settings.smooth_faces);
            vis->SetWireframe(vis_settings.wireframe);
            for (auto& mesh : item.second.mesh)
                mesh->AddVisualShapeFEA(vis);
        }

        if (vis_settings.glyph_type != ChVisualShapeFEA::GlyphType::NONE) {
            auto vis = chrono_types::make_shared<ChVisualShapeFEA>();
            vis->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
            vis->SetFEMglyphType(vis_settings.glyph_type);
            vis->SetSymbolsThickness(vis_settings.glyph_size);
            vis->SetDefaultSymbolsColor(vis_settings.glyph_color);
            for (auto& mesh : item.second.mesh)
                mesh->AddVisualShapeFEA(vis);
        }
    }

    // Visualization of FEA shells
    //// TODO
}

#endif

}  // namespace parsers
}  // namespace chrono
