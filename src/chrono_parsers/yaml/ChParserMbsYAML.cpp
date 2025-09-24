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
//// - perform checks during loading of YAML file or use a 3rd party YAML schema validator?
//// - add definition functions for link-type components using 2 local frames (alternative ChLink initialization)
//// - add support for other constraints (composite joints: rev-sph and rev-prismatic)
//// - add support for point-point actuators (hydraulic, FMU, external)
//// - what is the best way to deal with collision families?

#include <algorithm>

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono/physics//ChSystemNSC.h"
#include "chrono/physics//ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChForceFunctors.h"

#include "chrono/output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/output/ChOutputHDF5.h"
#endif

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_parsers/yaml/ChParserMbsYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserMbsYAML::ChParserMbsYAML()
    : m_name("YAML model"),
      m_data_path(DataPathType::ABS),
      m_rel_path("."),
      m_verbose(false),
      m_use_degrees(true),
      m_sim_loaded(false),
      m_model_loaded(false),
      m_instance_index(-1),
      m_output_dir("") {}

ChParserMbsYAML::ChParserMbsYAML(const std::string& yaml_model_filename,
                                 const std::string& yaml_sim_filename,
                                 bool verbose)
    : m_name("YAML model"),
      m_data_path(DataPathType::ABS),
      m_rel_path("."),
      m_verbose(verbose),
      m_use_degrees(true),
      m_sim_loaded(false),
      m_model_loaded(false),
      m_instance_index(-1),
      m_output_dir("") {
    LoadModelFile(yaml_model_filename);
    LoadSimulationFile(yaml_sim_filename);
}

ChParserMbsYAML::~ChParserMbsYAML() {}

// -----------------------------------------------------------------------------

static std::string ToUpper(std::string in) {
    std::transform(in.begin(), in.end(), in.begin(), ::toupper);
    return in;
}

static void CheckVersion(const YAML::Node& a) {
    std::string chrono_version = a.as<std::string>();

    auto first = chrono_version.find(".");
    ChAssertAlways(first != std::string::npos);
    std::string chrono_major = chrono_version.substr(0, first);

    ChAssertAlways(first < chrono_version.size() - 1);
    chrono_version = &chrono_version[first + 1];

    auto second = chrono_version.find(".");
    if (second == std::string::npos)
        second = chrono_version.size();
    std::string chrono_minor = chrono_version.substr(0, second);

    ChAssertAlways(chrono_major == CHRONO_VERSION_MAJOR);
    ChAssertAlways(chrono_minor == CHRONO_VERSION_MINOR);
}

void ChParserMbsYAML::LoadSimulationFile(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check a simulation object exists
    ChAssertAlways(yaml["simulation"]);
    auto sim = yaml["simulation"];

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserMbsYAML] Loading Chrono simulation specification from: " << yaml_filename << "\n" << endl;
    }

    // Mandatory
    ChAssertAlways(sim["time_step"]);
    ChAssertAlways(sim["contact_method"]);
    m_sim.time_step = sim["time_step"].as<double>();
    auto contact_method = ToUpper(sim["contact_method"].as<std::string>());
    if (contact_method == "SMC") {
        m_sim.contact_method = ChContactMethod::SMC;
    } else if (contact_method == "NSC") {
        m_sim.contact_method = ChContactMethod::NSC;
    } else {
        cerr << "Incorrect contact method: " << sim["contact_method"].as<std::string>() << endl;
        throw std::runtime_error("Incorrect contact method");
    }

    // Optional
    if (sim["end_time"])
        m_sim.end_time = sim["end_time"].as<double>();
    if (sim["enforce_realtime"])
        m_sim.enforce_realtime = sim["enforce_realtime"].as<bool>();
    if (sim["gravity"])
        m_sim.gravity = ReadVector(sim["gravity"]);

    // Integrator parameters (optional)
    if (sim["integrator"]) {
        auto intgr = sim["integrator"];
        ChAssertAlways(intgr["type"]);
        m_sim.integrator.type = ReadIntegratorType(intgr["type"]);
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
    if (sim["solver"]) {
        auto slvr = sim["solver"];
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
                break;
        }
    }

    // Run-time visualization (optional)
    if (sim["visualization"]) {
        m_sim.visualization.render = true;
        auto vis = sim["visualization"];
        if (vis["type"])
            m_sim.visualization.type = ReadVisualizationType(vis["type"]);
        if (vis["render_fps"])
            m_sim.visualization.render_fps = vis["render_fps"].as<double>();
        if (vis["enable_shadows"])
            m_sim.visualization.enable_shadows = vis["enable_shadows"].as<bool>();
        if (vis["camera"]) {
            if (vis["camera"]["vertical"]) {
                auto camera_vertical = ToUpper(vis["camera"]["vertical"].as<std::string>());
                if (camera_vertical == "Y")
                    m_sim.visualization.camera_vertical = CameraVerticalDir::Y;
                else if (camera_vertical == "Z")
                    m_sim.visualization.camera_vertical = CameraVerticalDir::Z;
                else {
                    cerr << "Incorrect camera vertical " << vis["camera"]["vertical"].as<std::string>() << endl;
                    throw std::runtime_error("Incorrect camera vertical");
                }
            }
            if (vis["camera"]["location"])
                m_sim.visualization.camera_location = ReadVector(vis["camera"]["location"]);
            if (vis["camera"]["target"])
                m_sim.visualization.camera_target = ReadVector(vis["camera"]["target"]);
        }
    } else {
        m_sim.visualization.render = false;
    }

    // Output (optional)
    if (sim["output"]) {
        ChAssertAlways(sim["output"]["type"]);
        m_sim.output.type = ReadOutputType(sim["output"]["type"]);
        if (sim["output"]["mode"])
            m_sim.output.mode = ReadOutputMode(sim["output"]["mode"]);
        if (sim["output"]["fps"])
            m_sim.output.fps = sim["output"]["fps"].as<double>();
        if (sim["output"]["output_directory"])
            m_sim.output.dir = sim["output"]["output_directory"].as<std::string>();
    }

    if (m_verbose)
        m_sim.PrintInfo();

    m_sim_loaded = true;
}

void ChParserMbsYAML::LoadModelFile(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    m_script_directory = path.parent_path().str();

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check a model object exists
    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    if (model["name"])
        m_name = model["name"].as<std::string>();

    if (model["angle_degrees"])
        m_use_degrees = model["angle_degrees"].as<bool>();

    if (model["data_path"]) {
        ChAssertAlways(model["data_path"]["type"]);
        m_data_path = ReadDataPathType(model["data_path"]["type"]);
        if (model["data_path"]["root"])
            m_rel_path = model["data_path"]["root"].as<std::string>();
    }

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserMbsYAML] Loading Chrono model specification from: '" << yaml_filename << "'\n" << endl;
        cout << "model name: '" << m_name << "'" << endl;
        cout << "angles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
        switch (m_data_path) {
            case DataPathType::ABS:
                cout << "using absolute file paths" << endl;
                break;
            case DataPathType::REL:
                cout << "using file paths relative to: '" << m_rel_path << "'" << endl;
                break;
        }
    }

    // Read bodies
    ChAssertAlways(model["bodies"]);
    auto bodies = model["bodies"];
    ChAssertAlways(bodies.IsSequence());
    if (m_verbose) {
        cout << "\nbodies: " << bodies.size() << endl;
    }

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
        if (!body.is_fixed)
            body.inertia_moments = ReadVector(bodies[i]["inertia"]["moments"]);
        if (bodies[i]["inertia"]["products"])
            body.inertia_products = ReadVector(bodies[i]["inertia"]["products"]);
        body.geometry = ReadGeometry(bodies[i]);

        if (m_verbose)
            body.PrintInfo(name);

        m_bodies.insert({name, body});
    }

    // Read joints
    if (model["joints"]) {
        auto joints = model["joints"];
        ChAssertAlways(joints.IsSequence());
        if (m_verbose) {
            cout << "\njoints: " << joints.size() << endl;
        }
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

            m_joints.insert({name, joint});
        }
    }

    // Read constraints
    if (model["constraints"]) {
        auto constraints = model["constraints"];
        ChAssertAlways(constraints.IsSequence());
        if (m_verbose) {
            cout << "\nconstraints: " << constraints.size() << endl;
        }
        for (size_t i = 0; i < constraints.size(); i++) {
            ChAssertAlways(constraints[i]["name"]);
            ChAssertAlways(constraints[i]["type"]);
            ChAssertAlways(constraints[i]["body1"]);
            ChAssertAlways(constraints[i]["body2"]);
            auto name = constraints[i]["name"].as<std::string>();
            auto type = ToUpper(constraints[i]["type"].as<std::string>());

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

                m_dists.insert({name, dist});

            } else if (type == "REVOLUTE-SPHERICAL") {
                //// TODO
            } else if (type == "REVOLUTE-TRANSLATIONAL") {
                //// TODO
            }
        }
    }

    // Read TSDA force elements elements
    if (model["tsdas"]) {
        auto tsdas = model["tsdas"];
        ChAssertAlways(tsdas.IsSequence());
        if (m_verbose) {
            cout << "\nTSDA (translational spring dampers): " << tsdas.size() << endl;
        }

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

            m_tsdas.insert({name, tsda});
        }
    }

    // Read RSDA force elements elements
    if (model["rsdas"]) {
        auto rsdas = model["rsdas"];
        ChAssertAlways(rsdas.IsSequence());
        if (m_verbose) {
            cout << "\nRSDA (rotational spring dampers): " << rsdas.size() << endl;
        }

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

            m_rsdas.insert({name, rsda});
        }
    }

    // Read applied body loads
    if (model["body_loads"]) {
        auto loads = model["body_loads"];
        ChAssertAlways(loads.IsSequence());
        if (m_verbose) {
            cout << "\nbody loads: " << loads.size() << endl;
        }
        for (size_t i = 0; i < loads.size(); i++) {
            ChAssertAlways(loads[i]["name"]);
            ChAssertAlways(loads[i]["type"]);
            ChAssertAlways(loads[i]["body"]);

            auto name = loads[i]["name"].as<std::string>();

            BodyLoadParams load;
            load.type = ReadBodyLoadType(loads[i]["type"]);
            load.body = loads[i]["body"].as<std::string>();
            load.local_load = loads[i]["local_load"].as<bool>();
            load.value = ReadVector(loads[i]["load"]);
            if (load.type == BodyLoadType::FORCE) {
                load.local_point = loads[i]["local_point"].as<bool>();
                load.point = ReadVector(loads[i]["point"]);
            }

            if (m_verbose)
                load.PrintInfo(name);

            m_body_loads.insert({name, load});
        }
    }

    // Read motors
    if (model["motors"]) {
        auto motors = model["motors"];
        ChAssertAlways(motors.IsSequence());
        if (m_verbose) {
            cout << "\nmotors: " << motors.size() << endl;
        }
        for (size_t i = 0; i < motors.size(); i++) {
            ChAssertAlways(motors[i]["name"]);
            ChAssertAlways(motors[i]["type"]);
            auto name = motors[i]["name"].as<std::string>();
            auto type = ToUpper(motors[i]["type"].as<std::string>());

            ChAssertAlways(motors[i]["body1"]);
            ChAssertAlways(motors[i]["body2"]);
            ChAssertAlways(motors[i]["actuation_type"]);
            ChAssertAlways(motors[i]["actuation_function"]);
            auto body1 = motors[i]["body1"].as<std::string>();
            auto body2 = motors[i]["body2"].as<std::string>();
            auto actuation_type = ReadMotorActuationType(motors[i]["actuation_type"]);
            auto actuation_function = ReadFunction(motors[i]["actuation_function"]);

            ChAssertAlways(motors[i]["location"]);
            ChAssertAlways(motors[i]["axis"]);
            if (type == "LINEAR") {
                MotorLinearParams motor;
                motor.body1 = body1;
                motor.body2 = body2;
                motor.actuation_type = actuation_type;
                if (motors[i]["guide"])
                    motor.guide = ReadMotorGuideType(motors[i]["guide"]);
                motor.pos = ReadVector(motors[i]["location"]);
                motor.axis = ReadVector(motors[i]["axis"]);
                motor.axis.Normalize();
                motor.actuation_function = actuation_function;

                if (m_verbose)
                    motor.PrintInfo(name);

                m_linmotors.insert({name, motor});

            } else if (type == "ROTATION") {
                MotorRotationParams motor;
                motor.body1 = body1;
                motor.body2 = body2;
                motor.actuation_type = actuation_type;
                if (motors[i]["spindle"])
                    motor.spindle = ReadMotorSpindleType(motors[i]["spindle"]);
                motor.pos = ReadVector(motors[i]["location"]);
                motor.axis = ReadVector(motors[i]["axis"]);
                motor.axis.Normalize();
                motor.actuation_function = actuation_function;

                if (m_verbose)
                    motor.PrintInfo(name);

                m_rotmotors.insert({name, motor});
            }
        }
    }

    m_model_loaded = true;
}

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
                break;
            }
            case ChSolver::Type::BICGSTAB:
            case ChSolver::Type::MINRES:
            case ChSolver::Type::GMRES: {
                auto solver = std::static_pointer_cast<ChIterativeSolverLS>(sys.GetSolver());
                solver->SetMaxIterations(params.max_iterations);
                solver->SetTolerance(params.tolerance);
                solver->EnableDiagonalPreconditioner(params.enable_diagonal_preconditioner);
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
            integrator->SetModifiedNewton(params.use_modified_newton);
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
    if (!m_sim_loaded) {
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
    if (!m_sim_loaded) {
        cerr << "[ChParserMbsYAML::CreateSystem] Warning: no YAML simulation file loaded." << endl;
        cerr << "Returning a default ChSystemNSC." << endl;
        return chrono_types::make_shared<ChSystemNSC>();
    }

    // Create a Chrono system of specified type
    auto sys = ChSystem::Create(m_sim.contact_method);
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(0.2);

    // Set solver and intergrator parameters
    SetSimulationParameters(*sys);

    return sys;
}

// -----------------------------------------------------------------------------

bool ChParserMbsYAML::HasBodyParams(const std::string& name) const {
    auto b = m_bodies.find(name);
    if (b == m_bodies.end())
        return false;
    return true;
}

const ChParserMbsYAML::BodyParams& ChParserMbsYAML::FindBodyParams(const std::string& name) const {
    auto b = m_bodies.find(name);
    if (b == m_bodies.end()) {
        cerr << "Cannot find body with name: " << name << endl;
        throw std::runtime_error("Invalid body name");
    }
    return b->second;
}

std::shared_ptr<ChBodyAuxRef> ChParserMbsYAML::FindBody(const std::string& name) const {
    auto b = m_bodies.find(name);
    if (b == m_bodies.end()) {
        cerr << "Cannot find body with name: " << name << endl;
        throw std::runtime_error("Invalid body name");
    }
    return b->second.body[m_instance_index];
}

int ChParserMbsYAML::Populate(ChSystem& sys, const ChFramed& model_frame, const std::string& model_prefix) {
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserMbsYAML] Populate ChSystem\n" << endl;
    }

    if (!m_model_loaded) {
        cerr << "[ChParserMbsYAML::Populate] Error: no YAML model loaded." << endl;
        throw std::runtime_error("No YAML model loaded");
    }

    m_instance_index++;

    // Create a load container for bushings and body forces
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    // Create bodies
    if (m_verbose && !m_bodies.empty())
        cout << "Create bodies" << endl;
    for (auto& item : m_bodies) {
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
        m_output_data.bodies.push_back(body);
    }

    // Create joints (kinematic or bushings)
    if (m_verbose && !m_joints.empty())
        cout << "Create joints" << endl;
    for (auto& item : m_joints) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);
        auto joint = chrono_types::make_shared<ChJoint>(item.second.type,                 //
                                                        model_prefix + item.first,        //
                                                        body1,                            //
                                                        body2,                            //
                                                        model_frame * item.second.frame,  //
                                                        item.second.bdata);
        if (joint->IsKinematic()) {
            sys.AddLink(joint->GetAsLink());
            m_output_data.joints.push_back(joint->GetAsLink());
        } else {
            load_container->Add(joint->GetAsBushing());
            m_output_data.bushings.push_back(joint->GetAsBushing());
        }
        item.second.joint.push_back(joint);
    }

    // Create distance constraints
    if (m_verbose && !m_dists.empty())
        cout << "Create distance constraints" << endl;
    for (auto& item : m_dists) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);

        auto dist = chrono_types::make_shared<ChLinkDistance>();
        dist->SetName(model_prefix + item.first);
        dist->Initialize(body1, body2, false, model_frame * item.second.point1, model_frame * item.second.point2);
        sys.AddLink(dist);
        item.second.dist.push_back(dist);
        m_output_data.constraints.push_back(dist);
    }

    // Create TSDAs
    if (m_verbose && !m_tsdas.empty())
        cout << "Create TSDAs" << endl;
    for (auto& item : m_tsdas) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);
        auto tsda = chrono_types::make_shared<ChLinkTSDA>();
        tsda->SetName(model_prefix + item.first);
        tsda->Initialize(body1, body2, false, model_frame * item.second.point1, model_frame * item.second.point2);
        tsda->SetRestLength(item.second.free_length);
        tsda->RegisterForceFunctor(item.second.force);
        sys.AddLink(tsda);
        item.second.tsda.push_back(tsda);
        m_output_data.tsdas.push_back(tsda);
    }

    // Create RSDAs
    if (m_verbose && !m_rsdas.empty())
        cout << "Create RSDAs" << endl;
    for (auto& item : m_rsdas) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);

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
        m_output_data.rsdas.push_back(rsda);
    }

    // Create body loads
    if (m_verbose && !m_body_loads.empty())
        cout << "Create body loads" << endl;
    for (auto& item : m_body_loads) {
        auto body = FindBody(item.second.body);
        std::shared_ptr<ChLoadCustom> load;
        switch (item.second.type) {
            case BodyLoadType::FORCE:
                load = chrono_types::make_shared<ChLoadBodyForce>(body, item.second.value, item.second.local_load,
                                                                  item.second.point, item.second.local_point);
                break;
            case BodyLoadType::TORQUE:
                load = chrono_types::make_shared<ChLoadBodyTorque>(body, item.second.value, item.second.local_load);
                break;
        }
        load->SetName(model_prefix + item.first);
        load_container->Add(load);
        item.second.load.push_back(load);
        m_output_data.loads.push_back(load);
    }

    // Create linear motors
    if (m_verbose && !m_linmotors.empty())
        cout << "Create linear motors" << endl;
    for (auto& item : m_linmotors) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);

        ChMatrix33d rot;
        rot.SetFromAxisX(item.second.axis);
        ChQuaterniond quat = rot.GetQuaternion() * QuatFromAngleY(CH_PI_2);

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
        m_output_data.lin_motors.push_back(motor);
    }

    // Create rotation motors
    if (m_verbose && !m_rotmotors.empty())
        cout << "Create rotational motors" << endl;
    for (auto& item : m_rotmotors) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);

        ChMatrix33d rot;
        rot.SetFromAxisX(item.second.axis);
        ChQuaterniond quat = rot.GetQuaternion() * QuatFromAngleY(CH_PI_2);

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
        m_output_data.rot_motors.push_back(motor);
    }

    // Create body collision models
    for (auto& item : m_bodies) {
        if (item.second.geometry->HasCollision())
            item.second.geometry->CreateCollisionShapes(item.second.body[m_instance_index], 0, sys.GetContactMethod());
    }

    // Create visualization assets
    for (auto& item : m_bodies)
        item.second.geometry->CreateVisualizationAssets(item.second.body[m_instance_index], m_sim.visualization.type);
    for (auto& item : m_tsdas)
        item.second.geometry->CreateVisualizationAssets(item.second.tsda[m_instance_index]);
    for (auto& item : m_dists)
        item.second.dist[m_instance_index]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    return m_instance_index;
}

void ChParserMbsYAML::Depopulate(ChSystem& sys, int instance_index) {
    for (auto& item : m_bodies) {
        ChAssertAlways(item.second.body.size() > instance_index);
        sys.Remove(item.second.body[instance_index]);
        item.second.body.erase(item.second.body.begin() + instance_index);
    }

    for (auto& item : m_joints) {
        ChAssertAlways(item.second.joint.size() > instance_index);
        ChJoint::Remove(item.second.joint[instance_index]);
        item.second.joint.erase(item.second.joint.begin() + instance_index);
    }

    for (auto& item : m_dists) {
        ChAssertAlways(item.second.dist.size() > instance_index);
        sys.Remove(item.second.dist[instance_index]);
        item.second.dist.erase(item.second.dist.begin() + instance_index);
    }

    for (auto& item : m_tsdas) {
        ChAssertAlways(item.second.tsda.size() > instance_index);
        sys.Remove(item.second.tsda[instance_index]);
        item.second.tsda.erase(item.second.tsda.begin() + instance_index);
    }

    for (auto& item : m_rsdas) {
        ChAssertAlways(item.second.rsda.size() > instance_index);
        sys.Remove(item.second.rsda[instance_index]);
        item.second.rsda.erase(item.second.rsda.begin() + instance_index);
    }
}

// -----------------------------------------------------------------------------

void ChParserMbsYAML::SaveOutput(ChSystem& sys, int frame) {
    if (m_sim.output.type == ChOutput::Type::NONE)
        return;

    // Create the output DB if needed
    if (!m_output_db) {
        std::string filename = m_sim.output.dir + "/" + m_name;
        if (!m_output_dir.empty())
            filename = m_output_dir + "/" + filename;
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\nOutput file: " << filename << endl;
        }
        switch (m_sim.output.type) {
            case ChOutput::Type::ASCII:
                m_output_db = chrono_types::make_shared<ChOutputASCII>(filename + ".txt");
                break;
            case ChOutput::Type::HDF5:
#ifdef CHRONO_HAS_HDF5
                m_output_db = chrono_types::make_shared<ChOutputHDF5>(filename + ".h5", m_sim.output.mode);
                break;
#else
                return;
#endif
        }
        m_output_db->Initialize();
    }

    // Output simulation results at current frame
    m_output_db->WriteTime(frame, sys.GetChTime());

    m_output_db->WriteBodies(m_output_data.bodies);
    m_output_db->WriteJoints(m_output_data.joints);
    m_output_db->WriteBodyBodyLoads(m_output_data.bushings);
    ////m_output_db->WriteConstraints(m_output_data.constraints);
    m_output_db->WriteLinSprings(m_output_data.tsdas);
    m_output_db->WriteRotSprings(m_output_data.rsdas);
    m_output_db->WriteLinMotors(m_output_data.lin_motors);
    m_output_db->WriteRotMotors(m_output_data.rot_motors);
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
      sharpness_factor(1.0) {}

ChParserMbsYAML::IntegratorParams::IntegratorParams()
    : type(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED),
      rtol(1e-4),
      atol_states(1e-4),
      atol_multipliers(1e2),
      max_iterations(50),
      use_stepsize_control(false),
      use_modified_newton(false) {}

ChParserMbsYAML::VisParams::VisParams()
    : type(VisualizationType::MESH),
      render(false),
      render_fps(120),
      camera_vertical(CameraVerticalDir::Z),
      camera_location({0, -1, 0}),
      camera_target({0, 0, 0}),
      enable_shadows(true) {}

ChParserMbsYAML::OutputParameters::OutputParameters()
    : type(ChOutput::Type::NONE), mode(ChOutput::Mode::FRAMES), fps(100), dir(".") {}

ChParserMbsYAML::SimParams::SimParams()
    : gravity({0, 0, -9.8}),
      contact_method(ChContactMethod::NSC),
      num_threads_chrono(1),
      num_threads_collision(1),
      num_threads_eigen(1),
      num_threads_pardiso(1),
      time_step(1e-3),
      end_time(-1),
      enforce_realtime(false) {}

void ChParserMbsYAML::SolverParams::PrintInfo() {
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
            cout << "  overrelaxation factor:        " << overrelaxation_factor << endl;
            cout << "  sharpness factor:             " << sharpness_factor << endl;
            break;
        case ChSolver::Type::BICGSTAB:
        case ChSolver::Type::MINRES:
        case ChSolver::Type::GMRES:
            cout << "  max iterations:               " << max_iterations << endl;
            cout << "  tolerance:                    " << tolerance << endl;
            cout << "  use diagonal preconditioner?  " << std::boolalpha << enable_diagonal_preconditioner << endl;
            break;
        case ChSolver::Type::PARDISO_MKL:
        case ChSolver::Type::MUMPS:
            cout << "  lock sparsity pattern?        " << std::boolalpha << lock_sparsity_pattern << endl;
            break;
    }
}

void ChParserMbsYAML::IntegratorParams::PrintInfo() {
    cout << "integrator" << endl;
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

void ChParserMbsYAML::SimParams::PrintInfo() {
    cout << "contact method:         " << (contact_method == ChContactMethod::NSC ? "NSC" : "SMC") << endl;
    cout << endl;
    cout << "simulation end time:    " << (end_time < 0 ? "infinite" : std::to_string(end_time)) << endl;
    cout << "integration time step:  " << time_step << endl;
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
    cout << endl;
    visualization.PrintInfo();
    cout << endl;
    output.PrintInfo();
}

void ChParserMbsYAML::VisParams::PrintInfo() {
    if (!render) {
        cout << "no run-time visualization" << endl;
        return;
    }

    cout << "run-time visualization" << endl;
    cout << "  type:                 " << utils::ChBodyGeometry::GetVisualizationTypeAsString(type) << endl;
    cout << "  render FPS:           " << render_fps << endl;
    cout << "  enable shadows?       " << std::boolalpha << enable_shadows << endl;
    cout << "  camera vertical dir:  " << (camera_vertical == CameraVerticalDir::Y ? "Y" : "Z") << endl;
    cout << "  camera location:      " << camera_location << endl;
    cout << "  camera target:        " << camera_target << endl;
}

void ChParserMbsYAML::OutputParameters::PrintInfo() {
    if (type == ChOutput::Type::NONE) {
        cout << "no output" << endl;
        return;
    }

    cout << "output" << endl;
    cout << "  type:                 " << ChOutput::GetOutputTypeAsString(type) << endl;
    cout << "  mode:                 " << ChOutput::GetOutputModeAsString(mode) << endl;
    cout << "  output FPS:           " << fps << endl;
    cout << "  outut directory:      " << dir << endl;
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

ChParserMbsYAML::JointParams::JointParams()
    : type(ChJoint::Type::LOCK),
      body1(""),
      body2(""),
      frame(ChFramed(VNULL, QUNIT)),
      bdata(nullptr),
      is_kinematic(true) {}

ChParserMbsYAML::DistanceConstraintParams::DistanceConstraintParams()
    : body1(""), body2(""), point1(VNULL), point2(VNULL) {}

ChParserMbsYAML::TsdaParams::TsdaParams()
    : body1(""), body2(""), point1(VNULL), point2(VNULL), free_length(0), force(nullptr) {}

ChParserMbsYAML::RsdaParams::RsdaParams()
    : body1(""), body2(""), pos(VNULL), axis(ChVector3d(0, 0, 1)), free_angle(0), torque(nullptr) {}

ChParserMbsYAML::BodyLoadParams::BodyLoadParams()
    : type(BodyLoadType::FORCE), body(""), local_load(true), local_point(true), value(VNULL), point(VNULL) {}

ChParserMbsYAML::MotorLinearParams::MotorLinearParams()
    : actuation_type(MotorActuation::NONE),
      actuation_function(chrono_types::make_shared<ChFunctionConst>(0.0)),
      body1(""),
      body2(""),
      pos(VNULL),
      axis(ChVector3d(0, 0, 1)),
      guide(ChLinkMotorLinear::GuideConstraint::PRISMATIC) {}

ChParserMbsYAML::MotorRotationParams::MotorRotationParams()
    : actuation_type(MotorActuation::NONE),
      actuation_function(chrono_types::make_shared<ChFunctionConst>(0.0)),
      body1(""),
      body2(""),
      pos(VNULL),
      axis(ChVector3d(0, 0, 1)),
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

    cout << "      collision? " << (collision ? "yes" : "no") << endl;
    cout << "      vis prims? " << (vis_prims ? "yes" : "no") << endl;
    cout << "      vis mesh?  " << (vis_mesh ? "yes" : "no") << endl;

    //// TODO
}

void ChParserMbsYAML::BodyParams::PrintInfo(const std::string& name) {
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

void ChParserMbsYAML::JointParams::PrintInfo(const std::string& name) {
    cout << "  name:           " << name << endl;
    cout << "     type:        " << ChJoint::GetTypeString(type);
    cout << " (" << (is_kinematic ? "kinematic joint" : "bushing") << ")" << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     joint frame: " << frame.GetPos() << " | " << frame.GetRot() << endl;
}

void ChParserMbsYAML::DistanceConstraintParams::PrintInfo(const std::string& name) {
    cout << "  name:           " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     point1:      " << point1 << endl;
    cout << "     point2:      " << point2 << endl;
}

void ChParserMbsYAML::TsdaParams::PrintInfo(const std::string& name) {
    cout << "  name:           " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     point1:      " << point1 << endl;
    cout << "     point2:      " << point2 << endl;
    cout << "     free_length: " << free_length << endl;
}

void ChParserMbsYAML::RsdaParams::PrintInfo(const std::string& name) {
    cout << "  name:           " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     pos:         " << pos << endl;
    cout << "     axis:        " << axis << endl;
    cout << "     free_angle:  " << free_angle << endl;
}

void ChParserMbsYAML::BodyLoadParams::PrintInfo(const std::string& name) {
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

void ChParserMbsYAML::MotorLinearParams::PrintInfo(const std::string& name) {
    cout << "  name:           " << name << endl;
    cout << "     actuation:   " << GetMotorActuationTypeString(actuation_type) << endl;
    cout << "     guide:       " << ChLinkMotorLinear::GetGuideTypeString(guide) << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     pos:         " << pos << endl;
    cout << "     axis:        " << axis << endl;
}

void ChParserMbsYAML::MotorRotationParams::PrintInfo(const std::string& name) {
    cout << "  name:           " << name << endl;
    cout << "     actuation:   " << GetMotorActuationTypeString(actuation_type) << endl;
    cout << "     spindle:     " << ChLinkMotorRotation::GetSpindleTypeString(spindle) << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     pos:         " << pos << endl;
    cout << "     axis:        " << axis << endl;
}

// =============================================================================

static void PrintNodeType(const YAML::Node& node) {
    switch (node.Type()) {
        case YAML::NodeType::Null:
            cout << " Null" << endl;
            break;
        case YAML::NodeType::Scalar:
            cout << " Scalar" << endl;
            break;
        case YAML::NodeType::Sequence:
            cout << " Sequence" << endl;
            break;
        case YAML::NodeType::Map:
            cout << " Map" << endl;
            break;
        case YAML::NodeType::Undefined:
            cout << " Undefined" << endl;
            break;
    }
}

std::string ChParserMbsYAML::GetDatafilePath(const std::string& filename) {
    std::string full_filename = "";
    switch (m_data_path) {
        case DataPathType::ABS:
            full_filename = filename;
            break;
        case DataPathType::REL:
            full_filename = m_script_directory + "/" + m_rel_path + "/" + filename;
            break;
    }

    cout << "File: " << full_filename << endl;
    auto filepath = filesystem::path(full_filename);

    ChAssertAlways(filepath.exists());
    ChAssertAlways(filepath.is_file());

    return full_filename;
}

ChVector3d ChParserMbsYAML::ReadVector(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChVector3d(a[0].as<double>(), a[1].as<double>(), a[2].as<double>());
}

ChQuaterniond ChParserMbsYAML::ReadRotation(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3 || a.size() == 4);

    return (a.size() == 3) ? ReadCardanAngles(a, use_degrees) : ReadQuaternion(a);
}

ChQuaterniond ChParserMbsYAML::ReadQuaternion(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 4);
    return ChQuaterniond(a[0].as<double>(), a[1].as<double>(), a[2].as<double>(), a[3].as<double>());
}

ChQuaterniond ChParserMbsYAML::ReadCardanAngles(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    ChVector3d angles = ReadVector(a);

    if (use_degrees)
        angles *= CH_DEG_TO_RAD;

    ChQuaterniond q1 = QuatFromAngleZ(angles.x());  // roll
    ChQuaterniond q2 = QuatFromAngleY(angles.y());  // pitch
    ChQuaterniond q3 = QuatFromAngleX(angles.z());  // yaw

    ChQuaterniond q = q1 * q2 * q3;

    return q;
}

ChCoordsysd ChParserMbsYAML::ReadCoordinateSystem(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a["location"]);
    ChAssertAlways(a["orientation"]);
    return ChCoordsysd(ReadVector(a["location"]), ReadRotation(a["orientation"], use_degrees));
}

ChColor ChParserMbsYAML::ReadColor(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChColor(a[0].as<float>(), a[1].as<float>(), a[2].as<float>());
}

// -----------------------------------------------------------------------------

ChParserMbsYAML::DataPathType ChParserMbsYAML::ReadDataPathType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "RELATIVE")
        return DataPathType::REL;
    else
        return DataPathType::ABS;
}

ChSolver::Type ChParserMbsYAML::ReadSolverType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "BARZILAI_BORWEIN")
        return ChSolver::Type::BARZILAIBORWEIN;
    if (type == "PSOR")
        return ChSolver::Type::PSOR;
    if (type == "APGD")
        return ChSolver::Type::APGD;
    if (type == "MINRES")
        return ChSolver::Type::MINRES;
    if (type == "GMRES")
        return ChSolver::Type::GMRES;
    if (type == "BICGSTAB")
        return ChSolver::Type::BICGSTAB;
    if (type == "PARDISO")
        return ChSolver::Type::PARDISO_MKL;
    if (type == "MUMPS")
        return ChSolver::Type::MUMPS;
    if (type == "SPARSE_LU")
        return ChSolver::Type::SPARSE_LU;
    if (type == "SPARSE_QR")
        return ChSolver::Type::SPARSE_QR;

    cerr << "Unknown solver type: " << a.as<std::string>() << endl;
    throw std::runtime_error("Invalid solver type");
}

ChTimestepper::Type ChParserMbsYAML::ReadIntegratorType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "EULER_IMPLICIT_LINEARIZED")
        return ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    if (type == "EULER_IMPLICIT_PROJECTED")
        return ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
    if (type == "EULER_IMPLICIT")
        return ChTimestepper::Type::EULER_IMPLICIT;
    if (type == "HHT")
        return ChTimestepper::Type::HHT;

    cerr << "Unknown integrator type: " << a.as<std::string>() << endl;
    throw std::runtime_error("Invalid integrator type");
}

VisualizationType ChParserMbsYAML::ReadVisualizationType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "NONE")
        return VisualizationType::NONE;
    if (type == "PRIMITIVES")
        return VisualizationType::PRIMITIVES;
    if (type == "MODEL_FILE")
        return VisualizationType::MESH;
    if (type == "COLLISION")
        return VisualizationType::COLLISION;
    return VisualizationType::NONE;
}

ChOutput::Type ChParserMbsYAML::ReadOutputType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "ASCII")
        return ChOutput::Type::ASCII;
    if (type == "HDF5")
        return ChOutput::Type::HDF5;
    return ChOutput::Type::NONE;
}

ChOutput::Mode ChParserMbsYAML::ReadOutputMode(const YAML::Node& a) {
    auto mode = ToUpper(a.as<std::string>());
    if (mode == "SERIES")
        return ChOutput::Mode::SERIES;
    if (mode == "FRAMES")
        return ChOutput::Mode::FRAMES;
    return ChOutput::Mode::FRAMES;
}

// -----------------------------------------------------------------------------

ChContactMaterialData ChParserMbsYAML::ReadMaterialData(const YAML::Node& mat) {
    ChContactMaterialData minfo;

    if (mat["coefficient_of_friction"])
        minfo.mu = mat["coefficient_of_friction"].as<float>();
    if (mat["coefficient_of_restitution"])
        minfo.cr = mat["coefficient_of_restitution"].as<float>();

    if (mat["properties"]) {
        ChAssertAlways(mat["properties"]["Young_modulus"]);
        ChAssertAlways(mat["properties"]["Poisson_ratio"]);
        minfo.Y = mat["properties"]["Young_modulus"].as<float>();
        minfo.nu = mat["properties"]["Poisson_ratio"].as<float>();
    }

    if (mat["Coefficients"]) {
        ChAssertAlways(mat["coefficients"]["normal_stiffness"]);
        ChAssertAlways(mat["coefficients"]["normal_damping"]);
        ChAssertAlways(mat["coefficients"]["tangential_stiffness"]);
        ChAssertAlways(mat["coefficients"]["tangential_damping"]);
        minfo.kn = mat["coefficients"]["normal_stiffness"].as<float>();
        minfo.gn = mat["coefficients"]["normal_damping"].as<float>();
        minfo.kt = mat["coefficients"]["tangential_stiffness"].as<float>();
        minfo.gt = mat["coefficients"]["tangential_damping"].as<float>();
    }

    return minfo;
}

ChJoint::Type ChParserMbsYAML::ReadJointType(const YAML::Node& a) {
    std::string type = ToUpper(a.as<std::string>());
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

int FindMaterial(const std::string& name, const std::unordered_map<std::string, size_t> materials) {
    auto m = materials.find(name);
    if (m == materials.end()) {
        cerr << "Cannot find contact material with name: " << name << endl;
        throw std::runtime_error("Invalid contact material name");
    }
    return (int)m->second;
}

std::shared_ptr<utils::ChBodyGeometry> ChParserMbsYAML::ReadGeometry(const YAML::Node& d) {
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();

    // Read contact information
    if (d["contact"]) {
        ChAssertAlways(d["contact"]["materials"]);
        ChAssertAlways(d["contact"]["shapes"]);

        // Read contact material information
        ChAssertAlways(d["contact"]["materials"].IsSequence());
        size_t num_mats = d["contact"]["materials"].size();

        std::unordered_map<std::string, size_t> materials;
        for (size_t i = 0; i < num_mats; i++) {
            ChAssertAlways(d["contact"]["materials"][i]["name"]);
            ChContactMaterialData mat_data = ChParserMbsYAML::ReadMaterialData(d["contact"]["materials"][i]);
            geometry->materials.push_back(mat_data);
            materials.insert({d["contact"]["materials"][i]["name"].as<std::string>(), i});
        }

        // Read contact shapes
        ChAssertAlways(d["contact"]["shapes"].IsSequence());
        size_t num_shapes = d["contact"]["shapes"].size();

        for (size_t i = 0; i < num_shapes; i++) {
            const YAML::Node& shape = d["contact"]["shapes"][i];
            ChAssertAlways(shape["type"]);
            ChAssertAlways(shape["material"]);
            std::string type = ToUpper(shape["type"].as<std::string>());
            int matID = FindMaterial(shape["material"].as<std::string>(), materials);

            if (type == "SPHERE") {
                ChAssertAlways(shape["location"]);
                ChAssertAlways(shape["radius"]);
                ChVector3d pos = ChParserMbsYAML::ReadVector(shape["location"]);
                double radius = shape["radius"].as<double>();
                geometry->coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(pos, radius, matID));
            } else if (type == "BOX") {
                ChAssertAlways(shape["location"]);
                ChAssertAlways(shape["orientation"]);
                ChAssertAlways(shape["dimensions"]);
                ChVector3d pos = ChParserMbsYAML::ReadVector(shape["location"]);
                ChQuaterniond rot = ChParserMbsYAML::ReadRotation(shape["orientation"], m_use_degrees);
                ChVector3d dims = ChParserMbsYAML::ReadVector(shape["dimensions"]);
                geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(pos, rot, dims, matID));
            } else if (type == "CYLINDER") {
                ChAssertAlways(shape["location"]);
                ChAssertAlways(shape["axis"]);
                ChAssertAlways(shape["radius"]);
                ChAssertAlways(shape["length"]);
                ChVector3d pos = ChParserMbsYAML::ReadVector(shape["location"]);
                ChVector3d axis = ChParserMbsYAML::ReadVector(shape["axis"]);
                double radius = shape["radius"].as<double>();
                double length = shape["length"].as<double>();
                geometry->coll_cylinders.push_back(
                    utils::ChBodyGeometry::CylinderShape(pos, axis, radius, length, matID));
            } else if (type == "HULL") {
                ChAssertAlways(shape["filename"]);
                std::string filename = shape["filename"].as<std::string>();
                geometry->coll_hulls.push_back(
                    utils::ChBodyGeometry::ConvexHullsShape(GetDatafilePath(filename), matID));
            } else if (type == "MESH") {
                ChAssertAlways(shape["filename"]);
                std::string filename = shape["filename"].as<std::string>();
                ChVector3d pos = VNULL;
                ChQuaterniond rot = QUNIT;
                double scale = 1;
                double radius = 0;
                if (shape["location"])
                    pos = ChParserMbsYAML::ReadVector(shape["location"]);
                if (shape["orientation"])
                    rot = ChParserMbsYAML::ReadRotation(shape["orientation"], m_use_degrees);
                if (shape["scale"])
                    scale = shape["scale"].as<double>();
                if (shape["contact_radius"])
                    radius = shape["contact_radius"].as<double>();
                geometry->coll_meshes.push_back(
                    utils::ChBodyGeometry::TrimeshShape(pos, rot, GetDatafilePath(filename), scale, radius, matID));
            }
        }
    }

    // Read visualization
    if (d["visualization"]) {
        if (d["visualization"]["model_file"]) {
            std::string filename = d["visualization"]["model_file"].as<std::string>();
            geometry->vis_model_file = GetDatafilePath(filename);
        }
        if (d["visualization"]["shapes"]) {
            ChAssertAlways(d["visualization"]["shapes"].IsSequence());
            size_t num_shapes = d["visualization"]["shapes"].size();

            for (size_t i = 0; i < num_shapes; i++) {
                const YAML::Node& shape = d["visualization"]["shapes"][i];
                std::string type = ToUpper(shape["type"].as<std::string>());
                ChColor color(-1, -1, -1);
                if (shape["color"]) {
                    color = ReadColor(shape["color"]);
                }
                if (type == "SPHERE") {
                    ChAssertAlways(shape["location"]);
                    ChAssertAlways(shape["radius"]);
                    ChVector3d pos = ChParserMbsYAML::ReadVector(shape["location"]);
                    double radius = shape["radius"].as<double>();
                    auto sphere = utils::ChBodyGeometry::SphereShape(pos, radius);
                    sphere.color = color;
                    geometry->vis_spheres.push_back(sphere);
                } else if (type == "BOX") {
                    ChAssertAlways(shape["location"]);
                    ChAssertAlways(shape["orientation"]);
                    ChAssertAlways(shape["dimensions"]);
                    ChVector3d pos = ChParserMbsYAML::ReadVector(shape["location"]);
                    ChQuaterniond rot = ChParserMbsYAML::ReadRotation(shape["orientation"], m_use_degrees);
                    ChVector3d dims = ChParserMbsYAML::ReadVector(shape["dimensions"]);
                    auto box = utils::ChBodyGeometry::BoxShape(pos, rot, dims);
                    box.color = color;
                    geometry->vis_boxes.push_back(box);
                } else if (type == "CYLINDER") {
                    ChAssertAlways(shape["location"]);
                    ChAssertAlways(shape["axis"]);
                    ChAssertAlways(shape["radius"]);
                    ChAssertAlways(shape["length"]);
                    ChVector3d pos = ChParserMbsYAML::ReadVector(shape["location"]);
                    ChVector3d axis = ChParserMbsYAML::ReadVector(shape["axis"]);
                    double radius = shape["radius"].as<double>();
                    double length = shape["length"].as<double>();
                    auto cylinder = utils::ChBodyGeometry::CylinderShape(pos, axis, radius, length);
                    cylinder.color = color;
                    geometry->vis_cylinders.push_back(cylinder);
                } else if (type == "MESH") {
                    ChAssertAlways(shape["filename"]);
                    std::string filename = shape["filename"].as<std::string>();
                    ChVector3d pos = VNULL;
                    ChQuaterniond rot = QUNIT;
                    double scale = 1;
                    if (shape["location"])
                        pos = ChParserMbsYAML::ReadVector(shape["location"]);
                    if (shape["orientation"])
                        rot = ChParserMbsYAML::ReadRotation(shape["orientation"], m_use_degrees);
                    if (shape["scale"])
                        scale = shape["scale"].as<double>();
                    auto mesh = utils::ChBodyGeometry::TrimeshShape(pos, rot, GetDatafilePath(filename), scale);
                    mesh.color = color;
                    geometry->vis_meshes.push_back(mesh);
                }
            }
        }
    }

    return geometry;
}

std::shared_ptr<utils::ChTSDAGeometry> ChParserMbsYAML::ReadTSDAGeometry(const YAML::Node& d) {
    auto geometry = chrono_types::make_shared<utils::ChTSDAGeometry>();

    if (d["visualization"]) {
        ChAssertAlways(d["visualization"]["type"]);
        std::string type = ToUpper(d["visualization"]["type"].as<std::string>());
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
            geometry->vis_spring =
                chrono_types::make_shared<utils::ChTSDAGeometry::SpringShape>(radius, resolution, turns);
        } else {
            cerr << "Incorrect TSDA visualization shape type: " << d["visualization"]["type"].as<std::string>() << endl;
            throw std::runtime_error("Incorrect TSDA visualization shape type");
        }
    }

    return geometry;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChLinkTSDA::ForceFunctor> ChParserMbsYAML::ReadTSDAFunctor(const YAML::Node& tsda,
                                                                           double& free_length) {
    enum class FunctorType {
        LinearSpring,
        NonlinearSpring,
        LinearDamper,
        NonlinearDamper,
        DegressiveDamper,
        LinearSpringDamper,
        NonlinearSpringDamper,
        MapSpringDamper,
        Unknown
    };

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
            ChAssertAlways(tsda["map_data"].IsSequence() &&
                           tsda["map_data"][0].size() == tsda["deformation"].size() + 1);
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

std::shared_ptr<ChLinkRSDA::TorqueFunctor> ChParserMbsYAML::ReadRSDAFunctor(const YAML::Node& rsda,
                                                                            double& free_angle) {
    enum class FunctorType {
        LinearSpring,
        NonlinearSpring,
        LinearDamper,
        NonlinearDamper,
        LinearSpringDamper,
        NonlinearSpringDamper,
        Unknown
    };

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
    std::string type = ToUpper(a.as<std::string>());
    if (type == "TORQUE")
        return BodyLoadType::TORQUE;
    return BodyLoadType::FORCE;
}

ChParserMbsYAML::MotorActuation ChParserMbsYAML::ReadMotorActuationType(const YAML::Node& a) {
    std::string type = ToUpper(a.as<std::string>());
    if (type == "POSITION") {
        return MotorActuation::POSITION;
    } else if (type == "SPEED") {
        return MotorActuation::SPEED;
    } else if (type == "FORCE") {
        return MotorActuation::FORCE;
    } else {
        return MotorActuation::NONE;
    }
}

ChLinkMotorLinear::GuideConstraint ChParserMbsYAML::ReadMotorGuideType(const YAML::Node& a) {
    std::string type = ToUpper(a.as<std::string>());
    if (type == "FREE") {
        return ChLinkMotorLinear::GuideConstraint::FREE;
    } else if (type == "PRISMATIC") {
        return ChLinkMotorLinear::GuideConstraint::PRISMATIC;
    } else if (type == "SPHERICAL") {
        return ChLinkMotorLinear::GuideConstraint::SPHERICAL;
    } else {
        return ChLinkMotorLinear::GuideConstraint::PRISMATIC;
    }
}

ChLinkMotorRotation::SpindleConstraint ChParserMbsYAML::ReadMotorSpindleType(const YAML::Node& a) {
    std::string type = ToUpper(a.as<std::string>());
    if (type == "FREE") {
        return ChLinkMotorRotation::SpindleConstraint::FREE;
    } else if (type == "REVOLUTE") {
        return ChLinkMotorRotation::SpindleConstraint::REVOLUTE;
    } else if (type == "CYLINDRICAL") {
        return ChLinkMotorRotation::SpindleConstraint::CYLINDRICAL;
    } else {
        return ChLinkMotorRotation::SpindleConstraint::REVOLUTE;
    }
}

std::shared_ptr<ChFunction> ChParserMbsYAML::ReadFunction(const YAML::Node& a) {
    ChAssertAlways(a["type"]);
    auto type = ToUpper(a["type"].as<std::string>());

    bool repeat = false;
    double repeat_start = 0;
    double repeat_width = 0;
    double repeat_shift = 0;
    if (a["repeat"]) {
        repeat = true;
        repeat_start = a["repeat"]["start"].as<double>();
        repeat_width = a["repeat"]["width"].as<double>();
        repeat_shift = a["repeat"]["shift"].as<double>();
    }

    std::shared_ptr<ChFunction> f_base = nullptr;
    if (type == "CONSTANT") {
        ChAssertAlways(a["value"]);
        auto value = a["value"].as<double>();
        f_base = chrono_types::make_shared<ChFunctionConst>(value);
    } else if (type == "POLYNOMIAL") {
        ChAssertAlways(a["coefficients"]);
        ChAssertAlways(a["coefficients"].IsSequence());
        auto num_coeffs = a["coefficients"].size();
        std::vector<double> coeffs;
        for (size_t i = 0; i < num_coeffs; i++)
            coeffs.push_back(a["coefficients"][i].as<double>());
        auto f_poly = chrono_types::make_shared<ChFunctionPoly>();
        f_poly->SetCoefficients(coeffs);
        f_base = f_poly;
    } else if (type == "SINE") {
        ChAssertAlways(a["amplitude"]);
        ChAssertAlways(a["frequency"]);
        double amplitude = a["amplitude"].as<double>();
        double frequency = a["frequency"].as<double>();
        double phase = 0;
        if (a["phase"])
            phase = a["phase"].as<double>();
        if (m_use_degrees)
            phase *= CH_DEG_TO_RAD;
        f_base = chrono_types::make_shared<ChFunctionSine>(amplitude, frequency, phase);
    } else if (type == "RAMP") {
        ChAssertAlways(a["slope"]);
        auto slope = a["slope"].as<double>();
        double intercept = 0;
        if (a["intercept"])
            intercept = a["intercept"].as<double>();
        f_base = chrono_types::make_shared<ChFunctionRamp>(intercept, slope);
    } else if (type == "DATA") {
        ChAssertAlways(a["data"]);                 // key 'data' must exist
        ChAssertAlways(a["data"].IsSequence());    // 'data' must be an array
        ChAssertAlways(a["data"][0].size() == 2);  // each entry in 'data' must have size 2
        auto num_points = a["data"].size();
        auto f_interp = chrono_types::make_shared<ChFunctionInterp>();
        for (size_t i = 0; i < num_points; i++) {
            double t = a["data"][i][0].as<double>();
            double f = a["data"][i][1].as<double>();
            f_interp->AddPoint(t, f);
        }
        f_base = f_interp;
    } else {
        cerr << "Incorrect function type: " << a["type"] << endl;
        throw std::runtime_error("Incorrect function type");
    }
    //// TODO - more function types

    // Check if base function must be repeated
    if (repeat) {
        // Yes: construct and return repeated function
        auto f_repeat = chrono_types::make_shared<ChFunctionRepeat>(f_base, repeat_start, repeat_width, repeat_shift);
        return f_repeat;
    } else {
        // No: return base underlying function
        return f_base;
    }
}

}  // namespace parsers
}  // namespace chrono
