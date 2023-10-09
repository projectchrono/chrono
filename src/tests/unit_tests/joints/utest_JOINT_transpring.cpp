// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Mike Taylor, Radu Serban
// =============================================================================
//
// Test for the translational spring damper (ChLinkTSDA)
//
// =============================================================================

#include <ostream>
#include <fstream>
#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// =============================================================================
// Local variables
//
static const std::string val_dir = "../RESULTS/";
static const std::string out_dir = val_dir + "transpring_force/";
static const std::string ref_dir = "testing/joints/transpring_force/";

// =============================================================================
// Prototypes of local functions
//
bool TestTranSpring(const ChVector<>& jointLocGnd,
                    const ChVector<>& jointLocPend,
                    const ChCoordsys<>& PendCSYS,
                    const double spring_coef,
                    const double damping_coef,
                    double simTimeStep,
                    double outTimeStep,
                    const std::string& testName);
bool ValidateReference(const std::string& testName, const std::string& what, double tolerance);
bool ValidateConstraints(const std::string& testName, double tolerance);
bool ValidateEnergy(const std::string& testName, double tolerance);
utils::CSV_writer OutStream();

// =============================================================================
//
// Main driver function for running the simulation and validating the results.
//
int main(int argc, char* argv[]) {
    // Create output directory (if it does not already exist)
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cout << "Error creating directory " << val_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Set the simulation and output step sizes
    double sim_step = 1e-4;
    double out_step = 1e-2;

    std::string test_name;
    bool test_passed = true;

    // Case 1 - Pendulum dropped from the origin with a spring connected between the CG and ground.

    test_name = "TranSpring_Case01";
    TestTranSpring(ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT), 10, 0.5,
                   sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-3);
    test_passed &= ValidateReference(test_name, "Vel", 3e-4);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-10);
    test_passed &= ValidateReference(test_name, "Avel", 1e-10);
    test_passed &= ValidateReference(test_name, "Aacc", 1e-10);
    test_passed &= ValidateReference(test_name, "Rforce", 5e-3);

    // Case 2 - Pendulum CG at Y = 2 with the spring connected between the CG and ground.

    test_name = "TranSpring_Case02";
    TestTranSpring(ChVector<>(0, 0, 0), ChVector<>(0, 2, 0), ChCoordsys<>(ChVector<>(0, 2, 0), QUNIT), 100, 5, sim_step,
                   out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-3);
    test_passed &= ValidateReference(test_name, "Vel", 3e-4);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-10);
    test_passed &= ValidateReference(test_name, "Avel", 1e-10);
    test_passed &= ValidateReference(test_name, "Aacc", 1e-10);
    test_passed &= ValidateReference(test_name, "Rforce", 5e-3);

    // Return 0 if all tests passed and 1 otherwise
    std::cout << std::endl << "UNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestTranSpring(
    const ChVector<>& jointLocGnd,   // absolute location of the distance constrain ground attachment point
    const ChVector<>& jointLocPend,  // absolute location of the distance constrain pendulum attachment point
    const ChCoordsys<>& PendCSYS,    // Coordinate system for the pendulum
    const double spring_coef,        // Linear Spring stiffness
    const double damping_coef,       // Damping Coefficent
    double simTimeStep,              // simulation time step
    double outTimeStep,              // output time step
    const std::string& testName)     // if true, also save animation data
{
    std::cout << "TEST: " << testName << std::endl;

    // Settings
    //---------

    // There are no units in Chrono, so values must be consistent
    // (MKS is used in this example)

    double mass = 1.0;                     // mass of pendulum
    ChVector<> inertiaXX(0.04, 0.1, 0.1);  // mass moments of inertia of pendulum (centroidal frame)
    double g = 9.80665;

    double timeRecord = 5;  // simulation length

    // Create the mechanical system
    // ----------------------------

    // Create a ChronoENGINE physical system: all bodies and constraints will be
    // handled by this ChSystem object.

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0.0, 0.0, -g));

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(100);
    sys.SetSolverForceTolerance(1e-4);

    // Create the ground body

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetBodyFixed(true);

    // Create the pendulum body in an initial configuration at rest, with an
    // orientatoin that matches the specified joint orientation and a position
    // consistent with the specified joint location.
    // The pendulum CG is assumed to be at half its length.

    auto pendulum = chrono_types::make_shared<ChBody>();
    sys.AddBody(pendulum);
    pendulum->SetPos(PendCSYS.pos);
    pendulum->SetRot(PendCSYS.rot);
    pendulum->SetMass(mass);
    pendulum->SetInertiaXX(inertiaXX);

    // Create a translational spring damper force between pendulum at "jointLocPend"
    // and ground at "jointLocGnd" in the global reference frame.
    // The free length is set equal to the inital distance between
    // "jointLocPend" and "jointLocGnd".

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(pendulum, ground, false, jointLocPend, jointLocGnd);
    spring->SetSpringCoefficient(spring_coef);
    spring->SetDampingCoefficient(damping_coef);
    sys.AddLink(spring);

    // Perform the simulation (record results option)
    // ------------------------------------------------

    // Create the CSV_Writer output objects (TAB delimited)
    utils::CSV_writer out_pos = OutStream();
    utils::CSV_writer out_vel = OutStream();
    utils::CSV_writer out_acc = OutStream();

    utils::CSV_writer out_quat = OutStream();
    utils::CSV_writer out_avel = OutStream();
    utils::CSV_writer out_aacc = OutStream();

    utils::CSV_writer out_rfrc = OutStream();
    utils::CSV_writer out_rtrq = OutStream();

    utils::CSV_writer out_energy = OutStream();

    // Write headers
    out_pos << "Time"
            << "X_Pos"
            << "Y_Pos"
            << "Z_Pos" << std::endl;
    out_vel << "Time"
            << "X_Vel"
            << "Y_Vel"
            << "Z_Vel" << std::endl;
    out_acc << "Time"
            << "X_Acc"
            << "Y_Acc"
            << "Z_Acc" << std::endl;

    out_quat << "Time"
             << "e0"
             << "e1"
             << "e2"
             << "e3" << std::endl;
    out_avel << "Time"
             << "X_AngVel"
             << "Y_AngVel"
             << "Z_AngVel" << std::endl;
    out_aacc << "Time"
             << "X_AngAcc"
             << "Y_AngAcc"
             << "Z_AngAcc" << std::endl;

    out_rfrc << "Time"
             << "X_Force"
             << "Y_Force"
             << "Z_Force" << std::endl;
    out_rtrq << "Time"
             << "X_Torque"
             << "Y_Torque"
             << "Z_Torque" << std::endl;

    out_energy << "Time"
               << "Transl_KE"
               << "Rot_KE"
               << "Delta_PE"
               << "KE+PE" << std::endl;

    // Perform a system assembly to ensure we have the correct accelerations at
    // the initial time.
    sys.DoFullAssembly();

    // Total energy at initial time.
    ChMatrix33<> inertia = pendulum->GetInertia();
    ChVector<> angVelLoc = pendulum->GetWvel_loc();
    double transKE = 0.5 * mass * pendulum->GetPos_dt().Length2();
    double rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
    double deltaPE = mass * g * (pendulum->GetPos().z() - PendCSYS.pos.z());
    double totalE0 = transKE + rotKE + deltaPE;

    // Simulation loop
    double simTime = 0;
    double outTime = 0;

    while (simTime <= timeRecord + simTimeStep / 2) {
        // Ensure that the final data point is recorded.
        if (simTime >= outTime - simTimeStep / 2) {
            // CM position, velocity, and acceleration (expressed in global frame).
            const ChVector<>& position = pendulum->GetPos();
            const ChVector<>& velocity = pendulum->GetPos_dt();
            out_pos << simTime << position << std::endl;
            out_vel << simTime << velocity << std::endl;
            out_acc << simTime << pendulum->GetPos_dtdt() << std::endl;

            // Orientation, angular velocity, and angular acceleration (expressed in
            // global frame).
            out_quat << simTime << pendulum->GetRot() << std::endl;
            out_avel << simTime << pendulum->GetWvel_par() << std::endl;
            out_aacc << simTime << pendulum->GetWacc_par() << std::endl;

            // Spring Force and Torque
            // Force is given as a scalar and is then transformed into a vector in global coordiantes
            // Torques are expressed in the link coordinate system. We convert them to
            // the coordinate system of Body2 (in our case this is the ground).
            ChVector<> springVector = spring->GetPoint2Abs() - spring->GetPoint1Abs();
            springVector.Normalize();
            double springForce = spring->GetForce();
            ChVector<> springForceGlobal = springForce * springVector;
            out_rfrc << simTime << springForceGlobal << std::endl;

            ChCoordsys<> linkCoordsys = spring->GetLinkRelativeCoords();
            ChVector<> reactTorque = spring->Get_react_torque();
            ChVector<> reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
            out_rtrq << simTime << reactTorqueGlobal << std::endl;

            // Conservation of Energy
            // Translational Kinetic Energy (1/2*m*||v||^2)
            // Rotational Kinetic Energy (1/2 w'*I*w)
            // Delta Potential Energy (m*g*dz)
            angVelLoc = pendulum->GetWvel_loc();
            transKE = 0.5 * mass * velocity.Length2();
            rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
            deltaPE = mass * g * (position.z() - PendCSYS.pos.z());
            double totalE = transKE + rotKE + deltaPE;
            out_energy << simTime << transKE << rotKE << deltaPE << totalE - totalE0 << std::endl;
            ;

            // Increment output time
            outTime += outTimeStep;
        }

        // Advance simulation by one step
        sys.DoStepDynamics(simTimeStep);

        // Increment simulation time
        simTime += simTimeStep;
    }

    // Write output files
    out_pos.write_to_file(out_dir + testName + "_CHRONO_Pos.txt", testName + "\n");
    out_vel.write_to_file(out_dir + testName + "_CHRONO_Vel.txt", testName + "\n");
    out_acc.write_to_file(out_dir + testName + "_CHRONO_Acc.txt", testName + "\n");

    out_quat.write_to_file(out_dir + testName + "_CHRONO_Quat.txt", testName + "\n");
    out_avel.write_to_file(out_dir + testName + "_CHRONO_Avel.txt", testName + "\n");
    out_aacc.write_to_file(out_dir + testName + "_CHRONO_Aacc.txt", testName + "\n");

    out_rfrc.write_to_file(out_dir + testName + "_CHRONO_Rforce.txt", testName + "\n");
    out_rtrq.write_to_file(out_dir + testName + "_CHRONO_Rtorque.txt", testName + "\n");

    out_energy.write_to_file(out_dir + testName + "_CHRONO_Energy.txt", testName + "\n");

    return true;
}

// =============================================================================
//
// Wrapper function for comparing the specified simulation quantities against a
// reference file.
//
bool ValidateReference(const std::string& testName,  // name of this test
                       const std::string& what,      // identifier for test quantity
                       double tolerance)             // validation tolerance
{
    std::string sim_file = out_dir + testName + "_CHRONO_" + what + ".txt";
    std::string ref_file = ref_dir + testName + "_ADAMS_" + what + ".txt";
    utils::DataVector norms;

    bool check = utils::Validate(sim_file, utils::GetValidationDataFile(ref_file), utils::RMS_NORM, tolerance, norms);
    std::cout << "   validate " << what << (check ? ": Passed" : ": Failed") << "  [  ";
    for (size_t col = 0; col < norms.size(); col++)
        std::cout << norms[col] << "  ";
    std::cout << "  ]" << std::endl;

    return check;
}

// =============================================================================
//
// Utility function to create a CSV output stream and set output format options.
//
utils::CSV_writer OutStream() {
    utils::CSV_writer out("\t");

    out.stream().setf(std::ios::scientific | std::ios::showpos);
    out.stream().precision(6);

    return out;
}
