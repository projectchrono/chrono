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
// Test for universal joint
//
// =============================================================================

#include <ostream>
#include <fstream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// =============================================================================
// Local variables
//
static const std::string val_dir = "../RESULTS/";
static const std::string out_dir = val_dir + "universal_joint/";
static const std::string ref_dir = "testing/joints/universal_joint/";

// =============================================================================
// Prototypes of local functions
//
bool TestUniversal(const ChVector3d& jointLoc,
                   const ChQuaternion<>& jointRot,
                   double simTimeStep,
                   double outTimeStep,
                   const std::string& testName);
bool ValidateReference(const std::string& testName, const std::string& what, double tolerance);
bool ValidateConstraints(const std::string& testName, double tolerance);
bool ValidateEnergy(const std::string& testName, double tolerance);
ChWriterCSV OutStream();

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
    double sim_step = 5e-4;
    double out_step = 1e-2;

    std::string test_name;
    bool test_passed = true;

    // Case 1

    test_name = "Universal_Case01";
    TestUniversal(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2), sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    test_passed &= ValidateReference(test_name, "Vel", 2e-3);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(test_name, 1e-2);
    test_passed &= ValidateConstraints(test_name, 1e-5);

    // Case 2

    test_name = "Universal_Case02";
    TestUniversal(ChVector3d(0, 0, 0), QuatFromAngleY(CH_PI_2), sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    test_passed &= ValidateReference(test_name, "Vel", 2e-3);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(test_name, 1e-2);
    test_passed &= ValidateConstraints(test_name, 1e-5);

    // Case 3

    ////test_name = "Universal_Case03";
    ////TestUniversal(ChVector3d(0, 0, 0), QuatFromAngleAxis(CH_PI / 2, ChVector3d(0.707107, -0.707107, 0)), sim_step,
    /// out_step, test_name); /test_passed &= ValidateReference(test_name, "Pos", 2e-3); /test_passed &=
    /// ValidateReference(test_name, "Vel", 2e-3); /test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    ////test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Rforce", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Rtorque", 5e-4);
    ////test_passed &= ValidateEnergy(test_name, 1e-2);
    ////test_passed &= ValidateConstraints(test_name, 1e-5);

    // Return 0 if all tests passed and 1 otherwise
    std::cout << "\nUNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestUniversal(const ChVector3d& jointLoc,      // absolute location of joint
                   const ChQuaternion<>& jointRot,  // orientation of joint
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
    double length = 4.0;                   // length of pendulum
    ChVector3d inertiaXX(0.1, 0.1, 0.04);  // mass moments of inertia of pendulum (centroidal frame)
    double g = 9.80665;

    double timeRecord = 5;  // simulation length

    // Create the mechanical system
    // ----------------------------

    // Create a Chrono physical system: all bodies and constraints will be
    // handled by this ChSystem object.

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -g));

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(300);
    sys.GetSolver()->AsIterative()->SetTolerance(simTimeStep * 1e-4);

    // Create the ground body

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);

    // Create the pendulum body in an initial configuration at rest.
    // The pendulum CG is assumed to be at half its length.

    auto pendulum = chrono_types::make_shared<ChBody>();
    sys.AddBody(pendulum);
    pendulum->SetPos(jointLoc + jointRot.Rotate(ChVector3d(0, 0, -0.5 * length)));
    pendulum->SetRot(jointRot);
    pendulum->SetMass(mass);
    pendulum->SetInertiaXX(inertiaXX);

    // Create universal joint between pendulum and ground at "loc" in the global
    // reference frame.

    auto universalJoint = chrono_types::make_shared<ChLinkUniversal>();
    universalJoint->Initialize(ground, pendulum, ChFrame<>(jointLoc, jointRot));
    sys.AddLink(universalJoint);

    // Perform the simulation (record results option)
    // ------------------------------------------------

    // Create the CSV_Writer output objects (TAB delimited)
    ChWriterCSV out_pos = OutStream();
    ChWriterCSV out_vel = OutStream();
    ChWriterCSV out_acc = OutStream();

    ChWriterCSV out_quat = OutStream();
    ChWriterCSV out_avel = OutStream();
    ChWriterCSV out_aacc = OutStream();

    ChWriterCSV out_rfrc = OutStream();
    ChWriterCSV out_rtrq = OutStream();

    ChWriterCSV out_energy = OutStream();

    ChWriterCSV out_cnstr = OutStream();

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

    out_cnstr << "Time"
              << "Cnstr_1"
              << "Cnstr_2"
              << "Cnstr_3"
              << "Constraint_4" << std::endl;

    // Perform a system assembly to ensure we have the correct accelerations at the initial time.
    sys.DoAssembly(AssemblyAnalysis::Level::FULL);

    // Total energy at initial time.
    ChMatrix33<> inertia = pendulum->GetInertia();
    ChVector3d angVelLoc = pendulum->GetAngVelLocal();
    double transKE = 0.5 * mass * pendulum->GetPosDt().Length2();
    double rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
    double deltaPE = mass * g * (pendulum->GetPos().z() - jointLoc.z());
    double totalE0 = transKE + rotKE + deltaPE;

    // Simulation loop
    double simTime = 0;
    double outTime = 0;

    while (simTime <= timeRecord + simTimeStep / 2) {
        // Ensure that the final data point is recorded.
        if (simTime >= outTime - simTimeStep / 2) {
            // CM position, velocity, and acceleration (expressed in global frame).
            const ChVector3d& position = pendulum->GetPos();
            const ChVector3d& velocity = pendulum->GetPosDt();
            out_pos << simTime << position << std::endl;
            out_vel << simTime << velocity << std::endl;
            out_acc << simTime << pendulum->GetPosDt2() << std::endl;

            // Orientation, angular velocity, and angular acceleration (expressed in
            // global frame).
            out_quat << simTime << pendulum->GetRot() << std::endl;
            out_avel << simTime << pendulum->GetAngVelParent() << std::endl;
            out_aacc << simTime << pendulum->GetAngAccParent() << std::endl;

            // Reaction Force and Torque: acting on the ground body, as applied at the
            // joint location and expressed in the global frame.

            // Chrono returns the reaction force and torque on body 2 (as specified in
            // the joint Initialize() function), as applied at the joint location and
            // expressed in the joint frame. Here, the 2nd body is the pendulum.

            //    joint frame on 2nd body (pendulum), expressed in the body frame
            ChFrame<> linkCoordsys = universalJoint->GetFrame2Rel();

            //    reaction force and torque on pendulum, expressed in joint frame
            const auto& reaction = universalJoint->GetReaction2();
            ChVector3d reactForce = reaction.force;
            ChVector3d reactTorque = reaction.torque;
            //    reaction force and torque on pendulum, expressed in pendulum frame
            reactForce = linkCoordsys.TransformDirectionLocalToParent(reactForce);
            reactTorque = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
            //    reaction force and torque on pendulum, expresed in global frame
            reactForce = pendulum->TransformDirectionLocalToParent(reactForce);
            reactTorque = pendulum->TransformDirectionLocalToParent(reactTorque);
            //    reaction force and torque on ground, expressed in global frame
            reactForce *= -1.0;
            reactTorque *= -1.0;

            out_rfrc << simTime << reactForce << std::endl;
            out_rtrq << simTime << reactTorque << std::endl;

            // Conservation of Energy
            // Translational Kinetic Energy (1/2*m*||v||^2)
            // Rotational Kinetic Energy (1/2 w'*I*w)
            // Delta Potential Energy (m*g*dz)
            angVelLoc = pendulum->GetAngVelLocal();
            transKE = 0.5 * mass * velocity.Length2();
            rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
            deltaPE = mass * g * (position.z() - jointLoc.z());
            double totalE = transKE + rotKE + deltaPE;
            out_energy << simTime << transKE << rotKE << deltaPE << totalE - totalE0 << std::endl;

            // Constraint violations
            ChVectorDynamic<> C = universalJoint->GetConstraintViolation();
            out_cnstr << simTime << C(0) << C(1) << C(2) << C(3) << std::endl;

            // Increment output time
            outTime += outTimeStep;
        }

        // Advance simulation by one step
        sys.DoStepDynamics(simTimeStep);

        // Increment simulation time
        simTime += simTimeStep;
    }

    // Write output files
    out_pos.WriteToFile(out_dir + testName + "_CHRONO_Pos.txt", testName + "\n");
    out_vel.WriteToFile(out_dir + testName + "_CHRONO_Vel.txt", testName + "\n");
    out_acc.WriteToFile(out_dir + testName + "_CHRONO_Acc.txt", testName + "\n");

    out_quat.WriteToFile(out_dir + testName + "_CHRONO_Quat.txt", testName + "\n");
    out_avel.WriteToFile(out_dir + testName + "_CHRONO_Avel.txt", testName + "\n");
    out_aacc.WriteToFile(out_dir + testName + "_CHRONO_Aacc.txt", testName + "\n");

    out_rfrc.WriteToFile(out_dir + testName + "_CHRONO_Rforce.txt", testName + "\n");
    out_rtrq.WriteToFile(out_dir + testName + "_CHRONO_Rtorque.txt", testName + "\n");

    out_energy.WriteToFile(out_dir + testName + "_CHRONO_Energy.txt", testName + "\n");

    out_cnstr.WriteToFile(out_dir + testName + "_CHRONO_Constraints.txt", testName + "\n");

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

// Wrapper function for checking constraint violations.
//
bool ValidateConstraints(const std::string& testName,  // name of this test
                         double tolerance)             // validation tolerance
{
    std::string sim_file = out_dir + testName + "_CHRONO_Constraints.txt";
    utils::DataVector norms;

    bool check = utils::Validate(sim_file, utils::RMS_NORM, tolerance, norms);
    std::cout << "   validate Constraints" << (check ? ": Passed" : ": Failed") << "  [  ";
    for (size_t col = 0; col < norms.size(); col++)
        std::cout << norms[col] << "  ";
    std::cout << "  ]" << std::endl;

    return check;
}

// wrapper function for checking energy conservation.
//
bool ValidateEnergy(const std::string& testName,  // name of this test
                    double tolerance)             // validation tolerance
{
    std::string sim_file = out_dir + testName + "_CHRONO_Energy.txt";
    utils::DataVector norms;

    utils::Validate(sim_file, utils::RMS_NORM, tolerance, norms);

    bool check = norms[norms.size() - 1] <= tolerance;
    std::cout << "   validate Energy" << (check ? ": Passed" : ": Failed") << "  [  " << norms[norms.size() - 1]
              << "  ]" << std::endl;

    return check;
}

// =============================================================================
//
// Utility function to create a CSV output stream and set output format options.
//
ChWriterCSV OutStream() {
    ChWriterCSV out("\t");

    out.Stream().setf(std::ios::scientific | std::ios::showpos);
    out.Stream().precision(6);

    return out;
}
