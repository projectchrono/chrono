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
// Test for the revolute-spherical constraint
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
static const std::string out_dir = val_dir + "revsph_constraint/";
static const std::string ref_dir = "testing/joints/revsph_constraint/";

// =============================================================================
// Prototypes of local functions
//
bool TestRevSpherical(const ChVector3d& jointLocGnd,
                      const ChVector3d& jointRevAxis,
                      const ChVector3d& jointLocPend,
                      const ChCoordsys<>& PendCSYS,
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
    double sim_step = 1e-5;
    double out_step = 1e-2;

    std::string test_name;
    bool test_passed = true;

    // Case 1 - Revolute connect to ground at (0,0,0) and aligned with the global z axis
    //   Spherical joint at one end of a horizontal pendulum (2,0,0) with CG at (2,2,0)

    test_name = "RevSpherical_Case01";
    TestRevSpherical(ChVector3d(0, 0, 0), ChVector3d(0, 0, 1), ChVector3d(2, 0, 0),
                     ChCoordsys<>(ChVector3d(2, 2, 0), QUNIT), sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-4);
    test_passed &= ValidateReference(test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(test_name, "Acc", 1e-1);
    test_passed &= ValidateReference(test_name, "Quat", 1e-5);
    test_passed &= ValidateReference(test_name, "Avel", 1e-4);
    test_passed &= ValidateReference(test_name, "Aacc", 5e-1);
    test_passed &= ValidateReference(test_name, "Rforce_Body1", 5e-1);
    test_passed &= ValidateReference(test_name, "Rtorque_Body1", 5e-1);
    test_passed &= ValidateReference(test_name, "Rforce_Body2", 5e-1);
    test_passed &= ValidateReference(test_name, "Rtorque_Body2", 5e-1);
    test_passed &= ValidateEnergy(test_name, 1e-2);
    test_passed &= ValidateConstraints(test_name, 1e-5);

    // Case 2 - Revolute connect to ground at (1,2,3) and aligned with the global y=z axis
    //   Spherical joint at one end of a horizontal pendulum (3,2,3) with CG at (3,4,3)

    test_name = "RevSpherical_Case02";
    TestRevSpherical(ChVector3d(1, 2, 3), ChVector3d(0, 1, 1), ChVector3d(3, 2, 3),
                     ChCoordsys<>(ChVector3d(3, 4, 3), QUNIT), sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-4);
    test_passed &= ValidateReference(test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(test_name, "Acc", 1e-1);
    test_passed &= ValidateReference(test_name, "Quat", 1e-5);
    test_passed &= ValidateReference(test_name, "Avel", 1e-3);
    test_passed &= ValidateReference(test_name, "Aacc", 5e-1);
    test_passed &= ValidateReference(test_name, "Rforce_Body1", 5e-1);
    test_passed &= ValidateReference(test_name, "Rtorque_Body1", 5e-1);
    test_passed &= ValidateReference(test_name, "Rforce_Body2", 5e-1);
    test_passed &= ValidateReference(test_name, "Rtorque_Body2", 5e-1);
    test_passed &= ValidateEnergy(test_name, 1e-2);
    test_passed &= ValidateConstraints(test_name, 1e-5);

    // Return 0 if all tests passed and 1 otherwise
    std::cout << "\nUNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestRevSpherical(
    const ChVector3d& jointLocGnd,   // absolute location of the RevSpherical constrain ground attachment point
    const ChVector3d& jointRevAxis,  // absolute vector for the revolute axis of the RevSpherical constrain
    const ChVector3d& jointLocPend,  // absolute location of the distance constrain pendulum attachment point
    const ChCoordsys<>& PendCSYS,    // Coordinate system for the pendulum
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
    ChVector3d inertiaXX(0.1, 0.04, 0.1);  // mass moments of inertia of pendulum (centroidal frame)
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

    // Create a Revolute-Spherical constraint between pendulum at "jointLocPend"
    // and ground at "jointLocGnd" in the global reference frame.
    // The constrained distance is set equal to the inital distance between
    // "jointLocPend" and "jointLocGnd".

    auto revSphericalConstraint = chrono_types::make_shared<ChLinkRevoluteSpherical>();
    revSphericalConstraint->Initialize(ground, pendulum, false, jointLocGnd, jointRevAxis, jointLocPend, true);
    sys.AddLink(revSphericalConstraint);

    // Perform the simulation (record results option)
    // ------------------------------------------------

    // Create the CSV_Writer output objects (TAB delimited)
    ChWriterCSV out_pos = OutStream();
    ChWriterCSV out_vel = OutStream();
    ChWriterCSV out_acc = OutStream();

    ChWriterCSV out_quat = OutStream();
    ChWriterCSV out_avel = OutStream();
    ChWriterCSV out_aacc = OutStream();

    ChWriterCSV out_rfrc1 = OutStream();
    ChWriterCSV out_rtrq1 = OutStream();
    ChWriterCSV out_rfrc2 = OutStream();
    ChWriterCSV out_rtrq2 = OutStream();

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

    out_rfrc1 << "Time"
              << "X_Force"
              << "Y_Force"
              << "Z_Force" << std::endl;
    out_rtrq1 << "Time"
              << "X_Torque"
              << "Y_Torque"
              << "Z_Torque" << std::endl;

    out_rfrc2 << "Time"
              << "X_Force"
              << "Y_Force"
              << "Z_Force" << std::endl;
    out_rtrq2 << "Time"
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
              << "Cnstr_2" << std::endl;

    // Perform a system assembly to ensure we have the correct accelerations at the initial time.
    sys.DoAssembly(AssemblyAnalysis::Level::FULL);

    // Total energy at initial time.
    ChMatrix33<> inertia = pendulum->GetInertia();
    ChVector3d angVelLoc = pendulum->GetAngVelLocal();
    double transKE = 0.5 * mass * pendulum->GetPosDt().Length2();
    double rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
    double deltaPE = mass * g * (pendulum->GetPos().z() - PendCSYS.pos.z());
    double totalE0 = transKE + rotKE + deltaPE;

    // Simulation loop
    double simTime = 0;
    double outTime = 0;

    // timeRecord = .0001;
    while (simTime <= timeRecord + simTimeStep / 2) {
        // Ensure that the final data point is recorded.
        if (simTime >= outTime - simTimeStep / 2) {
            // CM position, velocity, and acceleration (expressed in global frame).
            const ChVector3d& position = pendulum->GetPos();
            const ChVector3d& velocity = pendulum->GetPosDt();
            out_pos << simTime << position << std::endl;
            out_vel << simTime << velocity << std::endl;
            out_acc << simTime << pendulum->GetPosDt2() << std::endl;

            // Orientation, angular velocity, and angular acceleration (expressed in global frame).
            out_quat << simTime << pendulum->GetRot() << std::endl;
            out_avel << simTime << pendulum->GetAngVelParent() << std::endl;
            out_aacc << simTime << pendulum->GetAngAccParent() << std::endl;

            //    reaction force and torque on body1, at frame 1, expressed in global frame
            const auto& linkFrame1 = revSphericalConstraint->GetFrame1Abs();
            const auto& reaction1_loc = revSphericalConstraint->GetReaction1();
            ChVector3d reactForceB1 = linkFrame1.TransformDirectionLocalToParent(reaction1_loc.force);
            ChVector3d reactTorqueB1 = linkFrame1.TransformDirectionLocalToParent(reaction1_loc.torque);

            //    reaction force and torque on body2, at frame 2, expressed in global frame
            const auto& linkFrame2 = revSphericalConstraint->GetFrame2Abs();
            const auto& reaction2_loc = revSphericalConstraint->GetReaction2();
            ChVector3d reactForceB2 = linkFrame2.TransformDirectionLocalToParent(reaction2_loc.force);
            ChVector3d reactTorqueB2 = linkFrame2.TransformDirectionLocalToParent(reaction2_loc.torque);

            out_rfrc1 << simTime << reactForceB1 << std::endl;
            out_rtrq1 << simTime << reactTorqueB1 << std::endl;
            out_rfrc2 << simTime << reactForceB2 << std::endl;
            out_rtrq2 << simTime << reactTorqueB2 << std::endl;

            // Conservation of Energy
            // Translational Kinetic Energy (1/2*m*||v||^2)
            // Rotational Kinetic Energy (1/2 w'*I*w)
            // Delta Potential Energy (m*g*dz)
            angVelLoc = pendulum->GetAngVelLocal();
            transKE = 0.5 * mass * velocity.Length2();
            rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
            deltaPE = mass * g * (position.z() - PendCSYS.pos.z());
            double totalE = transKE + rotKE + deltaPE;
            out_energy << simTime << transKE << rotKE << deltaPE << totalE - totalE0 << std::endl;

            // Constraint violations
            ChVectorDynamic<> C = revSphericalConstraint->GetConstraintViolation();
            out_cnstr << simTime << C(0) << C(1) << std::endl;

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

    out_rfrc1.WriteToFile(out_dir + testName + "_CHRONO_Rforce_Body1.txt", testName + "\n");
    out_rtrq1.WriteToFile(out_dir + testName + "_CHRONO_Rtorque_Body1.txt", testName + "\n");
    out_rfrc2.WriteToFile(out_dir + testName + "_CHRONO_Rforce_Body2.txt", testName + "\n");
    out_rtrq2.WriteToFile(out_dir + testName + "_CHRONO_Rtorque_Body2.txt", testName + "\n");

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
