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
// Test for the cylindrical joint
//
// =============================================================================

#include <ostream>
#include <fstream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

enum class eChLinkFormulation { Lock, Mate };

// =============================================================================
// Local variables
//
static const std::string val_dir = "../RESULTS/";
static const std::string out_dir = val_dir + "cylindrical_joint/";
static const std::string ref_dir = "testing/joints/cylindrical_joint/";

// =============================================================================
// Prototypes of local functions
//
bool TestCylindrical(const ChVector3d& jointLoc,
                     const ChQuaternion<>& jointRot,
                     eChLinkFormulation formulation,
                     double simTimeStep,
                     double outTimeStep,
                     const std::string& testName);
bool ValidateReference(const std::string& testName,
                       const std::string& refTestName,
                       const std::string& what,
                       double tolerance);
bool ValidateConstraints(const std::string& testName, double tolerance);
bool ValidateEnergy(const std::string& testName, double tolerance);
utils::ChWriterCSV OutStream();

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

    std::string ref_test_name;
    std::string chrono_test_name;
    bool test_passed = true;

    // Case 1 - Joint at the origin and aligned with the global frame.
    // In this case, the pendulum should fall straight down due to gravity

    ref_test_name = "Cylindrical_Case01";

    chrono_test_name = "Lock" + ref_test_name;
    TestCylindrical(ChVector3d(0, 0, 0), QUNIT, eChLinkFormulation::Lock, sim_step, out_step, chrono_test_name);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Pos", 1e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(chrono_test_name, 1e-1);
    test_passed &= ValidateConstraints(chrono_test_name, 1e-5);

    chrono_test_name = "Mate" + ref_test_name;
    TestCylindrical(ChVector3d(0, 0, 0), QUNIT, eChLinkFormulation::Mate, sim_step, out_step, chrono_test_name);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Pos", 1e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(chrono_test_name, 1e-1);
    test_passed &= ValidateConstraints(chrono_test_name, 1e-5);

    // Case 2 - Joint at (0,0,0) and aligned with the global Y axis.
    // In this case, the joint must be rotated -pi/2 about the global X-axis.
    // In this case, the cylindrical joint is acting like a revolute joint

    ref_test_name = "Cylindrical_Case02";

    chrono_test_name = "Lock" + ref_test_name;
    TestCylindrical(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2), eChLinkFormulation::Lock, sim_step, out_step,
                    chrono_test_name);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Pos", 1e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(chrono_test_name, 1e-2);
    test_passed &= ValidateConstraints(chrono_test_name, 1e-5);

    chrono_test_name = "Mate" + ref_test_name;
    TestCylindrical(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2), eChLinkFormulation::Mate, sim_step, out_step,
                    chrono_test_name);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Pos", 1e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(chrono_test_name, 1e-2);
    test_passed &= ValidateConstraints(chrono_test_name, 1e-5);

    // Case 3 - Joint at (1,2,3) and aligned with the global axis along Y = Z.
    // In this case, the joint must be rotated -pi/4 about the global X-axis.
    // In this case, the pendulum both translates and rotates about the joint's
    // z-axis.

    ref_test_name = "Cylindrical_Case03";

    chrono_test_name = "Lock" + ref_test_name;
    TestCylindrical(ChVector3d(1, 2, 3), QuatFromAngleX(-CH_PI_4), eChLinkFormulation::Lock, sim_step, out_step,
                    chrono_test_name);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Pos", 1e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rtorque", 5e-1);
    test_passed &= ValidateEnergy(chrono_test_name, 1e-1);
    test_passed &= ValidateConstraints(chrono_test_name, 1e-5);

    chrono_test_name = "Mate" + ref_test_name;
    TestCylindrical(ChVector3d(1, 2, 3), QuatFromAngleX(-CH_PI_4), eChLinkFormulation::Mate, sim_step, out_step,
                    chrono_test_name);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Pos", 1e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(chrono_test_name, ref_test_name, "Rtorque", 5e-1);
    test_passed &= ValidateEnergy(chrono_test_name, 1e-1);
    test_passed &= ValidateConstraints(chrono_test_name, 1e-5);

    // Return 0 if all tests passed and 1 otherwise
    std::cout << "\nUNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestCylindrical(const ChVector3d& jointLoc,      // absolute location of joint
                     const ChQuaternion<>& jointRot,  // orientation of joint
                     eChLinkFormulation formulation,  // joint formulation
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
    ChVector3d inertiaXX(0.04, 0.1, 0.1);  // mass moments of inertia of pendulum (centroidal frame)
    double g = 9.80665;

    double timeRecord = 5;  // simulation length

    // Create the mechanical system
    // ----------------------------

    // Create a ChronoENGINE physical system: all bodies and constraints will be
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
    pendulum->SetPos(jointLoc + jointRot.Rotate(ChVector3d(length / 2, 0, 0)));
    pendulum->SetRot(jointRot);
    pendulum->SetMass(mass);
    pendulum->SetInertiaXX(inertiaXX);

    // Create cylindrical joint between pendulum and ground at "loc" in the global
    // reference frame. The cylindrical joint's axis of rotation and translation
    // will be the Z axis of the specified rotation matrix.

    std::shared_ptr<ChLink> cylindricalJoint;
    switch (formulation) {
        case eChLinkFormulation::Lock:
            cylindricalJoint = chrono_types::make_shared<ChLinkLockCylindrical>();
            std::dynamic_pointer_cast<ChLinkLockCylindrical>(cylindricalJoint)
                ->Initialize(pendulum, ground, ChFrame<>(jointLoc, jointRot));
            sys.AddLink(cylindricalJoint);
            break;
        case eChLinkFormulation::Mate:
            cylindricalJoint = chrono_types::make_shared<ChLinkMateCylindrical>();
            std::dynamic_pointer_cast<ChLinkMateCylindrical>(cylindricalJoint)
                ->Initialize(pendulum, ground, ChFrame<>(jointLoc, jointRot));
            sys.AddLink(cylindricalJoint);
            break;
        default:
            break;
    }

    // Perform the simulation (record results option)
    // ------------------------------------------------

    // Create the CSV_Writer output objects (TAB delimited)
    utils::ChWriterCSV out_pos = OutStream();
    utils::ChWriterCSV out_vel = OutStream();
    utils::ChWriterCSV out_acc = OutStream();

    utils::ChWriterCSV out_quat = OutStream();
    utils::ChWriterCSV out_avel = OutStream();
    utils::ChWriterCSV out_aacc = OutStream();

    utils::ChWriterCSV out_rfrc = OutStream();
    utils::ChWriterCSV out_rtrq = OutStream();

    utils::ChWriterCSV out_energy = OutStream();

    utils::ChWriterCSV out_cnstr = OutStream();

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
              << "Constraint_4"
              << "Cnstr_5" << std::endl;

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

            // Reaction Force and Torque
            // These are expressed in the link coordinate system. We convert them to
            // the coordinate system of Body2 (in our case this is the ground).
            ChFrame<> linkCoordsys = cylindricalJoint->GetFrame2Rel();
            const auto& reaction = cylindricalJoint->GetReaction2();

            ChVector3d reactForce = reaction.force;
            ChVector3d reactForceGlobal = linkCoordsys.TransformDirectionLocalToParent(reactForce);
            out_rfrc << simTime << reactForceGlobal << std::endl;

            ChVector3d reactTorque = reaction.torque;
            ChVector3d reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
            out_rtrq << simTime << reactTorqueGlobal << std::endl;

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
            ChVectorDynamic<> C = cylindricalJoint->GetConstraintViolation();
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
bool ValidateReference(const std::string& chronoTestName,  // name of the Chrono test
                       const std::string& refTestName,     // name the reference test
                       const std::string& what,            // identifier for test quantity
                       double tolerance)                   // validation tolerance
{
    std::string sim_file = out_dir + chronoTestName + "_CHRONO_" + what + ".txt";
    std::string ref_file = ref_dir + refTestName + "_ADAMS_" + what + ".txt";
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
bool ValidateConstraints(const std::string& chronoTestName,  // name of the Chrono test
                         double tolerance)                   // validation tolerance
{
    std::string sim_file = out_dir + chronoTestName + "_CHRONO_Constraints.txt";
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
bool ValidateEnergy(const std::string& chronoTestName,  // name of the Chrono test
                    double tolerance)                   // validation tolerance
{
    std::string sim_file = out_dir + chronoTestName + "_CHRONO_Energy.txt";
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
utils::ChWriterCSV OutStream() {
    utils::ChWriterCSV out("\t");

    out.Stream().setf(std::ios::scientific | std::ios::showpos);
    out.Stream().precision(6);

    return out;
}
