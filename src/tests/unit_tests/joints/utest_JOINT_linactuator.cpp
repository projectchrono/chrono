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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Test for linear actuator
//
// =============================================================================

#include <ostream>
#include <fstream>

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
static const std::string out_dir = val_dir + "lin_actuator/";
static const std::string ref_dir = "testing/joints/lin_actuator/";

// =============================================================================
// Prototypes of local functions
//
bool TestLinActuator(const ChQuaternion<>& rot,
                     double desiredSpeed,
                     double simTimeStep,
                     double outTimeStep,
                     const std::string& testName);
bool ValidateReference(const std::string& testName, const std::string& what, double tolerance);
bool ValidateConstraints(const std::string& testName, const std::string& what, double tolerance);
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
    double sim_step = 1e-3;
    double out_step = 1e-2;

    std::string test_name;
    bool test_passed = true;

    // Case 1 - Translation axis vertical, imposed speed 1 m/s

    test_name = "LinActuator_Case01";
    TestLinActuator(QUNIT, 1, sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    test_passed &= ValidateReference(test_name, "Vel", 1e-3);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(test_name, "RforceP", 2e-2);
    test_passed &= ValidateReference(test_name, "RtorqueP", 1e-10);
    test_passed &= ValidateReference(test_name, "RforceA", 5e-1);
    test_passed &= ValidateReference(test_name, "RtorqueA", 1e-10);
    test_passed &= ValidateConstraints(test_name, "ConstraintsP", 1e-5);
    test_passed &= ValidateConstraints(test_name, "ConstraintsA", 1e-5);

    // Case 2 - Translation axis along X = Z, imposed speed 0.5 m/s

    test_name = "LinActuator_Case02";
    TestLinActuator(QuatFromAngleY(CH_PI / 4), 0.5, sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    test_passed &= ValidateReference(test_name, "Vel", 1e-3);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(test_name, "RforceP", 3e-1);
    test_passed &= ValidateReference(test_name, "RtorqueP", 5e-3);
    test_passed &= ValidateReference(test_name, "RforceA", 5e-1);
    test_passed &= ValidateReference(test_name, "RtorqueA", 1e-10);
    test_passed &= ValidateConstraints(test_name, "ConstraintsP", 1e-5);
    test_passed &= ValidateConstraints(test_name, "ConstraintsA", 1e-5);

    // Return 0 if all tests passed and 1 otherwise
    std::cout << "\nUNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestLinActuator(const ChQuaternion<>& rot,    // translation along Z axis
                     double desiredSpeed,          // imposed translation speed
                     double simTimeStep,           // simulation time step
                     double outTimeStep,           // output time step
                     const std::string& testName)  // if true, animate with Irrlich
{
    std::cout << "TEST: " << testName << std::endl;

    // Unit vector along translation axis, expressed in global frame
    ChVector3d axis = rot.GetAxisZ();

    // Settings
    //---------

    double mass = 1.0;              // mass of plate
    ChVector3d inertiaXX(1, 1, 1);  // mass moments of inertia of plate (centroidal frame)
    double g = 9.80665;

    double timeRecord = 5;  // Stop recording to the file after this much simulated time

    // Create the mechanical system
    // ----------------------------

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -g));

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(300);
    sys.GetSolver()->AsIterative()->SetTolerance(simTimeStep * 1e-4);

    // Create the ground body.

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);

    // Create the plate body.

    auto plate = chrono_types::make_shared<ChBody>();
    sys.AddBody(plate);
    plate->SetPos(ChVector3d(0, 0, 0));
    plate->SetRot(rot);
    plate->SetPosDt(desiredSpeed * axis);
    plate->SetMass(mass);
    plate->SetInertiaXX(inertiaXX);

    // Create prismatic (translational) joint between plate and ground.
    // We set the ground as the "master" body (second one in the initialization
    // call) so that the link coordinate system is expressed in the ground frame.

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(plate, ground, ChFrame<>(ChVector3d(0, 0, 0), rot));
    sys.AddLink(prismatic);

    // Create a ramp function to impose constant speed.  This function returns
    //   y(t) = 0 + t * desiredSpeed
    //   y'(t) = desiredSpeed

    auto actuator_fun = chrono_types::make_shared<ChFunctionRamp>(0.0, desiredSpeed);

    // Create the linear actuator, connecting the plate to the ground.
    // Here, we set the plate as the master body (second one in the initialization
    // call) so that the link coordinate system is expressed in the plate body
    // frame.

    auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
    ChVector3d pt1 = ChVector3d(0, 0, 0);
    ChVector3d pt2 = axis;
    actuator->Initialize(ground, plate, false, ChFrame<>(pt1, rot), ChFrame<>(pt2, rot));
    actuator->SetDistanceOffset(1);
    actuator->SetActuatorFunction(actuator_fun);
    sys.AddLink(actuator);

    // Perform the simulation (record results)
    // ------------------------------------------------

    // Create the CSV_Writer output objects (TAB delimited)
    utils::ChWriterCSV out_pos = OutStream();
    utils::ChWriterCSV out_vel = OutStream();
    utils::ChWriterCSV out_acc = OutStream();

    utils::ChWriterCSV out_quat = OutStream();
    utils::ChWriterCSV out_avel = OutStream();
    utils::ChWriterCSV out_aacc = OutStream();

    utils::ChWriterCSV out_rfrcP = OutStream();
    utils::ChWriterCSV out_rtrqP = OutStream();

    utils::ChWriterCSV out_rfrcA = OutStream();
    utils::ChWriterCSV out_rtrqA = OutStream();

    utils::ChWriterCSV out_cnstrP = OutStream();

    utils::ChWriterCSV out_cnstrA = OutStream();

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

    out_rfrcP << "Time"
              << "X_Force"
              << "Y_Force"
              << "Z_Force" << std::endl;
    out_rtrqP << "Time"
              << "X_Torque"
              << "Y_Torque"
              << "Z_Torque" << std::endl;

    out_rfrcA << "Time"
              << "X_Force"
              << "Y_Force"
              << "Z_Force" << std::endl;
    out_rtrqA << "Time"
              << "X_Torque"
              << "Y_Torque"
              << "Z_Torque" << std::endl;

    out_cnstrP << "Time"
               << "Cnstr_1"
               << "Cnstr_2"
               << "Cnstr_3"
               << "Constraint_4"
               << "Cnstr_5" << std::endl;

    out_cnstrA << "Time"
               << "Cnstr_1" << std::endl;

    // Simulation loop
    double simTime = 0;
    double outTime = 0;

    while (simTime <= timeRecord + simTimeStep / 2) {
        // Ensure that the final data point is recorded.
        if (simTime >= outTime - simTimeStep / 2) {
            // CM position, velocity, and acceleration (expressed in global frame).
            const ChVector3d& position = plate->GetPos();
            const ChVector3d& velocity = plate->GetPosDt();
            out_pos << simTime << position << std::endl;
            out_vel << simTime << velocity << std::endl;
            out_acc << simTime << plate->GetPosDt2() << std::endl;

            // Orientation, angular velocity, and angular acceleration (expressed in
            // global frame).
            out_quat << simTime << plate->GetRot() << std::endl;
            out_avel << simTime << plate->GetAngVelParent() << std::endl;
            out_aacc << simTime << plate->GetAngAccParent() << std::endl;

            // Reaction Force and Torque in prismatic joint.
            // These are expressed in the link coordinate system. We convert them to
            // the coordinate system of Body2 (in our case this is the ground).
            ChFrame<> linkCoordsysP = prismatic->GetFrame2Rel();
            const auto& reactionP = prismatic->GetReaction2();

            ChVector3d reactForceP = reactionP.force;
            ChVector3d reactForceGlobalP = linkCoordsysP.TransformDirectionLocalToParent(reactForceP);
            out_rfrcP << simTime << reactForceGlobalP << std::endl;

            ChVector3d reactTorqueP = reactionP.torque;
            ChVector3d reactTorqueGlobalP = linkCoordsysP.TransformDirectionLocalToParent(reactTorqueP);
            out_rtrqP << simTime << reactTorqueGlobalP << std::endl;

            // Reaction force and Torque in linear actuator.
            // These are expressed  in the link coordinate system. We convert them to
            // the coordinate system of Body2 (in our case this is the plate). As such,
            // the reaction force represents the force that needs to be applied to the
            // plate in order to maintain the prescribed constant velocity.  These are
            // then converted to the global frame for comparison to ADAMS
            ChFrame<> linkCoordsysA = actuator->GetFrame2Rel();
            const auto& reactionA = actuator->GetReaction2();

            ChVector3d reactForceA = reactionA.force;
            reactForceA = linkCoordsysA.TransformDirectionLocalToParent(reactForceA);
            ChVector3d reactForceGlobalA = plate->TransformDirectionLocalToParent(reactForceA);
            out_rfrcA << simTime << reactForceGlobalA << std::endl;

            ChVector3d reactTorqueA = reactionA.torque;
            reactTorqueA = linkCoordsysA.TransformDirectionLocalToParent(reactTorqueA);
            ChVector3d reactTorqueGlobalA = plate->TransformDirectionLocalToParent(reactTorqueA);
            out_rtrqA << simTime << reactTorqueGlobalA << std::endl;

            // Constraint violations in prismatic joint
            ChVectorDynamic<> CP = prismatic->GetConstraintViolation();
            out_cnstrP << simTime << CP(0) << CP(1) << CP(2) << CP(3) << CP(4) << std::endl;

            // Constraint violations in linear actuator
            ChVectorDynamic<> CA = actuator->GetConstraintViolation();
            out_cnstrA << simTime << CA(0) << std::endl;

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

    out_rfrcP.WriteToFile(out_dir + testName + "_CHRONO_RforceP.txt", testName + "\n");
    out_rtrqP.WriteToFile(out_dir + testName + "_CHRONO_RtorqueP.txt", testName + "\n");

    out_rfrcA.WriteToFile(out_dir + testName + "_CHRONO_RforceA.txt", testName + "\n");
    out_rtrqA.WriteToFile(out_dir + testName + "_CHRONO_RtorqueA.txt", testName + "\n");

    out_cnstrP.WriteToFile(out_dir + testName + "_CHRONO_ConstraintsP.txt", testName + "\n");

    out_cnstrA.WriteToFile(out_dir + testName + "_CHRONO_ConstraintsA.txt", testName + "\n");

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
                         const std::string& what,      // identifier for test quantity
                         double tolerance)             // validation tolerance
{
    std::string sim_file = out_dir + testName + "_CHRONO_" + what + ".txt";
    utils::DataVector norms;

    bool check = utils::Validate(sim_file, utils::RMS_NORM, tolerance, norms);
    std::cout << "   validate Constraints" << (check ? ": Passed" : ": Failed") << "  [  ";
    for (size_t col = 0; col < norms.size(); col++)
        std::cout << norms[col] << "  ";
    std::cout << "  ]" << std::endl;

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
