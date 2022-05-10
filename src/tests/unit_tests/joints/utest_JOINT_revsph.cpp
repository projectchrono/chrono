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
#include "chrono/utils/ChUtilsInputOutput.h"
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
bool TestRevSpherical(const ChVector<>& jointLocGnd,
                      const ChVector<>& jointRevAxis,
                      const ChVector<>& jointLocPend,
                      const ChCoordsys<>& PendCSYS,
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
    double sim_step = 1e-5;
    double out_step = 1e-2;

    std::string test_name;
    bool test_passed = true;

    // Case 1 - Revolute connect to ground at (0,0,0) and aligned with the global z axis
    //   Spherical joint at one end of a horizontal pendulum (2,0,0) with CG at (2,2,0)

    test_name = "RevSpherical_Case01";
    TestRevSpherical(ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), ChVector<>(2, 0, 0),
                     ChCoordsys<>(ChVector<>(2, 2, 0), QUNIT), sim_step, out_step, test_name);
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
    TestRevSpherical(ChVector<>(1, 2, 3), ChVector<>(0, 1, 1), ChVector<>(3, 2, 3),
                     ChCoordsys<>(ChVector<>(3, 4, 3), QUNIT), sim_step, out_step, test_name);
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
    std::cout << std::endl << "UNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestRevSpherical(
    const ChVector<>& jointLocGnd,   // absolute location of the RevSpherical constrain ground attachment point
    const ChVector<>& jointRevAxis,  // absolute vector for the revolute axis of the RevSpherical constrain
    const ChVector<>& jointLocPend,  // absolute location of the distance constrain pendulum attachment point
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
    ChVector<> inertiaXX(0.1, 0.04, 0.1);  // mass moments of inertia of pendulum (centroidal frame)
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
    utils::CSV_writer out_pos = OutStream();
    utils::CSV_writer out_vel = OutStream();
    utils::CSV_writer out_acc = OutStream();

    utils::CSV_writer out_quat = OutStream();
    utils::CSV_writer out_avel = OutStream();
    utils::CSV_writer out_aacc = OutStream();

    utils::CSV_writer out_rfrc = OutStream();
    utils::CSV_writer out_rtrq = OutStream();
    utils::CSV_writer out_rfrc1 = OutStream();
    utils::CSV_writer out_rtrq1 = OutStream();
    utils::CSV_writer out_rfrc2 = OutStream();
    utils::CSV_writer out_rtrq2 = OutStream();

    utils::CSV_writer out_energy = OutStream();

    utils::CSV_writer out_cnstr = OutStream();

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

    // timeRecord = .0001;
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

            // Chrono returns the reaction force and torque on body 2 (as specified in
            // the joint Initialize() function), as applied at the joint location and
            // expressed in the joint frame. Here, the 2nd body is the pendulum.

            //    joint frame on 2nd body (pendulum), expressed in the body frame
            ChCoordsys<> linkCoordsys = revSphericalConstraint->GetLinkRelativeCoords();

            //    reaction force and torque on pendulum, expressed in joint frame
            //       at the joint frame origin (center of the revolute)
            ChVector<> reactForce = revSphericalConstraint->Get_react_force();
            ChVector<> reactTorque = revSphericalConstraint->Get_react_torque();

            //    reaction force and torque on the ground, expressed in joint frame
            //       at the revolute joint center (joint frame origin)
            ChVector<> reactForceB1 = revSphericalConstraint->Get_react_force_body1();
            ChVector<> reactTorqueB1 = revSphericalConstraint->Get_react_torque_body1();

            //    reaction force and torque on the ground, expressed in joint frame
            //       at the spherical joint center
            ChVector<> reactForceB2 = revSphericalConstraint->Get_react_force_body2();
            ChVector<> reactTorqueB2 = revSphericalConstraint->Get_react_torque_body2();

            //    Transform from the joint frame into the pendulum frame
            reactForce = linkCoordsys.TransformDirectionLocalToParent(reactForce);
            reactTorque = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
            reactForceB1 = linkCoordsys.TransformDirectionLocalToParent(reactForceB1);
            reactTorqueB1 = linkCoordsys.TransformDirectionLocalToParent(reactTorqueB1);
            reactForceB2 = linkCoordsys.TransformDirectionLocalToParent(reactForceB2);
            reactTorqueB2 = linkCoordsys.TransformDirectionLocalToParent(reactTorqueB2);

            //    Transform from the joint frame into the global frame
            reactForce = pendulum->TransformDirectionLocalToParent(reactForce);
            reactTorque = pendulum->TransformDirectionLocalToParent(reactTorque);
            reactForceB1 = pendulum->TransformDirectionLocalToParent(reactForceB1);
            reactTorqueB1 = pendulum->TransformDirectionLocalToParent(reactTorqueB1);
            reactForceB2 = pendulum->TransformDirectionLocalToParent(reactForceB2);
            reactTorqueB2 = pendulum->TransformDirectionLocalToParent(reactTorqueB2);

            out_rfrc << simTime << reactForce << std::endl;
            out_rtrq << simTime << reactTorque << std::endl;
            out_rfrc1 << simTime << reactForceB1 << std::endl;
            out_rtrq1 << simTime << reactTorqueB1 << std::endl;
            out_rfrc2 << simTime << reactForceB2 << std::endl;
            out_rtrq2 << simTime << reactTorqueB2 << std::endl;

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
    out_pos.write_to_file(out_dir + testName + "_CHRONO_Pos.txt", testName + "\n\n");
    out_vel.write_to_file(out_dir + testName + "_CHRONO_Vel.txt", testName + "\n\n");
    out_acc.write_to_file(out_dir + testName + "_CHRONO_Acc.txt", testName + "\n\n");

    out_quat.write_to_file(out_dir + testName + "_CHRONO_Quat.txt", testName + "\n\n");
    out_avel.write_to_file(out_dir + testName + "_CHRONO_Avel.txt", testName + "\n\n");
    out_aacc.write_to_file(out_dir + testName + "_CHRONO_Aacc.txt", testName + "\n\n");

    out_rfrc.write_to_file(out_dir + testName + "_CHRONO_Rforce.txt", testName + "\n\n");
    out_rtrq.write_to_file(out_dir + testName + "_CHRONO_Rtorque.txt", testName + "\n\n");
    out_rfrc1.write_to_file(out_dir + testName + "_CHRONO_Rforce_Body1.txt", testName + "\n\n");
    out_rtrq1.write_to_file(out_dir + testName + "_CHRONO_Rtorque_Body1.txt", testName + "\n\n");
    out_rfrc2.write_to_file(out_dir + testName + "_CHRONO_Rforce_Body2.txt", testName + "\n\n");
    out_rtrq2.write_to_file(out_dir + testName + "_CHRONO_Rtorque_Body2.txt", testName + "\n\n");

    out_energy.write_to_file(out_dir + testName + "_CHRONO_Energy.txt", testName + "\n\n");

    out_cnstr.write_to_file(out_dir + testName + "_CHRONO_Constraints.txt", testName + "\n\n");

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
utils::CSV_writer OutStream() {
    utils::CSV_writer out("\t");

    out.stream().setf(std::ios::scientific | std::ios::showpos);
    out.stream().precision(6);

    return out;
}
