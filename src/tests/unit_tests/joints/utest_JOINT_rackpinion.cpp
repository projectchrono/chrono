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
// Test for the rack and pinion joint
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

// =============================================================================
// Local variables
//
static const std::string val_dir = "../RESULTS/";
static const std::string out_dir = val_dir + "rackpinion_joint/";
static const std::string ref_dir = "testing/joints/rackpinion_joint/";

// =============================================================================
// Prototypes of local functions
//
bool TestRackPinion(const ChVector3d& jointLoc,
                    const ChQuaternion<>& jointRot,
                    double simTimeStep,
                    double outTimeStep,
                    const std::string& testName);
bool ValidateReference(const std::string& testName, const std::string& what, double tolerance);
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

    std::string test_name;
    bool test_passed = true;

    // Case 1 - Joint at the origin and aligned with the global Frame.
    // Pendulum Falls due to gravity.

    test_name = "RackPinion_Case01";
    TestRackPinion(ChVector3d(0, 0, 0), QUNIT, sim_step, out_step, test_name);
    // test_passed &= ValidateReference(test_name, "Pinion_Pos", 2e-3);
    // test_passed &= ValidateReference(test_name, "Pinion_Vel", 1e-4);
    // test_passed &= ValidateReference(test_name, "Pinion_Acc", 2e-2);
    // test_passed &= ValidateReference(test_name, "Pinion_Quat", 1e-3);
    // test_passed &= ValidateReference(test_name, "Pinion_Avel", 2e-2);
    // test_passed &= ValidateReference(test_name, "Pinion_Aacc", 2e-2);
    // test_passed &= ValidateReference(test_name, "Rack_Pos", 2e-3);
    // test_passed &= ValidateReference(test_name, "Rack_Vel", 1e-4);
    // test_passed &= ValidateReference(test_name, "Rack_Acc", 2e-2);
    // test_passed &= ValidateReference(test_name, "Rack_Quat", 1e-3);
    // test_passed &= ValidateReference(test_name, "Rack_Avel", 2e-2);
    // test_passed &= ValidateReference(test_name, "Rack_Aacc", 2e-2);
    // test_passed &= ValidateEnergy(test_name, 1e-2);

    // Return 0 if all tests passed and 1 otherwise
    std::cout << "\nUNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestRackPinion(const ChVector3d& jointLoc,      // absolute location of joint
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

    double massPinion = 1.0;                      // mass of pinion
    double radiusPinion = 0.1;                    // radius of pinion
    ChVector3d inertiaXX_Pinion(0.1, 0.1, 0.04);  // mass moments of inertia of pinion (centroidal frame)

    double massRack = 1.0;                      // mass of pendulum
    ChVector3d inertiaXX_Rack(0.1, 0.1, 0.04);  // mass moments of inertia of pendulum (centroidal frame)
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

    // Create the pinion body in an initial configuration at rest
    auto pinion = chrono_types::make_shared<ChBody>();
    sys.AddBody(pinion);
    pinion->SetPos(jointLoc);
    pinion->SetRot(QuatFromAngleY(CH_PI_2));
    pinion->SetMass(massPinion);
    pinion->SetInertiaXX(inertiaXX_Pinion);

    // Create the rack body in an initial configuration at rest
    auto rack = chrono_types::make_shared<ChBody>();
    sys.AddBody(rack);
    rack->SetPos(jointLoc + ChVector3d(0, radiusPinion, 0));
    rack->SetRot(QUNIT);
    rack->SetMass(massRack);
    rack->SetInertiaXX(inertiaXX_Rack);

    // Create revolute joint between pinion and ground at "loc" in the global
    // reference frame. The revolute joint's axis of rotation (Z) will be the
    // global x axis.
    auto revoluteJoint = chrono_types::make_shared<ChLinkLockRevolute>();
    revoluteJoint->Initialize(pinion, ground, ChFrame<>(jointLoc, QuatFromAngleY(CH_PI_2)));
    sys.AddLink(revoluteJoint);

    // Create prismatic joint between rack and ground at "loc" - Pinion Radius in the global
    // reference frame. The prismatic joint's axis of translation will be the Z axis
    // of the specified rotation matrix.
    auto prismaticJoint = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismaticJoint->Initialize(rack, ground, ChFrame<>(jointLoc + ChVector3d(0, -radiusPinion, 0), QUNIT));
    sys.AddLink(prismaticJoint);

    // Create the Rack and Pinion joint
    auto rackpinionJoint = chrono_types::make_shared<ChLinkMateRackPinion>();
    rackpinionJoint->Initialize(pinion, rack, false, ChFrame<>(jointLoc, QuatFromAngleY(-CH_PI_2)),
                                ChFrame<>(jointLoc + ChVector3d(0, 0, 0), QuatFromAngleY(-CH_PI_2)));
    rackpinionJoint->SetPinionRadius(-radiusPinion);
    rackpinionJoint->SetPressureAngle(CH_PI_4);
    rackpinionJoint->SetPitchAngle(0);
    rackpinionJoint->SetEnforcePhase(1);
    sys.AddLink(rackpinionJoint);

    // Perform the simulation (record results option)
    // ------------------------------------------------

    // Create the CSV_Writer output objects (TAB delimited)
    utils::ChWriterCSV out_posPinion = OutStream();
    utils::ChWriterCSV out_velPinion = OutStream();
    utils::ChWriterCSV out_accPinion = OutStream();

    utils::ChWriterCSV out_quatPinion = OutStream();
    utils::ChWriterCSV out_avelPinion = OutStream();
    utils::ChWriterCSV out_aaccPinion = OutStream();

    utils::ChWriterCSV out_posRack = OutStream();
    utils::ChWriterCSV out_velRack = OutStream();
    utils::ChWriterCSV out_accRack = OutStream();

    utils::ChWriterCSV out_quatRack = OutStream();
    utils::ChWriterCSV out_avelRack = OutStream();
    utils::ChWriterCSV out_aaccRack = OutStream();

    utils::ChWriterCSV out_energy = OutStream();

    // Write headers
    out_posPinion << "Time"
                  << "X_Pos"
                  << "Y_Pos"
                  << "Z_Pos" << std::endl;
    out_velPinion << "Time"
                  << "X_Vel"
                  << "Y_Vel"
                  << "Z_Vel" << std::endl;
    out_accPinion << "Time"
                  << "X_Acc"
                  << "Y_Acc"
                  << "Z_Acc" << std::endl;

    out_quatPinion << "Time"
                   << "e0"
                   << "e1"
                   << "e2"
                   << "e3" << std::endl;
    out_avelPinion << "Time"
                   << "X_AngVel"
                   << "Y_AngVel"
                   << "Z_AngVel" << std::endl;
    out_aaccPinion << "Time"
                   << "X_AngAcc"
                   << "Y_AngAcc"
                   << "Z_AngAcc" << std::endl;

    out_posRack << "Time"
                << "X_Pos"
                << "Y_Pos"
                << "Z_Pos" << std::endl;
    out_velRack << "Time"
                << "X_Vel"
                << "Y_Vel"
                << "Z_Vel" << std::endl;
    out_accRack << "Time"
                << "X_Acc"
                << "Y_Acc"
                << "Z_Acc" << std::endl;

    out_quatRack << "Time"
                 << "e0"
                 << "e1"
                 << "e2"
                 << "e3" << std::endl;
    out_avelRack << "Time"
                 << "X_AngVel"
                 << "Y_AngVel"
                 << "Z_AngVel" << std::endl;
    out_aaccRack << "Time"
                 << "X_AngAcc"
                 << "Y_AngAcc"
                 << "Z_AngAcc" << std::endl;

    out_energy << "Time"
               << "Transl_KE"
               << "Rot_KE"
               << "Delta_PE"
               << "KE+PE" << std::endl;

    // Perform a system assembly to ensure we have the correct accelerations at the initial time.
    sys.DoAssembly(AssemblyAnalysis::Level::FULL);

    // Total energy at initial time.
    ChMatrix33<> inertiaPinion = pinion->GetInertia();
    ChVector3d angVelLocPinion = pinion->GetAngVelLocal();
    ChMatrix33<> inertiaRack = rack->GetInertia();
    ChVector3d angVelLocRack = rack->GetAngVelLocal();
    double transKE = 0.5 * massPinion * pinion->GetPosDt().Length2() + 0.5 * massRack * rack->GetPosDt().Length2();
    double rotKE = 0.5 * Vdot(angVelLocPinion, inertiaPinion * angVelLocPinion) +
                   0.5 * Vdot(angVelLocRack, inertiaRack * angVelLocRack);
    double deltaPE =
        massPinion * g * (pinion->GetPos().z() - jointLoc.z()) + massRack * g * (rack->GetPos().z() - jointLoc.z());
    double totalE0 = transKE + rotKE + deltaPE;

    // Simulation loop
    double simTime = 0;
    double outTime = 0;

    while (simTime <= timeRecord + simTimeStep / 2) {
        // Ensure that the final data point is recorded.
        if (simTime >= outTime - simTimeStep / 2) {
            // CM position, velocity, and acceleration (expressed in global frame).
            const ChVector3d& positionPinion = pinion->GetPos();
            const ChVector3d& velocityPinion = pinion->GetPosDt();
            const ChVector3d& positionRack = rack->GetPos();
            const ChVector3d& velocityRack = rack->GetPosDt();
            out_posPinion << simTime << positionPinion << std::endl;
            out_velPinion << simTime << velocityPinion << std::endl;
            out_accPinion << simTime << pinion->GetPosDt2() << std::endl;

            out_posRack << simTime << positionRack << std::endl;
            out_velRack << simTime << velocityRack << std::endl;
            out_accRack << simTime << rack->GetPosDt2() << std::endl;

            // Orientation, angular velocity, and angular acceleration (expressed in
            // global frame).
            out_quatPinion << simTime << pinion->GetRot() << std::endl;
            out_avelPinion << simTime << pinion->GetAngVelParent() << std::endl;
            out_aaccPinion << simTime << pinion->GetAngAccParent() << std::endl;

            out_quatRack << simTime << rack->GetRot() << std::endl;
            out_avelRack << simTime << rack->GetAngVelParent() << std::endl;
            out_aaccRack << simTime << rack->GetAngAccParent() << std::endl;

            // Conservation of Energy
            // Translational Kinetic Energy (1/2*m*||v||^2)
            // Rotational Kinetic Energy (1/2 w'*I*w)
            // Delta Potential Energy (m*g*dz)
            angVelLocPinion = pinion->GetAngVelLocal();
            angVelLocRack = rack->GetAngVelLocal();
            transKE = 0.5 * massPinion * pinion->GetPosDt().Length2() + 0.5 * massRack * rack->GetPosDt().Length2();
            rotKE = 0.5 * Vdot(angVelLocPinion, inertiaPinion * angVelLocPinion) +
                    0.5 * Vdot(angVelLocRack, inertiaRack * angVelLocRack);
            deltaPE = massPinion * g * (pinion->GetPos().z() - jointLoc.z()) +
                      massRack * g * (rack->GetPos().z() - jointLoc.z());
            double totalE = transKE + rotKE + deltaPE;
            out_energy << simTime << transKE << rotKE << deltaPE << totalE - totalE0 << std::endl;

            // Increment output time
            outTime += outTimeStep;
        }

        // Advance simulation by one step
        sys.DoStepDynamics(simTimeStep);

        // Increment simulation time
        simTime += simTimeStep;
    }

    // Write output files
    out_posPinion.WriteToFile(out_dir + testName + "_CHRONO_Pinion_Pos.txt", testName + "\n");
    out_velPinion.WriteToFile(out_dir + testName + "_CHRONO_Pinion_Vel.txt", testName + "\n");
    out_accPinion.WriteToFile(out_dir + testName + "_CHRONO_Pinion_Acc.txt", testName + "\n");

    out_posRack.WriteToFile(out_dir + testName + "_CHRONO_Rack_Pos.txt", testName + "\n");
    out_velRack.WriteToFile(out_dir + testName + "_CHRONO_Rack_Vel.txt", testName + "\n");
    out_accRack.WriteToFile(out_dir + testName + "_CHRONO_Rack_Acc.txt", testName + "\n");

    out_quatPinion.WriteToFile(out_dir + testName + "_CHRONO_Pinion_Quat.txt", testName + "\n");
    out_avelPinion.WriteToFile(out_dir + testName + "_CHRONO_Pinion_Avel.txt", testName + "\n");
    out_aaccPinion.WriteToFile(out_dir + testName + "_CHRONO_Pinion_Aacc.txt", testName + "\n");

    out_quatRack.WriteToFile(out_dir + testName + "_CHRONO_Rack_Quat.txt", testName + "\n");
    out_avelRack.WriteToFile(out_dir + testName + "_CHRONO_Rack_Avel.txt", testName + "\n");
    out_aaccRack.WriteToFile(out_dir + testName + "_CHRONO_Rack_Aacc.txt", testName + "\n");

    out_energy.WriteToFile(out_dir + testName + "_CHRONO_Energy.txt", testName + "\n");

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
utils::ChWriterCSV OutStream() {
    utils::ChWriterCSV out("\t");

    out.Stream().setf(std::ios::scientific | std::ios::showpos);
    out.Stream().precision(6);

    return out;
}
