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

#include <cmath>
#include <fstream>
#include <ostream>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// =============================================================================
// Local variables
//
static const std::string val_dir = "../RESULTS/";
static const std::string out_dir = val_dir + "transpringcb_force/";
static const std::string ref_dir = "testing/joints/transpringcb_force/";

// =============================================================================

// Functor classes implementing the force for a ChLinkTSDA link.
class MySpringForceCase01 : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double rest_length,     // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
    ) {
        double spring_coef = 10;
        double damping_coef = .5;

        double force = -spring_coef * (length - rest_length) - damping_coef * vel;
        return force;
    }
};

class MySpringForceCase02 : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double rest_length,     // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
    ) {
        double spring_coef = 100;
        double damping_coef = 5;

        double force = -spring_coef * (length - rest_length) - damping_coef * vel;
        return force;
    }
};

class MySpringForceCase03 : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double rest_length,     // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
    ) {
        double spring_coef = 50;
        double spring_nonlin_coef = 10;
        double damping_coef = 5;

        double force = -spring_coef * (length - rest_length) -
                       spring_nonlin_coef * fabs(length - rest_length) * (length - rest_length) - damping_coef * vel;
        return force;
    }
};

// =============================================================================
// Prototypes of local functions
//
bool TestTranSpringCB(const ChVector3d& jointLocGnd,
                      const ChVector3d& jointLocPend,
                      const ChCoordsys<>& PendCSYS,
                      const int customSpringType,
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
    double sim_step = 1e-5;
    double out_step = 1e-2;

    std::string test_name;
    bool test_passed = true;

    // Case 1 - Pendulum dropped from the origin with a spring connected between
    // the CG and ground.

    test_name = "TranSpringCB_Case01";
    TestTranSpringCB(ChVector3d(0, 0, 0), ChVector3d(0, 0, 0), ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 1, sim_step,
                     out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-4);
    test_passed &= ValidateReference(test_name, "Vel", 5e-5);
    test_passed &= ValidateReference(test_name, "Acc", 5e-4);
    test_passed &= ValidateReference(test_name, "Quat", 1e-10);
    test_passed &= ValidateReference(test_name, "Avel", 1e-10);
    test_passed &= ValidateReference(test_name, "Aacc", 1e-10);
    test_passed &= ValidateReference(test_name, "Rforce", 5e-4);

    // Case 2 - Pendulum CG at Y = 2 with a linear spring between the CG and
    // ground.

    test_name = "TranSpringCB_Case02";
    TestTranSpringCB(ChVector3d(0, 0, 0), ChVector3d(0, 2, 0), ChCoordsys<>(ChVector3d(0, 2, 0), QUNIT), 2, sim_step,
                     out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-4);
    test_passed &= ValidateReference(test_name, "Vel", 5e-5);
    test_passed &= ValidateReference(test_name, "Acc", 5e-4);
    test_passed &= ValidateReference(test_name, "Quat", 1e-10);
    test_passed &= ValidateReference(test_name, "Avel", 1e-10);
    test_passed &= ValidateReference(test_name, "Aacc", 1e-10);
    test_passed &= ValidateReference(test_name, "Rforce", 5e-4);

    // Case 3 - Pendulum inital position is perpendicular to the non-linear spring
    // between ground

    test_name = "TranSpringCB_Case03";
    TestTranSpringCB(ChVector3d(1, 2, 3), ChVector3d(1, 4, 3), ChCoordsys<>(ChVector3d(-1, 4, 3), QUNIT), 3, sim_step,
                     out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-4);
    test_passed &= ValidateReference(test_name, "Vel", 5e-4);
    test_passed &= ValidateReference(test_name, "Acc", 1e-3);
    test_passed &= ValidateReference(test_name, "Quat", 5e-5);
    test_passed &= ValidateReference(test_name, "Avel", 5e-3);
    test_passed &= ValidateReference(test_name, "Aacc", 5e-2);
    test_passed &= ValidateReference(test_name, "Rforce", 5e-3);

    // Case 4 - Pendulum inital position is streched out along the Y axis with the
    // non-linear spring on the end of the pendulum to ground (Double Pendulum).

    test_name = "TranSpringCB_Case04";
    TestTranSpringCB(ChVector3d(0, 0, 0), ChVector3d(0, 2, 0),
                     ChCoordsys<>(ChVector3d(0, 4, 0), QuatFromAngleZ(-CH_PI_2)), 3, sim_step, out_step, test_name);
    test_passed &= ValidateReference(test_name, "Pos", 1e-4);
    test_passed &= ValidateReference(test_name, "Vel", 5e-4);
    test_passed &= ValidateReference(test_name, "Acc", 1e-3);
    test_passed &= ValidateReference(test_name, "Quat", 5e-5);
    test_passed &= ValidateReference(test_name, "Avel", 5e-3);
    test_passed &= ValidateReference(test_name, "Aacc", 5e-2);
    test_passed &= ValidateReference(test_name, "Rforce", 5e-3);

    // Return 0 if all tests passed and 1 otherwise
    std::cout << "\nUNIT TEST: " << (test_passed ? "PASSED" : "FAILED") << std::endl;
    return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestTranSpringCB(const ChVector3d& jointLocGnd,   // absolute location of the distance
                                                       // constrain ground attachment point
                      const ChVector3d& jointLocPend,  // absolute location of the distance
                                                       // constrain pendulum attachment point
                      const ChCoordsys<>& PendCSYS,    // Coordinate system for the pendulum
                      const int customSpringType,      // Flag for Custom spring force Callback
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
    pendulum->SetPos(PendCSYS.pos);
    pendulum->SetRot(PendCSYS.rot);
    pendulum->SetMass(mass);
    pendulum->SetInertiaXX(inertiaXX);

    // Create a translational spring damper force between pendulum at "jointLocPend" and ground at
    // "jointLocGnd" in the global reference frame.
    // The free length is set equal to the initial distance between "jointLocPend" and "jointLocGnd".

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(pendulum, ground, false, jointLocPend, jointLocGnd);
    if (customSpringType == 1) {
        auto force = chrono_types::make_shared<MySpringForceCase01>();
        spring->RegisterForceFunctor(force);
    } else if (customSpringType == 2) {
        auto force = chrono_types::make_shared<MySpringForceCase02>();
        spring->RegisterForceFunctor(force);
    } else if (customSpringType == 3) {
        auto force = chrono_types::make_shared<MySpringForceCase03>();
        spring->RegisterForceFunctor(force);
    }
    sys.AddLink(spring);

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

            // Spring Force and Torque
            // Force is given as a scalar and is then transformed into a vector in
            // global coordiantes
            // Torques are expressed in the link coordinate system. We convert them to
            // the coordinate system of Body2 (in our case this is the ground).
            ChVector3d springVector = spring->GetPoint2Abs() - spring->GetPoint1Abs();
            springVector.Normalize();
            double springForce = spring->GetForce();
            ChVector3d springForceGlobal = springForce * springVector;
            out_rfrc << simTime << springForceGlobal << std::endl;

            ChFrame<> linkCoordsys = spring->GetFrame2Rel();
            ChVector3d reactTorque = spring->GetReaction2().torque;
            ChVector3d reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
            out_rtrq << simTime << reactTorqueGlobal << std::endl;

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
utils::ChWriterCSV OutStream() {
    utils::ChWriterCSV out("\t");

    out.Stream().setf(std::ios::scientific | std::ios::showpos);
    out.Stream().precision(6);

    return out;
}
