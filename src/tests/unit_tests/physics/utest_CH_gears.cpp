// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include <iostream>

using namespace chrono;


TEST(ChLinkLockGear, cylindrical) {
    
    // Reference data:

    double radA = 4;
    double radB = 2;
    double torque = 1000;
    double alpha = 20 * CH_DEG_TO_RAD;  // pressure angle, normal
    double beta = 10 * CH_DEG_TO_RAD;   // helix angle

    double teeth_K = 1e6;         // [N/m] teeth equivalent stiffness
    double teeth_D_coeff = 0.01;  // stiffness-proportional damping, as beta damping in Rayleigh damping

    // double alpha_tangent = atan(tan(alpha) / cos(beta));  // effective pressure angle in tangent plane
    double teeth_K_tangent = teeth_K * (cos(alpha) * cos(alpha)) * cos(beta) * cos(beta);

    // Expected theoretical results, from analytical solutions for helical gears
    double P_t = torque / radB;                   // tangent component
    double P_r = P_t * (tan(alpha) / cos(beta));  // radial component
    double P_a = P_t * tan(beta);                 // axial component

    double expected_deform_B = (torque / radB) / (teeth_K_tangent);
    double expected_angle_B = (expected_deform_B / radB) * CH_RAD_TO_DEG;  

    std::cout << "THEORY, expected angular slip because of compliance: " << expected_angle_B << " [deg] \n";
    std::cout << "THEORY, expected tangent deformation of teeth: " << expected_deform_B << " [m] \n";
    std::cout << "THEORY, theoretical contact force:   P_t=" << P_t << "  P_r=" << P_r << "  P_a=" << P_a
              << "     P_n=" << sqrt(P_t * P_t + P_r * P_r + P_a * P_a) << std::endl;



    // Create a Chrono physical system
    ChSystemNSC sys;

    // Contact material shared among all bodies
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create all the rigid bodies.
    // 
    // ...the truss
    auto mbody_truss = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
    sys.Add(mbody_truss);
    mbody_truss->SetFixed(true);
    mbody_truss->SetPos(ChVector3d(0, 0, 3));

    // ...the first gear
    auto mbody_gearA = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, radA, 0.2, 1000, true, false, mat);
    sys.Add(mbody_gearA);
    mbody_gearA->SetPos(ChVector3d(0, 0, -1));

    // ...the second gear bearing: motor that imposes (zero) rotation speed
    auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motor->Initialize(mbody_gearA, mbody_truss, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    link_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    sys.AddLink(link_motor);

    // ...the second gear
    double interaxis12 = radA + radB;
    auto mbody_gearB = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, radB, 0.2, 1000, true, false, mat);
    sys.Add(mbody_gearB);
    mbody_gearB->SetPos(ChVector3d(interaxis12, 0, -1));

    // ... the second gear bearing
    auto link_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revolute->Initialize(mbody_gearB, mbody_truss, ChFrame<>(ChVector3d(interaxis12, 0, 0), QUNIT));
    sys.AddLink(link_revolute);

    // ...the gear constraint between the two wheels A and B.
    auto link_gearAB = chrono_types::make_shared<ChLinkLockGear>();
    link_gearAB->Initialize(mbody_gearA, mbody_gearB, ChFrame<>());
    link_gearAB->SetFrameShaft1(ChFrame<>(VNULL));
    link_gearAB->SetFrameShaft2(ChFrame<>(VNULL));
    link_gearAB->SetTransmissionRatio(radA / radB);  // note: always positive, use SetEpicyclic() for inner&outer
    link_gearAB->SetEnforcePhase(true);              // needed if compliant gear is used
    link_gearAB->SetPressureAngle(alpha);
    link_gearAB->SetPitchAngle(beta);
    sys.AddLink(link_gearAB);

    sys.SetGravitationalAcceleration(VNULL);

    //
    // TEST 1 : apply torque to wheel 2 and see if reactions on bearings are correct
    //

    // ...apply torque to second gear
    auto mtorque = chrono_types::make_shared<ChForce>();
    mtorque->SetMode(ChForce::TORQUE);
    mtorque->SetBody(mbody_gearB.get());
    mtorque->SetDir(ChVector3d(0, 0, 1));
    mtorque->SetMforce(torque);
    mbody_gearB->AddForce(mtorque);

    // Simulate few steps
    double timestep = 0.001;
    while (sys.GetChTime() < 0.02) {
        // Advance simulation by one step
        sys.DoStepDynamics(timestep);

        // std::cout << "test1 time: " << sys.GetChTime() << "  angle=" << mbody_gearB->GetRotAngle() * CH_RAD_TO_DEG <<
        // "\n";
    }
    std::cout << "TEST 1, rigid ChLinkLockGear: F_n=" << link_gearAB->GetContactForce() << std::endl;



    /*
    std::cout << "Theoretical contact force:   P_t=" << P_t << "  P_r=" << P_r << "  P_a=" << P_a
              << "     P=" << sqrt(P_t * P_t + P_r * P_r + P_a * P_a) << std::endl;
    std::cout << "Reaction on bearing 1: " << link_motor->GetReaction2().force << std::endl;
    std::cout << "Reaction on bearing 2: " << link_revolute->GetReaction2().force << std::endl;
    std::cout << "Contact force: " << link_gearAB->GetReaction2().force.x() << std::endl;
    */

    // check equilibrium in radial dir:
    ASSERT_TRUE(std::fabs(link_revolute->GetReaction2().force.x() - P_r) < 1e-4);
    // check equilibrium in axial dir:
    ASSERT_TRUE(std::fabs(link_revolute->GetReaction2().force.z() - P_a) < 1e-4);
    // check equilibrium in tangent dir:
    ASSERT_TRUE(std::fabs(link_revolute->GetReaction2().force.y() - P_t) < 1e-4);

    //
    // TEST 2 : make the gear a bit compliant and see some small relative rotation
    //

    // Same setup as before, same applied torque, but now make the gear
    // teeth a bit compliant. Do this by providing a teeth equivalent stiffness (i.e.
    // a stiffness in [N/m] for the tangential direction, that already consider the
    // fact that multiple teeth are engaged, on average. Also introduce a damping coefficient
    // otherwise it will cause too many vibrations and won't settle to the steady state that we want
    // to measure.


    link_gearAB->SetTeethStiffness(teeth_K);
    link_gearAB->SetTeethDamping(teeth_D_coeff * teeth_K);

    // TEST EULER IMPLICIT
    auto mystepper = chrono_types::make_shared<ChTimestepperEulerImplicit>(&sys);
    mystepper->SetStepControl(false);
    mystepper->SetJacobianUpdateMethod(ChTimestepperHHT::JacobianUpdate::EVERY_ITERATION);
    mystepper->SetMaxIters(1);
    sys.SetTimestepper(mystepper);
    
    /*
    // TEST HHT
    auto mystepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    mystepper->SetStepControl(false);
    mystepper->SetJacobianUpdateMethod(ChTimestepperHHT::JacobianUpdate::EVERY_ITERATION);
    mystepper->SetMaxIters(4);
    sys.SetTimestepper(mystepper);
    */

    sys.SetSolverType(ChSolver::Type::GMRES);
    sys.GetSolver()->AsIterative()->SetMaxIterations(250);

    timestep = 0.005;
    while (sys.GetChTime() < 4) {
        // Advance simulation by one step
        sys.DoStepDynamics(timestep);

        // std::cout << "test 2 time: " << sys.GetChTime() << "  angle_B=" <<
        //  mbody_gearB->GetRotAngle() * CH_RAD_TO_DEG  << "\n";
    }
    std::cout << "TEST 2, compliant ChLinkLockGear: F_n=" << link_gearAB->GetContactForce() << std::endl;
    std::cout << "TEST 2, compliant ChLinkLockGear: ang.slip=" << mbody_gearB->GetRotAngle() * CH_RAD_TO_DEG
              << std::endl;


    // check angular slip is what expected because of teeth compliance:
    ASSERT_TRUE(std::fabs(mbody_gearB->GetRotAngle() * CH_RAD_TO_DEG - expected_angle_B) < 1e-4);

    //
    // TEST 3:   1D gear model using ChShaft objects
    //

    // Create a Chrono physical system
    ChSystemNSC sys_shafts;

    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetFixed(true);
    my_shaftA->SetInertia(100);
    sys_shafts.AddShaft(my_shaftA);

    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(314);
    my_shaftB->SetAppliedLoad(torque);
    sys_shafts.AddShaft(my_shaftB);

    auto my_shaft_gearAB = chrono_types::make_shared<ChShaftsGear>();
    my_shaft_gearAB->Initialize(my_shaftA, my_shaftB);
    my_shaft_gearAB->SetTransmissionRatio(-radA / radB);  // note minus sign, for outer&outer gears
    sys_shafts.Add(my_shaft_gearAB);

    // Test different solver types
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(); // direct: ok
    mkl_solver->LockSparsityPattern(true);
    sys_shafts.SetSolver(mkl_solver);
    /*
    sys_shafts.SetSolverType(ChSolver::Type::GMRES);  // iterative Schur-less: ok except MINRES cause E negative 
    sys_shafts.GetSolver()->AsIterative()->SetMaxIterations(100);
    */
    /*
    sys_shafts.SetSolverType(ChSolver::Type::PSOR);  // iterative Schur-based: ok, also BARZILAIBORWEIN etc.
    sys_shafts.GetSolver()->AsIterative()->SetMaxIterations(100);
    */

    timestep = 0.001;
    while (sys_shafts.GetChTime() < 0.02) {
        sys_shafts.DoStepDynamics(timestep);

        /*
        std::cout << "test 3 time: " << sys_shafts.GetChTime()
                  << "  angle_A=" << my_shaftA->GetPos() * CH_RAD_TO_DEG
                  << "  angle_B=" << my_shaftB->GetPos() * CH_RAD_TO_DEG
                  << "  P_t=" << my_shaft_gearAB->GetContactForceTangential(radB) << my_shaft_gearAB->GetReaction2()
                  << "\n";
        */
    }
    std::cout << "TEST 3, rigid ChShaftsGear: F_t=" << my_shaft_gearAB->GetContactForceTangential(radB) << std::endl;

    // check tangent contact force:
    ASSERT_TRUE(std::fabs(my_shaft_gearAB->GetContactForceTangential(radB) - P_t) < 1e-4);

    //
    // TEST 4:   make the gear a bit compliant and see some small relative rotation
    //

    // Same setup as before, same applied torque, but now make the gear
    // teeth a bit compliant. Do this by providing a teeth equivalent stiffness (i.e.
    // a stiffness in [N/m] for the tangential direction, that already consider the
    // fact that multiple teeth are engaged, on average. Also introduce a damping coefficient
    // otherwise it will cause too many vibrations and won't settle to the steady state that we want
    // to measure.

    my_shaft_gearAB->SetTeethStiffnessTangential(teeth_K_tangent, radB);
    my_shaft_gearAB->SetTorsionalDamping(0);

    // TEST EULER IMPLICIT
    /*
    auto mystepper_shafts = chrono_types::make_shared<ChTimestepperEulerImplicit>(&sys_shafts);
    mystepper_shafts->SetStepControl(false);
    mystepper_shafts->SetJacobianUpdateMethod(ChTimestepperHHT::JacobianUpdate::EVERY_ITERATION);
    mystepper_shafts->SetMaxIters(6);
    mystepper_shafts->SetVerbose(true);
    sys_shafts.SetTimestepper(mystepper_shafts);
    */
    
    // TEST HHT
    auto mystepper_shafts = chrono_types::make_shared<ChTimestepperHHT>(&sys_shafts);
    mystepper_shafts->SetStepControl(false);
    mystepper_shafts->SetJacobianUpdateMethod(ChTimestepperHHT::JacobianUpdate::EVERY_ITERATION);
    mystepper_shafts->SetMaxIters(4);
    //mystepper_shafts->SetVerbose(true);
    sys_shafts.SetTimestepper(mystepper_shafts);
    

    timestep = 0.05;
    while (sys_shafts.GetChTime() < 4) {
        // Advance simulation by one step
        sys_shafts.DoStepDynamics(timestep);

        /*
        std::cout << "test 4 time: " << sys_shafts.GetChTime() << "  angle_A=" << my_shaftA->GetPos() * CH_RAD_TO_DEG
            << "  angle_B=" << my_shaftB->GetPos() * CH_RAD_TO_DEG
            << "  P_t=" << my_shaft_gearAB->GetContactForceTangential(radB) << "\n";
        */
    }
    std::cout << "TEST 4, compliant ChShaftsGear: F_t=" << my_shaft_gearAB->GetContactForceTangential(radB) << std::endl;
    std::cout << "TEST 4, compliant ChShaftsGear: ang.slip=" << my_shaftB->GetPos() * CH_RAD_TO_DEG
              << std::endl;

    // check tangent contact force:
    ASSERT_TRUE(std::fabs(my_shaft_gearAB->GetContactForceTangential(radB) - P_t) < 1e-3);
    // check angular slip is what expected because of teeth compliance:
    ASSERT_TRUE(std::fabs(my_shaftB->GetPos() * CH_RAD_TO_DEG - expected_angle_B) < 1e-4);
}
