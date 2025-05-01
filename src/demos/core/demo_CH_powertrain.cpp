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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demo code about creating a power train using the '1D' items of ChShaft class
// (rotating parts that have only one degree of freedom and one inertia value).
// This is an easier alternative to creating full 3D ChBody objects that rotate
// on revolute joints, etc.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChShaftsTorqueConverter.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChShaftsAppliedTorque.h"
#include "chrono/physics/ChShaftsThermalEngine.h"
#include "chrono/physics/ChShaftsFreewheel.h"
#include "chrono/physics/ChShaftsMotorPosition.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;

void Example1(const std::string& out_dir, ChSolver::Type solver_type, ChTimestepper::Type integrator_type) {
    std::cout << "\n-------------------------------------------------------------------------\n";
    std::cout << "Example: create a simple power train with ChShaft objects\n" << std::endl;

    // We will model a very basic powertrain with two shafts A and B,
    // connected by a reducer [ t ] with transmission ratio 't'. Shafts are
    // free to rotate, shaft A has an applied torque Ta, so A and B will
    // constantly accelerate. Each shaft must have some inertia, it's like a
    // flywheel, marked as || in the following scheme:
    //
    //       A           B
    //  Ta  ||---[ t ]---||
    //

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create a 1-degree-of-freedom '1D' mechanical object, that
    // is a ChShaft (an item that can oly rotate, with one inertia value
    // and maybe one applied torque). The ChShaft objects do not have
    // any meaning in 3d: they are just 'building blocks' for making
    // power trains as in imput-output black box schemes.
    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetInertia(10);
    my_shaftA->SetAppliedLoad(6);
    sys.AddShaft(my_shaftA);

    // Create another shaft. Note that we use shared pointers for ChShaft
    // objects, as we did for ChBody objects.
    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(100);
    my_shaftB->SetFixed(false);
    sys.AddShaft(my_shaftB);

    // Create a ChShaftsGear, that represents a simplified model
    // of a reducer, with transmission ratio t, between two ChShaft objects.
    // (Note that you could also build a 3D powertrain by creating full rigid bodies
    // of ChBody type and join them using ChLinkLockRevolute, ChLinkLockGear 3D constraints,
    // but this would introduce many unnecessary degrees of freedom/constraints
    // whereas the 1D items of ChShaft type, in this example, make things much simplier).
    auto my_shaft_gearAB = chrono_types::make_shared<ChShaftsGear>();
    my_shaft_gearAB->Initialize(my_shaftA, my_shaftB);
    my_shaft_gearAB->SetTransmissionRatio(-0.1);  // ex., a couple of spur gears with 20 and 200 teeth
    sys.Add(my_shaft_gearAB);

    std::cout << "\nSystem hierarchy" << std::endl;
    sys.ShowHierarchy(std::cout);

    std::ofstream file_results(out_dir + "/example1.txt");

    // Simulation loop
    double end_time = 2.5;
    double step_size = 0.01;

    for (double frame_time = 0.05; frame_time < end_time; frame_time += 0.05) {
        // Perform simulation up to frame_time
        sys.DoFrameDynamics(frame_time, step_size);

        // Print results
        std::cout << "Time: " << frame_time << std::endl
                  << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPosDt()
                  << "  accel: " << my_shaftA->GetPosDt2() << std::endl
                  << "  shaft B  rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPosDt()
                  << "  accel: " << my_shaftB->GetPosDt2() << std::endl
                  << "  AB gear, torque on A side: " << my_shaft_gearAB->GetReaction1()
                  << "  AB gear, torque on B side: " << my_shaft_gearAB->GetReaction2() << std::endl;
        file_results << frame_time << ", " << my_shaftA->GetPos() << ", " << my_shaftA->GetPosDt() << ", "
                     << my_shaftA->GetPosDt2() << ", " << my_shaftB->GetPos() << ", " << my_shaftB->GetPosDt() << ", "
                     << my_shaftB->GetPosDt2() << ", " << my_shaft_gearAB->GetReaction1() << ", "
                     << my_shaft_gearAB->GetReaction2() << std::endl;
    }

    file_results.close();
}

void Example2(const std::string& out_dir, ChSolver::Type solver_type, ChTimestepper::Type integrator_type) {
    std::cout << "\n-------------------------------------------------------------------------\n";
    std::cout << "Example: a clutch between two shafts\n" << std::endl;

    // We will model a very basic powertrain with two shafts A and B,
    // connected by a clutch [ c ]. Shafts (see flywheels || in scheme)
    // starts with nonzero speed, and are free to rotate independently
    // until the clutch is activated: since activation, they will decelerate
    // until they have the same speed.
    //
    //       A           B
    //  Ta  ||---[ c ]---||
    //

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create a ChShaft that starts with nonzero angular velocity
    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetInertia(0.5);
    my_shaftA->SetPosDt(30);
    sys.AddShaft(my_shaftA);

    // Create another ChShaft, with opposite initial angular velocity
    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(0.6);
    my_shaftB->SetPosDt(-10);
    sys.AddShaft(my_shaftB);

    // Create a ChShaftsClutch, that represents a simplified model
    // of a clutch between two ChShaft objects (something that limits
    // the max transmitted torque, up to slippage).
    auto my_shaft_clutchAB = chrono_types::make_shared<ChShaftsClutch>();
    my_shaft_clutchAB->Initialize(my_shaftA, my_shaftB);
    my_shaft_clutchAB->SetTorqueLimit(60);
    sys.Add(my_shaft_clutchAB);

    // Let's begin the simulation with the clutch disengaged:
    my_shaft_clutchAB->SetModulation(0);

    std::cout << "\nSystem hierarchy" << std::endl;
    sys.ShowHierarchy(std::cout);

    std::ofstream file_results(out_dir + "/example2.txt");

    // Simulation loop
    double end_time = 2.5;
    double step_size = 0.01;

    for (double frame_time = 0.05; frame_time < end_time; frame_time += 0.05) {
        // Perform simulation up to frame_time
        sys.DoFrameDynamics(frame_time, step_size);

        // Activate the clutch only after 0.8 seconds of simulation
        if (frame_time > 0.8) {
            my_shaft_clutchAB->SetModulation(1);
        }

        // Print results
        std::cout << "Time: " << frame_time << std::endl
                  << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPosDt()
                  << "  accel: " << my_shaftA->GetPosDt2() << std::endl
                  << "  shaft B  rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPosDt()
                  << "  accel: " << my_shaftB->GetPosDt2() << std::endl
                  << "  AB clutch, torque on A side: " << my_shaft_clutchAB->GetReaction1()
                  << "  AB clutch, torque on B side: " << my_shaft_clutchAB->GetReaction2() << std::endl;
        file_results << frame_time << ", " << my_shaftA->GetPos() << ", " << my_shaftA->GetPosDt() << ", "
                     << my_shaftA->GetPosDt2() << ", " << my_shaftB->GetPos() << ", " << my_shaftB->GetPosDt() << ", "
                     << my_shaftB->GetPosDt2() << ", " << my_shaft_clutchAB->GetReaction1() << ", "
                     << my_shaft_clutchAB->GetReaction2() << std::endl;
    }

    file_results.close();
}

void Example3(const std::string& out_dir, ChSolver::Type solver_type, ChTimestepper::Type integrator_type) {
    std::cout << "\n-------------------------------------------------------------------------\n";
    std::cout << "Example: an epicycloidal reducer\n" << std::endl;

    // We will model an epicycloidal reducer using the ChShaftsPlanetary
    // constraint.
    // The ChShaftsPlanetary makes a kinematic constraint between three
    // shafts: so one of them will be 'fixed' and will represent the truss
    // of the reducer -in s reducer, this is the role of the
    // large gear with inner teeth- and the two remaining shafts are the
    // input and output shafts (in other cases, such as the differential
    // planetary gear of the cars, all three shafts are free).
    // Also, a brake is applied for the output shaft: the ChShaftsClutch
    // will be used to this end, it's enough that one of the two shafts is fixed.
    // In the following scheme, the brake is [ b ], the planetary (the
    // reducer) is [ p ], the shafts are A,B,C,D applied torque is Ta, inertias
    // of free shafts are shown as flywheels || , and fixed shafts are marked with * .
    //
    //       A           B            D
    //  Ta  ||---[ p ]---||---[ b ]---*
    //           [   ]---*
    //                   C

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create shaft A, with applied torque
    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetInertia(0.5);
    my_shaftA->SetAppliedLoad(10);
    sys.AddShaft(my_shaftA);

    // Create shaft B
    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(0.5);
    sys.AddShaft(my_shaftB);

    // Create shaft C, that will be fixed (to be used as truss of epicycloidal reducer)
    auto my_shaftC = chrono_types::make_shared<ChShaft>();
    my_shaftC->SetFixed(true);
    sys.AddShaft(my_shaftC);

    // Create a ChShaftsPlanetary, that represents a simplified model
    // of a planetary gear between THREE ChShaft objects (ex.: a car differential)
    // An epicycloidal reducer is a special type of planetary gear.
    auto my_shaft_planetaryBAC = chrono_types::make_shared<ChShaftsPlanetary>();
    my_shaft_planetaryBAC->Initialize(my_shaftB, my_shaftA, my_shaftC);  // output, carrier, fixed
    // We can set the ratios of the planetary using a simplified formula, for the
    // so called 'Willis' case. Imagine we hold fixed the carrier (shaft B in epic. reducers),
    // and leave free the truss C (the outer gear with inner teeth in our reducer); which is
    // the transmission ratio t0 that we get? It is simply t0=-Za/Zc, with Z = num of teeth of gears.
    // So just use the following to set all the three ratios automatically:
    double t0 = -50.0 / 100.0;  // suppose, in the reducer, that pinion A has 50 teeth and truss has 100 inner teeth.
    my_shaft_planetaryBAC->SetTransmissionRatioOrdinary(t0);
    sys.Add(my_shaft_planetaryBAC);

    // Now, let's make a shaft D, that is fixed, and used for the right side
    // of a clutch (so the clutch will act as a brake).
    auto my_shaftD = chrono_types::make_shared<ChShaft>();
    my_shaftD->SetFixed(true);
    sys.Add(my_shaftD);

    // Make the brake. It is, in fact a clutch between shafts B and D, where
    // D is fixed as a truss, so the clutch will operate as a brake.
    auto my_shaft_clutchBD = chrono_types::make_shared<ChShaftsClutch>();
    my_shaft_clutchBD->Initialize(my_shaftB, my_shaftD);
    my_shaft_clutchBD->SetTorqueLimit(60);
    sys.Add(my_shaft_clutchBD);

    std::cout << "\nSystem hierarchy" << std::endl;
    sys.ShowHierarchy(std::cout);

    std::ofstream file_results(out_dir + "/example3.txt");

    // Simulation loop
    double end_time = 2.5;
    double step_size = 0.01;

    for (double frame_time = 0.05; frame_time < end_time; frame_time += 0.05) {
        // Perform simulation up to frame_time
        sys.DoFrameDynamics(frame_time, step_size);

        // Print results
        std::cout << "Time: " << frame_time << std::endl
                  << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPosDt()
                  << "  accel: " << my_shaftA->GetPosDt2() << std::endl
                  << "  shaft B  rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPosDt()
                  << "  accel: " << my_shaftB->GetPosDt2() << std::endl
                  << "  epicycloidal react torques on shafts - on A: " << my_shaft_planetaryBAC->GetReaction2()
                  << " ,   on B: " << my_shaft_planetaryBAC->GetReaction1()
                  << " ,   on C: " << my_shaft_planetaryBAC->GetTorqueReactionOn3() << std::endl;
        file_results << frame_time << ", " << my_shaftA->GetPos() << ", " << my_shaftA->GetPosDt() << ", "
                     << my_shaftA->GetPosDt2() << ", " << my_shaftB->GetPos() << ", " << my_shaftB->GetPosDt() << ", "
                     << my_shaftB->GetPosDt2() << ", " << my_shaft_planetaryBAC->GetReaction2() << ", "
                     << my_shaft_planetaryBAC->GetReaction1() << ", " << my_shaft_planetaryBAC->GetTorqueReactionOn3()
                     << std::endl;
    }

    file_results.close();
}

void Example4a(const std::string& out_dir, ChSolver::Type solver_type, ChTimestepper::Type integrator_type) {
    std::cout << "\n-------------------------------------------------------------------------\n";
    std::cout << "Example: constraint between a ChBody and a ChShaft\n" << std::endl;

    // Suppose you want to create a 3D model, for instance a slider-crank,
    // built with multiple ChBody objects; moreover you want to create a
    // powertrain, for instance a motor, a clutch, etc, for the rotation of
    // the crank. How to connect the '1D items' of ChShaft class to the 3D
    // items of ChBody class? The solution is to use the ChShaftBodyRotation constraint,
    // shown as [ bs ] in the following scheme, where the 3D body is shown as <>.
    // In this example we also add a 'torsional spring damper' C, shown as [ t ]
    // that connects shafts A and C (C is shown as * because fixed).
    //
    //        B             A           C
    //  Ta   <>---[ bs ]---||---[ t ]---*
    //

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create 'A', a 1D shaft
    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetInertia(9);
    sys.AddShaft(my_shaftA);

    // Create 'C', a 1D shaft, fixed
    auto my_shaftC = chrono_types::make_shared<ChShaft>();
    my_shaftC->SetFixed(true);
    sys.AddShaft(my_shaftC);

    // Create 'B', a 3D rigid body
    auto my_bodyB = chrono_types::make_shared<ChBody>();
    my_bodyB->AddAccumulator();                                // single accumlator on this body (index = 0)
    my_bodyB->AccumulateTorque(0, ChVector3d(0, 0, 3), true);  // set some constant torque to body
    sys.Add(my_bodyB);

    // Make the torsional spring-damper between shafts A and C.
    auto my_shaft_torsionAC = chrono_types::make_shared<ChShaftsTorsionSpring>();
    my_shaft_torsionAC->Initialize(my_shaftA, my_shaftC);
    my_shaft_torsionAC->SetTorsionalStiffness(40);
    my_shaft_torsionAC->SetTorsionalDamping(0);
    sys.Add(my_shaft_torsionAC);

    // Make the shaft 'A' connected to the rotation of the 3D body 'B'.
    // We must specify the direction (in body coordinates) along which the
    // shaft will affect the body.
    auto my_shaftbody_connection = chrono_types::make_shared<ChShaftBodyRotation>();
    ChVector3d mshaftdir(VECT_Z);
    my_shaftbody_connection->Initialize(my_shaftA, my_bodyB, mshaftdir);
    sys.Add(my_shaftbody_connection);

    std::cout << "\nSystem hierarchy" << std::endl;
    sys.ShowHierarchy(std::cout);

    std::ofstream file_results(out_dir + "/example4a.txt");

    // Simulation loop
    double end_time = 2.5;
    double step_size = 0.01;

    for (double frame_time = 0.05; frame_time < end_time; frame_time += 0.05) {
        // Perform simulation up to frame_time
        sys.DoFrameDynamics(frame_time, step_size);

        // Print results
        std::cout << "Time: " << frame_time << std::endl
                  << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPosDt()
                  << "  accel: " << my_shaftA->GetPosDt2() << std::endl
                  << "  Body B angular speed on z: " << my_bodyB->GetAngVelLocal().z()
                  << "  accel on z: " << my_bodyB->GetAngAccLocal().z() << std::endl
                  << "  AC spring, torque on A side: " << my_shaft_torsionAC->GetReaction1()
                  << "  torque on C side: " << my_shaft_torsionAC->GetReaction2() << std::endl
                  << "  shafts/body reaction,  on shaft A: " << my_shaftbody_connection->GetTorqueReactionOnShaft()
                  << " ,   on body (x y z): " << my_shaftbody_connection->GetTorqueReactionOnBody().x() << " "
                  << my_shaftbody_connection->GetTorqueReactionOnBody().y() << " "
                  << my_shaftbody_connection->GetTorqueReactionOnBody().z() << " " << std::endl;
        file_results << frame_time << ", " << my_shaftA->GetPos() << ", " << my_shaftA->GetPosDt() << ", "
                     << my_shaftA->GetPosDt2() << ", " << my_bodyB->GetAngVelLocal().z() << ", "
                     << my_bodyB->GetAngAccLocal().z() << ", " << my_shaft_torsionAC->GetReaction1() << ", "
                     << my_shaft_torsionAC->GetReaction2() << ", "
                     << my_shaftbody_connection->GetTorqueReactionOnShaft() << ", "
                     << my_shaftbody_connection->GetTorqueReactionOnBody().x() << ", "
                     << my_shaftbody_connection->GetTorqueReactionOnBody().y() << ", "
                     << my_shaftbody_connection->GetTorqueReactionOnBody().z() << std::endl;
    }

    file_results.close();
}


void Example4b(const std::string& out_dir, ChSolver::Type solver_type, ChTimestepper::Type integrator_type) {
    std::cout << "\n-------------------------------------------------------------------------\n";
    std::cout << "Example: constraint between a ChBody and a ChShaft\n" << std::endl;

    // Same as Example4a, but with a rotational spring-damper between bodies A and C and a torque applied to shaft B.
    //
    //        B             A           C
    //  Ta   ||---[ bs ]---<>---[ t ]---*
    //

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create 'B', a 1D shaft
    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(9);
    my_shaftB->SetAppliedLoad(3);
    sys.AddShaft(my_shaftB);

    // Create 'C', a 3D rigid body, fixed
    auto my_bodyC = chrono_types::make_shared<ChBody>();
    my_bodyC->SetFixed(true);
    sys.AddBody(my_bodyC);

    // Create 'A', a 3D rigid body
    auto my_bodyA = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_bodyA);

    // Make a torsional spring between bodies A and C
    auto my_rsdaAC = chrono_types::make_shared<ChLinkRSDA>();
    my_rsdaAC->Initialize(my_bodyA, my_bodyC, ChFramed());
    my_rsdaAC->SetSpringCoefficient(40);
    my_rsdaAC->SetDampingCoefficient(0);
    sys.Add(my_rsdaAC);

    // Make the shaft 'A' connected to the rotation of the 3D body 'B'.
    // We must specify the direction (in body coordinates) along which the shaft will affect the body.
    auto my_shaftbody_connection = chrono_types::make_shared<ChShaftBodyRotation>();
    ChVector3d mshaftdir(VECT_Z);
    my_shaftbody_connection->Initialize(my_shaftB, my_bodyA, mshaftdir);
    sys.Add(my_shaftbody_connection);

    std::cout << "\nSystem hierarchy" << std::endl;
    sys.ShowHierarchy(std::cout);

    std::ofstream file_results(out_dir + "/example4b.txt");

    // Simulation loop
    double end_time = 2.5;
    double step_size = 0.01;

    for (double frame_time = 0.05; frame_time < end_time; frame_time += 0.05) {
        // Perform simulation up to frame_time
        sys.DoFrameDynamics(frame_time, step_size);

        // Print results
        std::cout << "Time: " << frame_time << std::endl
                  << "  shaft B rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPosDt()
                  << "  accel: " << my_shaftB->GetPosDt2() << std::endl
                  << "  Body A angular speed on z: " << my_bodyA->GetAngVelLocal().z()
                  << "  accel on z: " << my_bodyA->GetAngAccLocal().z() << std::endl
                  << "  AC spring, torque on A side: " << my_rsdaAC->GetReaction1().torque
                  << "  torque on C side: " << my_rsdaAC->GetReaction2().torque << std::endl
                  << "  shafts/body reaction,  on shaft B: " << my_shaftbody_connection->GetTorqueReactionOnShaft()
                  << " ,   on body A: " << my_shaftbody_connection->GetTorqueReactionOnBody() << std::endl;
    }

    file_results.close();
}

void Example5(const std::string& out_dir, ChSolver::Type solver_type, ChTimestepper::Type integrator_type) {
    std::cout << "\n-------------------------------------------------------------------------\n";
    std::cout << "Example 5: torque converter and thermal engine\n" << std::endl;

    // In this example we use a torque converter.
    // The torque converter is represented by a ChShaftsTorqueConverter
    // object, that connects three '1D items' of ChShaft class:
    // - the input shaft A, ie. the impeller connected to the engine
    // - the output shaft B, i.e. the turbine connected to the gears and wheels
    // - the stator C, that does not rotate and transmits reaction to the truss.
    // In the following scheme, the torque converter is represented as [ tc ],
    // and we also add a thermal engine, shown with [ e ], and a breaking torque Tb
    // (C is shown as * because fixed).
    //
    //   D           A             B
    //   *---[ e ]---||---[ tc ]---||  Tb
    //                    [    ]---*
    //                             C
    //

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create 'A', a 1D shaft
    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetInertia(1.5);
    sys.AddShaft(my_shaftA);

    // Create 'B', a 1D shaft
    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(3.2);
    my_shaftB->SetAppliedLoad(-5);  // apply const braking torque
    sys.AddShaft(my_shaftB);

    // Create 'C', a 1D shaft, fixed
    auto my_shaftC = chrono_types::make_shared<ChShaft>();
    my_shaftC->SetFixed(true);
    sys.AddShaft(my_shaftC);

    // Create 'D', a 1D shaft, fixed
    auto my_shaftD = chrono_types::make_shared<ChShaft>();
    my_shaftD->SetFixed(true);
    sys.AddShaft(my_shaftD);

    // Make the torque converter and connect the shafts:
    // A (input),B (output), C(truss stator)
    auto my_torqueconverter = chrono_types::make_shared<ChShaftsTorqueConverter>();
    my_torqueconverter->Initialize(my_shaftA, my_shaftB, my_shaftC);
    sys.Add(my_torqueconverter);

    auto mK = chrono_types::make_shared<ChFunctionInterp>();
    mK->AddPoint(0.0, 15);
    mK->AddPoint(0.25, 15);
    mK->AddPoint(0.50, 15);
    mK->AddPoint(0.75, 16);
    mK->AddPoint(0.90, 18);
    mK->AddPoint(1.00, 35);
    my_torqueconverter->SetCurveCapacityFactor(mK);

    auto mT = chrono_types::make_shared<ChFunctionInterp>();
    mT->AddPoint(0.0, 2.00);
    mT->AddPoint(0.25, 1.80);
    mT->AddPoint(0.50, 1.50);
    mT->AddPoint(0.75, 1.15);
    mT->AddPoint(1.00, 1.00);
    my_torqueconverter->SetCurveTorqueRatio(mT);

    // Create the thermal engine, acting on shaft A, the input to
    // the torque converter. Note that the thermal engine also
    // requires another shaft D, that is used to transmit the
    // reaction torque back to a truss (the motor block).

    // Option 1: use a ChShaftsAppliedTorque, it just applies a torque
    // to my_shaftA (say, the crankshaft) and the negative torque
    // to my_shaftD (say, the motor block).
    // It is a quick approach. But you should take care of changing
    // the torque at each timestep if you want to simulate a torque curve...
    /*
    auto my_motor = chrono_types::make_shared<ChShaftsAppliedTorque>();
    my_motor->Initialize(my_shaftA, my_shaftD);
    my_motor->SetTorque(30);
    sys.Add(my_motor);
    */

    // Option 2: a more powerful approach where you can
    // define a torque curve and a throttle value, using the
    // ChShaftsThermalEngine.

    auto my_motor = chrono_types::make_shared<ChShaftsThermalEngine>();
    my_motor->Initialize(my_shaftA, my_shaftD);
    sys.Add(my_motor);

    auto mTw = chrono_types::make_shared<ChFunctionInterp>();
    mTw->AddPoint(-5, 30);  //   [rad/s],  [Nm]
    mTw->AddPoint(0, 30);
    mTw->AddPoint(200, 60);
    mTw->AddPoint(400, 40);
    mTw->AddPoint(450, 0);
    mTw->AddPoint(500, -60);  // torque curve must be defined beyond max speed too - engine might be 'pulled'
    my_motor->SetTorqueCurve(mTw);

    std::cout << "\nSystem hierarchy" << std::endl;
    sys.ShowHierarchy(std::cout);

    std::ofstream file_results(out_dir + "/example5.txt");

    // Simulation loop
    double end_time = 2.5;
    double step_size = 0.01;

    for (double frame_time = 0.05; frame_time < end_time; frame_time += 0.05) {
        // Perform simulation up to frame_time
        sys.DoFrameDynamics(frame_time, step_size);

        // Print results
        std::cout << "Time: " << frame_time << std::endl
                  << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPosDt()
                  << "  accel: " << my_shaftA->GetPosDt2() << std::endl
                  << "  shaft B rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPosDt()
                  << "  accel: " << my_shaftB->GetPosDt2() << std::endl
                  << "  T.Convert.:"
                  << "  R=" << my_torqueconverter->GetSpeedRatio()
                  << "  Tin=" << my_torqueconverter->GetTorqueReactionOnInput()
                  << "  Tout=" << my_torqueconverter->GetTorqueReactionOnOutput()
                  << "  Tstator=" << my_torqueconverter->GetTorqueReactionOnStator() << std::endl
                  << "  T.Motor: "
                  << "  T(w)=" << my_motor->GetReaction1() << "[Nm]"
                  << "  w=" << my_motor->GetRelativePosDt() << "[rad/s]" << std::endl;
        file_results << frame_time << ", " << my_shaftA->GetPos() << my_shaftA->GetPosDt() << ", "
                     << my_shaftA->GetPosDt2() << ", " << my_shaftB->GetPos() << my_shaftB->GetPosDt() << ", "
                     << my_shaftB->GetPosDt2() << ", " << my_torqueconverter->GetSpeedRatio() << ", "
                     << my_torqueconverter->GetTorqueReactionOnInput() << ", "
                     << my_torqueconverter->GetTorqueReactionOnOutput() << ", "
                     << my_torqueconverter->GetTorqueReactionOnStator() << ", " << my_motor->GetReaction1() << ", "
                     << my_motor->GetRelativePosDt() << std::endl;
    }

    file_results.close();
}

void Example6(const std::string& out_dir, ChSolver::Type solver_type, ChTimestepper::Type integrator_type) {
    std::cout << "\n-------------------------------------------------------------------------\n";
    std::cout << "Example 6: a ratcheting freewheel, as a one-directional clutch\n" << std::endl;

    // In this example we use a ratcheting freewheel, that acts as a one-directional
    // clutch (where the locking in reverse direction happens only at discrete
    // steps, depending on the n.of ratcheting teeths).
    // The example consists of:
    // - the fixed shaft A
    // - the shaft B, that rotates back and forth
    // - the shaft C, that rotates only unidirectionally
    // - the fixed shaft D
    // In the following scheme
    // - the freewheel is represented as [ fw ],
    // - we also add a motor, shown with [ m ], to generate sinusoidal rotation of B for testing
    // - we also add a clutch, shown with [ cl ], just to keep the shaft C "stopped" when not
    //   pushed by the unidirectional freewheel, otherwise would proceed in one direction forever.
    // (A,D are shown as * because fixed).
    //
    //   A           B             C             D
    //   *---[ m ]---||---[ fw ]---||---[ cl ]---*
    //

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create 'A', a 1D shaft, fixed
    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetFixed(true);
    sys.AddShaft(my_shaftA);

    // Create 'B', a 1D shaft
    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(1.5);
    sys.AddShaft(my_shaftB);

    // Create 'C', a 1D shaft
    auto my_shaftC = chrono_types::make_shared<ChShaft>();
    my_shaftC->SetInertia(3.2);
    sys.AddShaft(my_shaftC);

    // Create D', a 1D shaft, fixed
    auto my_shaftD = chrono_types::make_shared<ChShaft>();
    my_shaftD->SetFixed(true);
    sys.AddShaft(my_shaftD);

    // Make the motor imposing a test sinusoidal rotation
    auto my_motor = chrono_types::make_shared<ChShaftsMotorPosition>();
    my_motor->Initialize(my_shaftA, my_shaftB);
    sys.Add(my_motor);
    auto my_sinefunction = chrono_types::make_shared<ChFunctionSine>(0.001 + 0.5 * CH_2PI / 20, 1.2);
    my_motor->SetPositionFunction(my_sinefunction);

    // Make the freewheel:
    auto my_freewheel = chrono_types::make_shared<ChShaftsFreewheel>();
    my_freewheel->Initialize(my_shaftB, my_shaftC);
    my_freewheel->SetRatchetingModeTeeth(25);
    // my_freewheel->SetJammingMode(); // this is like having infinite teeth, i.e. no backlash
    // my_freewheel->SetFreeBackward(); // this is to reverse the unidirectional behavior
    sys.Add(my_freewheel);

    // Make the clutch that keeps the shaft C in place:
    auto my_clutch = chrono_types::make_shared<ChShaftsClutch>();
    my_clutch->Initialize(my_shaftC, my_shaftD);
    my_clutch->SetTorqueLimit(100);
    sys.Add(my_clutch);

    std::cout << "\nSystem hierarchy" << std::endl;
    sys.ShowHierarchy(std::cout);

    std::ofstream file_results(out_dir + "/example6.txt");

    // Simulation loop
    double step = 0.01;

    while (sys.GetChTime() < 5.5) {
        // Advance dynamics by one step
        sys.DoStepDynamics(step);

        // Print results
        std::cout << "Time: " << sys.GetChTime() << std::endl
                  << "  shaft B rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPosDt()
                  << "  accel: " << my_shaftB->GetPosDt2() << std::endl
                  << "  shaft C rot: " << my_shaftC->GetPos() << "  speed: " << my_shaftC->GetPosDt()
                  << "  accel: " << my_shaftC->GetPosDt2() << std::endl
                  << "  Torque: Tmotor=" << my_motor->GetReaction1() << "  Tfreewheel=" << my_freewheel->GetReaction1()
                  << "  Tclutch=" << my_clutch->GetReaction1()
                  << "  ratchet vane=" << my_freewheel->GetCurrentTeethVane() << std::endl;
        file_results << sys.GetChTime() << ", " << my_shaftB->GetPos() << ", " << my_shaftC->GetPos() << ", "
                     << my_shaftC->GetPosDt() << ", " << my_clutch->GetReaction1() << ", "
                     << my_freewheel->GetCurrentTeethVane() << std::endl;
    }

    file_results.close();
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "DEMO_POWERTRAIN";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    auto solver_type = ChSolver::Type::BARZILAIBORWEIN;
    auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

    Example1(out_dir, solver_type, integrator_type);
    Example2(out_dir, solver_type, integrator_type);
    Example3(out_dir, solver_type, integrator_type);
    Example4a(out_dir, solver_type, integrator_type);
    Example4b(out_dir, solver_type, integrator_type);
    Example5(out_dir, solver_type, integrator_type);
    Example6(out_dir, solver_type, integrator_type);

    return 0;
}
