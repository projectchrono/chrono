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

using namespace chrono;

/*
static void TestEqualDouble(const ChQuaternion<double>& q1, const ChQuaternion<double>& q2) {
    ASSERT_DOUBLE_EQ(q1.e0(), q2.e0());
    ASSERT_DOUBLE_EQ(q1.e1(), q2.e1());
    ASSERT_DOUBLE_EQ(q1.e2(), q2.e2());
    ASSERT_DOUBLE_EQ(q1.e3(), q2.e3());
}

static void TestEqualFloat(const ChQuaternion<float>& q1, const ChQuaternion<float>& q2) {
    ASSERT_FLOAT_EQ(q1.e0(), q2.e0());
    ASSERT_FLOAT_EQ(q1.e1(), q2.e1());
    ASSERT_FLOAT_EQ(q1.e2(), q2.e2());
    ASSERT_FLOAT_EQ(q1.e3(), q2.e3());
}
*/

TEST(ChLinkLockGear, cylindrical) {
    
    // Create a Chrono physical system
    ChSystemNSC sys;

    // Contact material shared among all bodies
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create all the rigid bodies.
    double radA = 4;
    double radB = 2;
    double torque = 100;
    double alpha = 20 * CH_DEG_TO_RAD;  // pressure angle, normal 
    double beta = 15  * CH_DEG_TO_RAD;  // helix angle

    // ...the truss
    auto mbody_truss = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
    sys.Add(mbody_truss);
    mbody_truss->SetFixed(true);
    mbody_truss->SetPos(ChVector3d(0, 0, 3));

    // ...the first gear
    auto mbody_gearA = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radA, 0.5, 1000, true, false, mat);
    sys.Add(mbody_gearA);
    mbody_gearA->SetPos(ChVector3d(0, 0, -1));
    mbody_gearA->SetRot(QuatFromAngleX(CH_PI_2));

    // ...the second gear bearing: motor that imposes (zero) rotation speed 
    auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motor->Initialize(mbody_gearA, mbody_truss, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    link_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    sys.AddLink(link_motor);

    // ...the second gear
    double interaxis12 = radA + radB;
    auto mbody_gearB = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radB, 0.4, 1000, true, false, mat);
    sys.Add(mbody_gearB);
    mbody_gearB->SetPos(ChVector3d(interaxis12, 0, -1));
    mbody_gearB->SetRot(QuatFromAngleX(CH_PI_2));
    
    // ...apply torque to second gear
    auto mtorque = chrono_types::make_shared<ChForce>();
    mtorque->SetMode(ChForce::TORQUE);
    mtorque->SetBody(mbody_gearB.get());
    mtorque->SetDir(ChVector3d(0, 0, 1));
    mtorque->SetMforce(torque);
    mbody_gearB->AddForce(mtorque);
    
    // ... the second gear bearing
    auto link_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revolute->Initialize(mbody_gearB, mbody_truss, ChFrame<>(ChVector3d(interaxis12, 0, 0), QUNIT));
    sys.AddLink(link_revolute);

    
    // ...the gear constraint between the two wheels A and B.
    auto link_gearAB = chrono_types::make_shared<ChLinkLockGear>();
    link_gearAB->Initialize(mbody_gearA, mbody_gearB, ChFrame<>());
    link_gearAB->SetFrameShaft1(ChFrame<>(VNULL, chrono::QuatFromAngleX(-CH_PI_2)));
    link_gearAB->SetFrameShaft2(ChFrame<>(VNULL, chrono::QuatFromAngleX(-CH_PI_2)));
    link_gearAB->SetTransmissionRatio(radA / radB);
    link_gearAB->SetEnforcePhase(true);
    link_gearAB->SetPressureAngle(alpha);  
    link_gearAB->SetPitchAngle(beta);  
    sys.AddLink(link_gearAB);

    sys.SetGravitationalAcceleration(VNULL);
    // sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    // Simulate few steps
    double timestep = 0.001;
    while (sys.GetChTime()<0.02) {

        // Advance simulation by one step
        sys.DoStepDynamics(timestep);
    }
    
    
    
    // check if corresponds to P contact force, from analytical solutions for helical gears 
    double P_t = torque / radB;  // tangent component 
    double P_r = P_t * (tan(alpha)/cos(beta)); // radial component
    double P_a = P_t * tan(beta);              // axial component

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
}
