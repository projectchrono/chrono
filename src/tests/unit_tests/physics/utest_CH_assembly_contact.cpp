// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Tests for system assembly in the presence of contacts (issue #850).
//
// A 1 kg box (0.2 m cube) rests on a fixed ground box, with gravity along -Y.
// The box is first settled with a short dynamics run, so that contacts are
// present when DoAssembly() is called. Three cases are considered: resting,
// sliding (horizontal velocity), and separating (upward velocity).
//
// - NSC: the assembly analysis does not support contacts (with its tiny internal
//   step, the -C/dt stabilization drops resting contacts, and the contact rows
//   corrupt all assembly levels). DoAssembly() must return UNSUPPORTED_CONTACTS
//   at every assembly level and leave the system state unchanged.
// - NSC without contacts: the check is based on the contacts actually present,
//   so a box away from the ground assembles normally (free fall).
// - SMC: contacts enter as forces, so the assembly gives the expected results:
//   zero acceleration and a total normal force of m*g for the resting box, and
//   Coulomb friction for the sliding box. SMC contact forces are evaluated at
//   collision detection, so ComputeCollisions() is called after the velocity
//   of the box is set.
//
// =============================================================================

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

using namespace chrono;

namespace {

const double g = 9.81;
const double mass = 1.0;
const double side = 0.2;
const float mu = 0.5f;

enum class Motion { RESTING, SLIDING, SEPARATING };

const char* MotionName(Motion motion) {
    switch (motion) {
        case Motion::RESTING:
            return "resting";
        case Motion::SLIDING:
            return "sliding";
        case Motion::SEPARATING:
            return "separating";
    }
    return "";
}

ChVector3d InitialVelocity(Motion motion) {
    switch (motion) {
        case Motion::SLIDING:
            return ChVector3d(1, 0, 0);
        case Motion::SEPARATING:
            return ChVector3d(0, 1, 0);
        default:
            return ChVector3d(0, 0, 0);
    }
}

// Accumulate normal and tangential contact force magnitudes over all contacts.
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 double distance,
                                 double eff_radius,
                                 const ChVector3d& react_forces,
                                 const ChVector3d& react_torques,
                                 ChContactable* contactobjA,
                                 ChContactable* contactobjB,
                                 int constraint_offset) override {
        normal += std::abs(react_forces.x());
        tangential += std::sqrt(react_forces.y() * react_forces.y() + react_forces.z() * react_forces.z());
        count++;
        return true;
    }

    double normal = 0;
    double tangential = 0;
    int count = 0;
};

struct BoxOnGround {
    BoxOnGround(ChContactMethod method, double height = 0) {
        std::shared_ptr<ChContactMaterial> mat;
        if (method == ChContactMethod::NSC) {
            sys = chrono_types::make_shared<ChSystemNSC>();
            mat = chrono_types::make_shared<ChContactMaterialNSC>();
        } else {
            sys = chrono_types::make_shared<ChSystemSMC>();
            mat = chrono_types::make_shared<ChContactMaterialSMC>();
        }
        mat->SetFriction(mu);
        sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        sys->SetGravitationalAcceleration(ChVector3d(0, -g, 0));
        sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

        auto ground = chrono_types::make_shared<ChBodyEasyBox>(2, 0.2, 2, 1000, false, true, mat);
        ground->SetPos(ChVector3d(0, -0.1, 0));
        ground->SetFixed(true);
        sys->AddBody(ground);

        box = chrono_types::make_shared<ChBodyEasyBox>(side, side, side, mass / (side * side * side), false, true, mat);
        box->SetPos(ChVector3d(0, side / 2 + height, 0));
        sys->AddBody(box);
    }

    // Settle the box on the ground with a short dynamics run (this also runs collision detection).
    void Settle() {
        for (int i = 0; i < 200; i++)
            sys->DoStepDynamics(1e-3);
    }

    std::shared_ptr<ChSystem> sys;
    std::shared_ptr<ChBodyEasyBox> box;
};

}  // namespace

// NSC system with contacts present: assembly is not performed, at any level, and the state is not modified.
TEST(AssemblyContact, NSC_contacts_unsupported) {
    for (auto motion : {Motion::RESTING, Motion::SLIDING, Motion::SEPARATING}) {
        for (int level : {AssemblyAnalysis::Level::POSITION, AssemblyAnalysis::Level::VELOCITY,
                          AssemblyAnalysis::Level::ACCELERATION, AssemblyAnalysis::Level::FULL}) {
            BoxOnGround test(ChContactMethod::NSC);
            test.Settle();
            test.box->SetPosDt(InitialVelocity(motion));
            test.sys->ComputeCollisions();
            ASSERT_GT(test.sys->GetNumContacts(), 0u);

            ChVector3d pos = test.box->GetPos();
            ChQuaternion<> rot = test.box->GetRot();
            ChVector3d vel = test.box->GetPosDt();
            ChVector3d acc = test.box->GetPosDt2();
            double time = test.sys->GetChTime();

            auto flag = test.sys->DoAssembly(level);
            std::cout << "NSC " << MotionName(motion) << ", level " << level << ": flag = " << static_cast<int>(flag) << std::endl;
            EXPECT_EQ(flag, AssemblyAnalysis::ExitFlag::UNSUPPORTED_CONTACTS);

            EXPECT_EQ(test.box->GetPos(), pos);
            EXPECT_EQ(test.box->GetRot(), rot);
            EXPECT_EQ(test.box->GetPosDt(), vel);
            EXPECT_EQ(test.box->GetPosDt2(), acc);
            EXPECT_EQ(test.sys->GetChTime(), time);
        }
    }
}

// The kinematic analyses rely on DoAssembly and must propagate the flag.
TEST(AssemblyContact, NSC_contacts_unsupported_kinematics) {
    BoxOnGround test(ChContactMethod::NSC);
    test.Settle();
    ASSERT_GT(test.sys->GetNumContacts(), 0u);
    ChVector3d pos = test.box->GetPos();

    EXPECT_EQ(test.sys->DoStepKinematics(1e-3), AssemblyAnalysis::ExitFlag::UNSUPPORTED_CONTACTS);
    double time = test.sys->GetChTime();
    EXPECT_EQ(test.sys->DoFrameKinematics(time + 0.1, 1e-2), AssemblyAnalysis::ExitFlag::UNSUPPORTED_CONTACTS);
    EXPECT_NEAR(test.sys->GetChTime(), time + 1e-2, 1e-12);  // stopped after the first (rejected) assembly
    EXPECT_EQ(test.box->GetPos(), pos);
}

// NSC system without contacts: assembly proceeds normally (box in free fall).
TEST(AssemblyContact, NSC_no_contacts) {
    BoxOnGround test(ChContactMethod::NSC, 0.5);
    test.sys->DoStepDynamics(1e-3);  // runs collision detection
    ASSERT_EQ(test.sys->GetNumContacts(), 0u);

    auto flag = test.sys->DoAssembly(AssemblyAnalysis::Level::FULL);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::ACCELERATION_INACCURATE);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::UNSUPPORTED_CONTACTS);
    EXPECT_LT((test.box->GetPosDt2() - ChVector3d(0, -g, 0)).Length(), 1e-6);
}

// SMC system with contacts present: contacts enter as forces and the assembly gives the expected results.
TEST(AssemblyContact, SMC_contacts) {
    for (auto motion : {Motion::RESTING, Motion::SLIDING}) {
        BoxOnGround test(ChContactMethod::SMC);
        test.Settle();
        test.box->SetPosDt(InitialVelocity(motion));
        test.sys->ComputeCollisions();  // SMC contact forces are evaluated at collision detection
        ASSERT_GT(test.sys->GetNumContacts(), 0u);

        auto flag = test.sys->DoAssembly(AssemblyAnalysis::Level::FULL);
        EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);
        EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::ACCELERATION_INACCURATE);
        EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::UNSUPPORTED_CONTACTS);

        auto reporter = chrono_types::make_shared<ContactReporter>();
        test.sys->GetContactContainer()->ReportAllContacts(reporter);
        ChVector3d acc = test.box->GetPosDt2();
        std::cout << "SMC " << MotionName(motion) << ": contacts = " << reporter->count << "  N = " << reporter->normal
                  << "  T = " << reporter->tangential << "  a = " << acc << std::endl;

        EXPECT_GT(reporter->count, 0);
        EXPECT_NEAR(reporter->normal, mass * g, 0.02 * mass * g);
        switch (motion) {
            case Motion::RESTING:
                EXPECT_NEAR(reporter->tangential, 0.0, 1e-6);
                EXPECT_LT(acc.Length(), 1e-3);
                break;
            case Motion::SLIDING:
                // Coulomb friction opposing the sliding velocity
                EXPECT_NEAR(reporter->tangential, mu * reporter->normal, 1e-3);
                EXPECT_NEAR(acc.x(), -mu * reporter->normal / mass, 1e-3);
                break;
            default:
                break;
        }
    }
}
