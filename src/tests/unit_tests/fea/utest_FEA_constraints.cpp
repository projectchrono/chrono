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
// Authors: Simone Benatti, Radu Serban
// =============================================================================
//
// Test simulation of ANCF cables connected to rigid bodies.
// Compare results using MINRES and MKL solvers.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_mkl/ChSolverMKL.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::fea;

class Model {
  public:
    Model();
    std::shared_ptr<ChSystemNSC> GetSystem() const { return m_system; }
    std::shared_ptr<ChBodyEasyBox> GetBox1() const { return m_box1; }
    std::shared_ptr<ChBodyEasyBox> GetBox2() const { return m_box2; }

  private:
    std::shared_ptr<ChSystemNSC> m_system;
    std::shared_ptr<ChBodyEasyBox> m_box1;
    std::shared_ptr<ChBodyEasyBox> m_box2;
};

Model::Model() {
    m_system = std::make_shared<ChSystemNSC>();

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // pasted part begins
    auto msection_cable2 = std::make_shared<ChBeamSectionCable>();
    msection_cable2->SetDiameter(0.015);
    msection_cable2->SetYoungModulus(0.01e9);
    msection_cable2->SetBeamRaleyghDamping(0.000);

    auto mtruss = std::make_shared<ChBody>();
    mtruss->SetBodyFixed(true);

    ChBuilderBeamANCF builder;

    // Now, simply use BuildBeam to create a beam from a point to another:
    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection_cable2,       // ChBeamSectionCable to use for the ChElementBeamANCF elements
                      1,                     // number of ChElementBeamANCF to create
                      ChVector<>(0, 0, 0),   // point A (beginning of beam)
                      ChVector<>(0.1, 0, 0)  // point B (end of beam)
    );

    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, -0.2, 0));

    auto constraint_hinge = std::make_shared<ChLinkPointFrame>();
    constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), mtruss);
    m_system->Add(constraint_hinge);

    auto msphere = std::make_shared<ChSphereShape>();
    msphere->GetSphereGeometry().rad = 0.02;
    constraint_hinge->AddAsset(msphere);

    // make a box and connect it
    m_box1 = std::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
    m_box1->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
    m_system->Add(m_box1);

    auto constraint_pos = std::make_shared<ChLinkPointFrame>();
    constraint_pos->Initialize(builder.GetLastBeamNodes().back(), m_box1);
    m_system->Add(constraint_pos);

    auto constraint_dir = std::make_shared<ChLinkDirFrame>();
    constraint_dir->Initialize(builder.GetLastBeamNodes().back(), m_box1);
    constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
    m_system->Add(constraint_dir);

    // make another beam
    builder.BuildBeam(my_mesh,          // mesh where to put the created nodes and elements
                      msection_cable2,  // ChBeamSectionCable to use for the ChElementBeamANCF elements
                      7,                // number of ChElementBeamANCF to create
                      ChVector<>(m_box1->GetPos().x() + 0.1, 0, 0),           // point A (beginning of beam)
                      ChVector<>(m_box1->GetPos().x() + 0.1 + 0.1 * 6, 0, 0)  // point B (end of beam)
    );

    auto constraint_pos2 = std::make_shared<ChLinkPointFrame>();
    constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), m_box1);
    m_system->Add(constraint_pos2);

    auto constraint_dir2 = std::make_shared<ChLinkDirFrame>();
    constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), m_box1);
    constraint_dir2->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
    m_system->Add(constraint_dir2);

    // make a box and connect it
    m_box2 = std::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
    m_box2->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
    m_system->Add(m_box2);

    auto constraint_pos3 = std::make_shared<ChLinkPointFrame>();
    constraint_pos3->Initialize(builder.GetLastBeamNodes().back(), m_box2);
    m_system->Add(constraint_pos3);

    auto constraint_dir3 = std::make_shared<ChLinkDirFrame>();
    constraint_dir3->Initialize(builder.GetLastBeamNodes().back(), m_box2);
    constraint_dir3->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
    m_system->Add(constraint_dir3);

    m_system->Add(my_mesh);
    m_system->SetupInitial();

    // Set integrator
    m_system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    ////my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);

    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetAbsTolerances(1e-6);
    }
}

static void CompareVectors(const ChVector<>& v1, const ChVector<>& v2, double tol) {
    ASSERT_NEAR(v1.x(), v2.x(), tol);
    ASSERT_NEAR(v1.y(), v2.y(), tol);
    ASSERT_NEAR(v1.z(), v2.z(), tol);
}

static void CompareQuaternions(const ChQuaternion<>& q1, const ChQuaternion<>& q2, double tol) {
    ASSERT_NEAR(q1.e0(), q2.e0(), tol);
    ASSERT_NEAR(q1.e1(), q2.e1(), tol);
    ASSERT_NEAR(q1.e2(), q2.e2(), tol);
    ASSERT_NEAR(q1.e3(), q2.e3(), tol);
}

TEST(ANCFcables_rigid_constraints, Minres_MKL) {
    Model model1;
    Model model2;

    // Model1 uses MINRES
    model1.GetSystem()->SetSolverType(ChSolver::Type::MINRES);
    model1.GetSystem()->SetSolverWarmStarting(true);
    model1.GetSystem()->SetMaxItersSolverSpeed(200);
    model1.GetSystem()->SetMaxItersSolverStab(200);
    model1.GetSystem()->SetTolForce(1e-13);

    // MODEL2 uses MKL (Pardiso)
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    mkl_solver->SetSparsityPatternLock(false);
    model2.GetSystem()->SetSolver(mkl_solver);

    // Simulate both systems and compare states of the two rigid boxes
    const double precision = 1e-4;
    double timestep = 0.002;
    int num_steps = 1500;

    for (int i = 0; i < num_steps; i++) {
        model1.GetSystem()->DoStepDynamics(timestep);
        model2.GetSystem()->DoStepDynamics(timestep);

        CompareVectors(model1.GetBox1()->GetPos(), model2.GetBox1()->GetPos(), precision);
        CompareVectors(model1.GetBox2()->GetPos(), model2.GetBox2()->GetPos(), precision);
        CompareQuaternions(model1.GetBox1()->GetRot(), model2.GetBox1()->GetRot(), precision);
        CompareQuaternions(model1.GetBox2()->GetRot(), model2.GetBox2()->GetRot(), precision);
    }

    {
        auto p1 = model1.GetBox1()->GetPos();
        auto p2 = model1.GetBox2()->GetPos();
        auto q1 = model1.GetBox1()->GetRot();
        auto q2 = model1.GetBox2()->GetRot();
        std::cout << "Minres" << std::endl;
        std::cout << "  Box1: " << p1.x() << " " << p1.y() << " " << p1.z() << " ";
        std::cout << q1.e0() << " " << q1.e1() << " " << q1.e2() << " " << q1.e3() << std::endl;
        std::cout << "  Box2: " << p2.x() << " " << p2.y() << " " << p2.z() << " ";
        std::cout << q2.e0() << " " << q2.e1() << " " << q2.e2() << " " << q2.e3() << std::endl;
    }
    {
        auto p1 = model2.GetBox1()->GetPos();
        auto p2 = model2.GetBox2()->GetPos();
        auto q1 = model2.GetBox1()->GetRot();
        auto q2 = model2.GetBox2()->GetRot();
        std::cout << "MKL" << std::endl;
        std::cout << "  Box1: " << p1.x() << " " << p1.y() << " " << p1.z() << " ";
        std::cout << q1.e0() << " " << q1.e1() << " " << q1.e2() << " " << q1.e3() << std::endl;
        std::cout << "  Box2: " << p2.x() << " " << p2.y() << " " << p2.z() << " ";
        std::cout << q2.e0() << " " << q2.e1() << " " << q2.e2() << " " << q2.e3() << std::endl;
    }
}
