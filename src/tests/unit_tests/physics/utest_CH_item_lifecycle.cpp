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
//
// Unit tests for the initialization lifecycle of physics items.
// - each item runs SetupInitial and SetupInitialState once, and all items complete phase 1 before any starts phase 2
// - items added after the simulation started are initialized alone
// - adding a mesh or a node, or computing mesh mass properties, during a simulation does not re-record the reference
//   configuration of existing elements; ResetInitialization does
// - an item can set initial conditions in SetupInitialState, after all reference configurations are recorded
// - external dynamics items do not need an explicit call to Initialize
// - the active coordinates of a fixed FEA node are counted correctly when it is released
//
// =============================================================================

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChExternalDynamicsODE.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementCableANCF.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::fea;

// -----------------------------------------------------------------------------

// Physics item that records the calls to its initialization hooks in a shared log.
class RecorderItem : public ChPhysicsItem {
  public:
    RecorderItem(const std::string& name, std::vector<std::string>& log) : m_name(name), m_log(log) {}

    int num_phase1 = 0;
    int num_phase2 = 0;

  private:
    virtual void SetupInitial() override {
        num_phase1++;
        m_log.push_back(m_name + "1");
    }
    virtual void SetupInitialState() override {
        num_phase2++;
        m_log.push_back(m_name + "2");
    }

    std::string m_name;
    std::vector<std::string>& m_log;
};

// Physics item that pretensions a cable by moving its free end node in phase 2.
class Tensioner : public ChPhysicsItem {
  public:
    Tensioner(std::shared_ptr<ChNodeFEAxyzD> node, double dx) : m_node(node), m_dx(dx) {}

  private:
    virtual void SetupInitialState() override { m_node->SetPos(m_node->GetPos() + ChVector3d(m_dx, 0, 0)); }

    std::shared_ptr<ChNodeFEAxyzD> m_node;
    double m_dx;
};

// Van der Pol oscillator.
class VanDerPol : public ChExternalDynamicsODE {
  public:
    virtual VanDerPol* Clone() const override { return new VanDerPol(*this); }
    virtual unsigned int GetNumStates() const override { return 2; }
    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override {
        y0(0) = 2.0;
        y0(1) = 0.0;
    }
    virtual void CalculateRHS(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& rhs) override {
        rhs(0) = y(1);
        rhs(1) = (1 - y(0) * y(0)) * y(1) - y(0);
    }
};

// -----------------------------------------------------------------------------

// Cable cantilever under gravity.
struct Cantilever {
    Cantilever() {
        sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));
        sys.SetSolver(chrono_types::make_shared<ChSolverSparseLU>());
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

        mesh = chrono_types::make_shared<ChMesh>();
        auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
        section->SetDiameter(0.015);
        section->SetYoungModulus(0.01e9);
        ChBuilderCableANCF builder;
        builder.BuildBeam(mesh, section, 10, ChVector3d(0, 0, 0), ChVector3d(1, 0, 0));
        nodes = builder.GetLastBeamNodes();
        nodes.front()->SetFixed(true);
        sys.Add(mesh);
    }

    // Simulate 400 steps, calling the provided function before step 200. Return the final tip position.
    ChVector3d Run(std::function<void(Cantilever&)> action = nullptr) {
        for (int i = 0; i < 400; i++) {
            if (i == 200 && action)
                action(*this);
            sys.DoStepDynamics(1e-3);
        }
        return nodes.back()->GetPos();
    }

    ChSystemSMC sys;
    std::shared_ptr<ChMesh> mesh;
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> nodes;
};

static std::shared_ptr<ChNodeFEAxyz> FixedNode() {
    auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(5, 0, 0));
    node->SetFixed(true);
    return node;
}

// -----------------------------------------------------------------------------

TEST(ChPhysicsItemLifecycle, phases) {
    std::vector<std::string> log;
    ChSystemSMC sys;
    auto a = chrono_types::make_shared<RecorderItem>("A", log);
    auto b = chrono_types::make_shared<RecorderItem>("B", log);
    sys.Add(a);
    sys.Add(b);
    EXPECT_FALSE(a->IsInitialized());

    sys.DoStepDynamics(1e-3);
    sys.DoStepDynamics(1e-3);

    // Each phase runs once per item, and phase 1 completes for all items before phase 2 starts
    EXPECT_EQ(log, std::vector<std::string>({"A1", "B1", "A2", "B2"}));
    EXPECT_TRUE(a->IsInitialized());
    EXPECT_TRUE(b->IsInitialized());

    // An item added later is initialized alone
    auto c = chrono_types::make_shared<RecorderItem>("C", log);
    sys.Add(c);
    sys.DoStepDynamics(1e-3);
    EXPECT_EQ(log, std::vector<std::string>({"A1", "B1", "A2", "B2", "C1", "C2"}));

    // A reset item is initialized again, alone
    a->ResetInitialization();
    EXPECT_FALSE(a->IsInitialized());
    sys.DoStepDynamics(1e-3);
    EXPECT_EQ(a->num_phase1, 2);
    EXPECT_EQ(a->num_phase2, 2);
    EXPECT_EQ(b->num_phase1, 1);
    EXPECT_EQ(c->num_phase1, 1);
}

TEST(ChPhysicsItemLifecycle, mesh_reference) {
    auto ref = Cantilever().Run();

    // Repeated calls to Setup before the first step
    Cantilever setup;
    setup.sys.Setup();
    setup.sys.Setup();
    EXPECT_NEAR((setup.Run() - ref).Length(), 0, 1e-12);

    // Adding a mesh during the simulation leaves the existing mesh unchanged
    auto add_mesh = Cantilever().Run([](Cantilever& c) {
        auto mesh = chrono_types::make_shared<ChMesh>();
        mesh->AddNode(FixedNode());
        c.sys.Add(mesh);
    });
    EXPECT_NEAR((add_mesh - ref).Length(), 0, 1e-12);

    // Adding a node to the mesh during the simulation leaves the existing elements unchanged
    auto add_node = Cantilever().Run([](Cantilever& c) { c.mesh->AddNode(FixedNode()); });
    EXPECT_NEAR((add_node - ref).Length(), 0, 1e-12);

    // Computing the mesh mass properties during the simulation leaves the mesh unchanged
    auto mass_props = Cantilever().Run([](Cantilever& c) { c.mesh->ComputeMassProperties(); });
    EXPECT_NEAR((mass_props - ref).Length(), 0, 1e-12);

    // Resetting the mesh initialization records its current (deformed) shape as the new reference
    auto reset = Cantilever().Run([](Cantilever& c) { c.mesh->ResetInitialization(); });
    EXPECT_GT((reset - ref).Length(), 1e-3);
}

TEST(ChPhysicsItemLifecycle, initial_state) {
    Cantilever c;
    c.sys.SetGravitationalAcceleration(VNULL);

    // The tensioner moves the free end in phase 2, after the mesh recorded its reference configuration in phase 1
    auto tensioner = chrono_types::make_shared<Tensioner>(c.nodes.back(), 0.01);
    c.sys.Add(tensioner);

    c.sys.DoStepDynamics(1e-3);
    c.sys.DoStepDynamics(1e-3);

    // The stretched cable pulls its free end back
    EXPECT_LT(c.nodes.back()->GetPosDt().x(), -1e-3);
}

TEST(ChPhysicsItemLifecycle, external_dynamics) {
    auto run = [](bool call_initialize) {
        ChSystemSMC sys;
        auto ode = chrono_types::make_shared<VanDerPol>();
        if (call_initialize)
            ode->Initialize();
        sys.Add(ode);
        for (int i = 0; i < 100; i++)
            sys.DoStepDynamics(1e-2);
        EXPECT_TRUE(ode->IsInitialized());
        return ode->GetStates();
    };

    auto y_auto = run(false);
    auto y_init = run(true);
    ASSERT_EQ(y_auto.size(), 2);
    EXPECT_GT(std::abs(y_auto(0) - 2.0), 1e-3);
    EXPECT_NEAR((y_auto - y_init).norm(), 0, 1e-14);
}

TEST(ChPhysicsItemLifecycle, released_node) {
    Cantilever c;
    auto node = c.nodes.back();
    node->SetFixed(true);

    c.sys.DoStepDynamics(1e-3);
    auto num_fixed = c.sys.GetNumCoordsPosLevel();

    node->SetFixed(false);
    c.sys.DoStepDynamics(1e-3);
    EXPECT_EQ(c.sys.GetNumCoordsPosLevel(), num_fixed + node->GetNumCoordsPosLevelActive());
    EXPECT_EQ(node->GetNumCoordsPosLevelActive(), 6u);
}
