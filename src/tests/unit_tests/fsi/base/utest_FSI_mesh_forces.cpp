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
// Unit test for the application of fluid forces on FSI FEA mesh nodes through the
// generic FSI interface:
// - fluid forces are applied through node accumulators, in addition to (and not
//   replacing) the forces set on the nodes with SetForce;
// - forces for 1-D meshes are applied to the nodes of 1-D meshes and forces for
//   2-D meshes are applied to the nodes of 2-D meshes;
// - fluid forces are replaced (not accumulated) at each exchange;
// - FSI meshes are indexed in their respective lists.
//
// The fluid solver is a stub that does no work and reports prescribed node forces,
// so the test needs neither a GPU nor any particular fluid solver module.
//
// =============================================================================

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChNodeFEAxyz.h"

#include "chrono_fsi/ChFsiFluidSystem.h"
#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/ChFsiSystem.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;

// Prescribed fluid forces on the nodes of the FSI meshes (index-based node ordering).
ChVector3d Force1D(size_t imesh, size_t inode) {
    return ChVector3d(1.0 + inode, 10.0 * (imesh + 1), 0.0);
}

ChVector3d Force2D(size_t imesh, size_t inode) {
    return ChVector3d(0.0, 1.0 + inode, 100.0 * (imesh + 1));
}

// Fluid system that does nothing and reports prescribed forces on FSI mesh nodes.
class StubFluidSystem : public ChFsiFluidSystem {
  public:
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) override {}
    virtual void OnDoStepDynamics(double time, double step) override {}
    virtual void OnExchangeSolidForces() override {}
    virtual void OnExchangeSolidStates() override {}
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states) override {}
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces) override {}
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                 const std::vector<FsiMeshState>& mesh1D_states,
                                 const std::vector<FsiMeshState>& mesh2D_states) override {}
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces, std::vector<FsiMeshForce>& mesh1D_forces, std::vector<FsiMeshForce>& mesh2D_forces) override {
        for (auto& f : body_forces) {
            f.force = VNULL;
            f.torque = VNULL;
        }
        for (size_t imesh = 0; imesh < mesh1D_forces.size(); imesh++)
            for (size_t inode = 0; inode < mesh1D_forces[imesh].force.size(); inode++)
                mesh1D_forces[imesh].force[inode] = Force1D(imesh, inode);
        for (size_t imesh = 0; imesh < mesh2D_forces.size(); imesh++)
            for (size_t inode = 0; inode < mesh2D_forces[imesh].force.size(); inode++)
                mesh2D_forces[imesh].force[inode] = Force2D(imesh, inode);
    }

  protected:
    virtual void Initialize(const std::vector<FsiBodyState>& body_states) override { m_is_initialized = true; }
    virtual void Initialize(const std::vector<FsiBodyState>& body_states, const std::vector<FsiMeshState>& mesh1D_states, const std::vector<FsiMeshState>& mesh2D_states) override {
        m_is_initialized = true;
    }
};

// Multibody advance that does nothing (the multibody state is not changed).
class StubMBDCallback : public ChFsiSystem::MBDCallback {
  public:
    virtual void Advance(double step, double threshold) override {}
};

// FSI system coupling the stub fluid system and a multibody system with one rigid body, one FEA mesh of cable
// elements (1-D FSI mesh), and one FEA mesh of tetrahedral elements (2-D FSI mesh).
class StubFsiSystem : public ChFsiSystem {
  public:
    StubFsiSystem() : ChFsiSystem(&m_mbs, &m_cfd) {
        m_fsi_interface = chrono_types::make_shared<ChFsiInterfaceGeneric>(&m_mbs, &m_cfd);
        SetCouplingScheme(CouplingScheme::SEQUENTIAL);
        SetVerbose(false);
        SetStepSizeCFD(1e-3);
        SetStepsizeMBD(1e-3);
        RegisterMBDCallback(chrono_types::make_shared<StubMBDCallback>());

        // Rigid body (added first, so that FSI mesh indices differ from the number of FSI bodies)
        auto body = chrono_types::make_shared<ChBody>();
        m_mbs.AddBody(body);
        AddRigidBody(body, nullptr, false);

        // FEA mesh with cable elements
        auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
        section->SetDiameter(0.02);
        section->SetYoungModulus(1e6);
        section->SetDensity(1000);
        mesh1D = chrono_types::make_shared<ChMesh>();
        ChBuilderCableANCF builder;
        builder.BuildBeam(mesh1D, section, 3, ChVector3d(0, 0, 0), ChVector3d(1, 0, 0));
        m_mbs.Add(mesh1D);

        // FEA mesh with one tetrahedral element
        auto material = chrono_types::make_shared<ChContinuumElastic>();
        material->SetYoungModulus(1e6);
        material->SetPoissonRatio(0.3);
        material->SetDensity(1000);
        mesh2D = chrono_types::make_shared<ChMesh>();
        auto n0 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 1));
        auto n1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(1, 0, 1));
        auto n2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 1, 1));
        auto n3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 2));
        mesh2D->AddNode(n0);
        mesh2D->AddNode(n1);
        mesh2D->AddNode(n2);
        mesh2D->AddNode(n3);
        auto tet = chrono_types::make_shared<ChElementTetraCorot_4>();
        tet->SetNodes(n0, n1, n2, n3);
        tet->SetMaterial(material);
        mesh2D->AddElement(tet);
        m_mbs.Add(mesh2D);

        fsi_mesh1D = AddFeaMesh1D(mesh1D, false);
        fsi_mesh2D = AddFeaMesh2D(mesh2D, false);

        Initialize();
    }

    std::shared_ptr<ChMesh> mesh1D;
    std::shared_ptr<ChMesh> mesh2D;
    std::shared_ptr<FsiMesh1D> fsi_mesh1D;
    std::shared_ptr<FsiMesh2D> fsi_mesh2D;

  private:
    ChSystemNSC m_mbs;
    StubFluidSystem m_cfd;
};

// Return the applied force on the given node (position-level entries of the node residual).
ChVector3d NodeForce(ChNodeFEAxyz& node) {
    ChVectorDynamic<> R(node.GetNumCoordsVelLevel());
    R.setZero();
    node.NodeIntLoadResidual_F(0, R, 1.0);
    return ChVector3d(R(0), R(1), R(2));
}

void CheckForce(const ChVector3d& actual, const ChVector3d& expected) {
    ASSERT_NEAR(actual.x(), expected.x(), 1e-12);
    ASSERT_NEAR(actual.y(), expected.y(), 1e-12);
    ASSERT_NEAR(actual.z(), expected.z(), 1e-12);
}

TEST(FsiMeshForces, indices) {
    StubFsiSystem sys;
    EXPECT_EQ(sys.fsi_mesh1D->index, 0u);
    EXPECT_EQ(sys.fsi_mesh2D->index, 0u);
    EXPECT_EQ(sys.fsi_mesh1D->GetNumNodes(), 4u);
    EXPECT_EQ(sys.fsi_mesh2D->GetNumNodes(), 4u);
}

TEST(FsiMeshForces, application) {
    StubFsiSystem sys;

    // User-specified forces on all FSI mesh nodes
    auto user_force = [](int inode) { return ChVector3d(-0.5 * inode, 0.25, 3.0); };
    for (const auto& [inode, node] : sys.fsi_mesh1D->ind2ptr_map)
        node->SetForce(user_force(inode));
    for (const auto& [inode, node] : sys.fsi_mesh2D->ind2ptr_map)
        node->SetForce(user_force(inode));

    // Take several steps (each with a force exchange); fluid forces must not accumulate over steps
    for (int i = 0; i < 3; i++) {
        sys.DoStepDynamics(1e-3);

        for (const auto& [inode, node] : sys.fsi_mesh1D->ind2ptr_map)
            CheckForce(NodeForce(*node), user_force(inode) + Force1D(0, inode));
        for (const auto& [inode, node] : sys.fsi_mesh2D->ind2ptr_map)
            CheckForce(NodeForce(*node), user_force(inode) + Force2D(0, inode));
    }

    // The node forces set by the user are not modified
    for (const auto& [inode, node] : sys.fsi_mesh1D->ind2ptr_map)
        CheckForce(node->GetForce(), user_force(inode));
    for (const auto& [inode, node] : sys.fsi_mesh2D->ind2ptr_map)
        CheckForce(node->GetForce(), user_force(inode));
}
