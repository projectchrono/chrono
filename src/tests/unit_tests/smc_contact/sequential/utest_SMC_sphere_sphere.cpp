// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Check collision detection and contact forces for interaction of two spheres,
// for different combinations of collision models (primitive, mesh, convex hull)
//
// =============================================================================

#include "gtest/gtest.h"

#define SMC_SEQUENTIAL
#include "../utest_SMC.h"

enum class ShapeType { PRIMITIVE, MESH, HULL };

// -----------------------------------------------------------------------------

std::string ShapeTypeName(ShapeType type) {
    switch (type) {
        case ShapeType::PRIMITIVE:
            return "PRIMITIVE";
        case ShapeType::MESH:
            return "MESH";
        case ShapeType::HULL:
            return "HULL";
        default:
            return "";
    }
}

std::shared_ptr<ChCollisionShape> CreateSphereShape(ShapeType type,
                                                    double radius,
                                                    std::shared_ptr<ChMaterialSurfaceSMC> mat) {
    std::shared_ptr<ChCollisionShape> shape;

    switch (type) {
        case ShapeType::PRIMITIVE: {
            shape = chrono_types::make_shared<ChCollisionShapeSphere>(mat, radius);
            break;
        }
        case ShapeType::HULL: {
            auto mesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
                GetChronoDataFile("models/sphere.obj"), false, true);
            mesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(radius));
            shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(mat, mesh->getCoordsVertices());
            break;
        }
        case ShapeType::MESH: {
            auto mesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
                GetChronoDataFile("models/sphere.obj"), false, true);
            mesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(radius));
            shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, mesh, false, false, 0.005);
            break;
        }
        default:
            break;
    }

    return shape;
}

// -----------------------------------------------------------------------------

// Test system parameterized by the two collision models
class SphereSphereTest : public ::testing::TestWithParam<std::tuple<ShapeType, ShapeType>> {
  protected:
    SphereSphereTest() {
        auto type1 = std::get<0>(GetParam());
        auto type2 = std::get<1>(GetParam());

        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  // Default 2e5
        float p_ratio = 0.3f;      // Default 0.3f
        float s_frict = 0.3f;      // Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.3f;      // Default 0.6f
        float roll_frict = 0.0f;   // Usually around 1E-3
        float spin_frict = 0.0f;   // Usually around 1E-3
        float cor_in = 0.5f;       // coefficient of restitution [0,1]
        float ad = 0.0f;           // Magnitude of the adhesion in the Constant adhesion model
        float adDMT = 0.0f;        // Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;     // Magnitude of the adhesion in the SPerko adhesion model

        auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
        mat->SetYoungModulus(y_modulus);
        mat->SetPoissonRatio(p_ratio);
        mat->SetSfriction(s_frict);
        mat->SetKfriction(k_frict);
        mat->SetRollingFriction(roll_frict);
        mat->SetSpinningFriction(spin_frict);
        mat->SetRestitution(cor_in);
        mat->SetAdhesion(ad);
        mat->SetAdhesionMultDMT(adDMT);
        mat->SetAdhesionSPerko(adSPerko);

        // Create a multicore SMC system and set the system parameters
        sys = new ChSystemSMC();
        SetSimParameters(sys, ChVector<>(0, 0, 0), ChSystemSMC::ContactForceModel::Hertz);

        // Add the spheres to the system
        radius = 0.5;
        double mass = 1.0;

        init_pos = 2 * radius;
        init_vel = 1.0;

        body1 = chrono_types::make_shared<ChBody>();
        body1->SetMass(mass);
        body1->SetPos(ChVector<>(+init_pos, 0, 0));
        body1->SetPos_dt(ChVector<>(-init_vel, 0, 0));
        body1->SetCollide(true);
        body1->AddCollisionShape(CreateSphereShape(type1, radius, mat));
        sys->AddBody(body1);

        body2 = chrono_types::make_shared<ChBody>();
        body2->SetMass(mass);
        body2->SetPos(ChVector<>(-init_pos, 0, 0));
        body2->SetPos_dt(ChVector<>(+init_vel, 0, 0));
        body2->SetCollide(true);
        body2->AddCollisionShape(CreateSphereShape(type2, radius, mat));
        sys->AddBody(body2);
    }

    ~SphereSphereTest() { delete sys; }

    ChSystemSMC* sys;
    std::shared_ptr<ChBody> body1;
    std::shared_ptr<ChBody> body2;
    double radius;
    double init_pos;
    double init_vel;
};

TEST_P(SphereSphereTest, impact) {
    double t_end = 2 * (init_pos - radius) / init_vel;
    double time_step = 3e-5;

    std::cout << ShapeTypeName(std::get<0>(GetParam())) << " - " << ShapeTypeName(std::get<1>(GetParam())) << std::endl;
    std::cout << "init pos/vel 1: " << body1->GetPos().x() << " " << body1->GetPos_dt().x() << std::endl;
    std::cout << "init pos/vel 2: " << body2->GetPos().x() << " " << body2->GetPos_dt().x() << std::endl;

    bool contact = false;
    while (sys->GetChTime() < t_end) {
        sys->DoStepDynamics(time_step);
        ////std::cout << sys->GetChTime() << " " << body1->GetPos().x() << "  " << body1->GetPos_dt().x() << std::endl;
        if (!contact && sys->GetNcontacts() > 0)
            contact = true;
    }

    std::cout << "final pos/vel 1: " << body1->GetPos().x() << " " << body1->GetPos_dt().x() << std::endl;
    std::cout << "final pos/vel 2: " << body2->GetPos().x() << " " << body2->GetPos_dt().x() << std::endl;

    ASSERT_TRUE(contact);
    ASSERT_GT(body1->GetPos().x(), radius + 0.1);
    ASSERT_LT(body2->GetPos().x(), radius - 0.1);
    ASSERT_GT(body1->GetPos_dt().x(), 0.0);
    ASSERT_LT(body2->GetPos_dt().x(), 0.0);
}

INSTANTIATE_TEST_SUITE_P(
    ChronoSequential,                                                                              //
    SphereSphereTest,                                                                              //
    ::testing::Combine(::testing::Values(ShapeType::PRIMITIVE, ShapeType::HULL, ShapeType::MESH),  //
                       ::testing::Values(ShapeType::PRIMITIVE, ShapeType::HULL, ShapeType::MESH))  //
);
