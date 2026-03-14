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
// Authors: Bocheng Zou
// =============================================================================
//
// Unit test to create a OpenCascade shape and verify if it's dynamics is correct.
// =============================================================================

#include <BRepPrimAPI_MakeCylinder.hxx>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_cascade/ChCascadeBodyEasy.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChVisualShapeCascade.h"


#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::cascade;

TEST(ChCascadeBodyEasy, CreateCylinder) {
    // Create the simulation system and add items
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Set the global collision margins before creating shapes
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);

    // A collision material, will be used by two colliding shapes
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction(0.5f);

    // Create OpenCascade cylinder
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(0.09, 0.1).Shape();

    // Use it to make a body with proper center of mass and inertia tensor
    // given the CAD shape. Also visualize it.
    auto vis_params = chrono_types::make_shared<ChCascadeTriangulate>(0.1, true, 0.5);

    auto body = chrono_types::make_shared<ChCascadeBodyEasy>(
        cylinder,   // the CAD shape
        1000,       // density
        vis_params, // visualization triangulation parameters
        true,       // collide?
        material    // collision material
    );
    sys.Add(body);

    // Create a large cube as a floor
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(
        1.0, 0.2, 1.0, // x y z size
        1000,          // density
        true,          // visualize?
        true,          // collide?
        material       // collision material
    );
    floor->SetPos(ChVector3d(0, -0.3, 0));
    floor->SetFixed(true);

    auto visual_shape = floor->GetVisualShape(0);
    if (visual_shape) {
        visual_shape->SetTexture(GetChronoDataFile("textures/blue.png"));
    }

    sys.Add(floor);

    // Run the simulation
    sys.SetSolverType(ChSolver::Type::PSOR);

    const ChVector3d expected_start_pos(0, 0, 0.05);
    const ChVector3d expected_end_pos(0, -0.11, 0.05);
    const double atol = 1e-3;

    ChVector3d start_pos = body->GetPos();
    EXPECT_NEAR(start_pos.x(), expected_start_pos.x(), atol);
    EXPECT_NEAR(start_pos.y(), expected_start_pos.y(), atol);
    EXPECT_NEAR(start_pos.z(), expected_start_pos.z(), atol);

    for (int i = 0; i < 64; ++i) {
        sys.DoStepDynamics(0.005);
    }

    ChVector3d end_pos = body->GetPos();
    EXPECT_NEAR(end_pos.x(), expected_end_pos.x(), atol);
    EXPECT_NEAR(end_pos.y(), expected_end_pos.y(), atol);
    EXPECT_NEAR(end_pos.z(), expected_end_pos.z(), atol);
}