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
// Unit test for the removal of physics items with collision models.
//
// Removing an item from a system must also remove its collision models from the
// collision system, whatever the removal path (single item, all items of a kind,
// Clear, or removal from a sub-assembly). A collision model must also be detached
// from its implementation when the collision system is destroyed, so that the
// item can be used in another system (issue #845).
//
// =============================================================================

#include <memory>

#include "gtest/gtest.h"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

using namespace chrono;

// Two overlapping spheres, added to the given assembly (the system's own assembly by default).
struct Spheres {
    std::shared_ptr<ChBody> a;
    std::shared_ptr<ChBody> b;

    Spheres() {
        auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
        a = chrono_types::make_shared<ChBodyEasySphere>(0.5, 1000, false, true, mat);
        b = chrono_types::make_shared<ChBodyEasySphere>(0.5, 1000, false, true, mat);
        a->SetPos(ChVector3d(0, 0, 0));
        b->SetPos(ChVector3d(0.8, 0, 0));
    }

    bool Registered() const { return a->GetCollisionModel()->HasImplementation() || b->GetCollisionModel()->HasImplementation(); }
};

// System with the spheres, stepped once so that the collision system is initialized and the models registered.
struct Scene {
    ChSystemNSC sys;
    Spheres s;

    Scene() {
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        sys.AddBody(s.a);
        sys.AddBody(s.b);
        sys.DoStepDynamics(1e-3);
    }
};

// Check that the spheres collide in a new system, i.e., that they can be used again after removal.
void ExpectCollisionInNewSystem(const Spheres& s) {
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.AddBody(s.a);
    sys.AddBody(s.b);
    sys.DoStepDynamics(1e-3);
    EXPECT_EQ(sys.ComputeCollisions(), 1u);
}

TEST(ChSystemRemoveItems, remove_body) {
    Scene scene;
    ASSERT_EQ(scene.sys.ComputeCollisions(), 1u);

    scene.sys.RemoveBody(scene.s.a);
    scene.sys.RemoveBody(scene.s.b);

    EXPECT_FALSE(scene.s.Registered());
    EXPECT_EQ(scene.sys.ComputeCollisions(), 0u);
    ExpectCollisionInNewSystem(scene.s);
}

TEST(ChSystemRemoveItems, remove_all_bodies) {
    Scene scene;
    ASSERT_EQ(scene.sys.ComputeCollisions(), 1u);

    scene.sys.RemoveAllBodies();
    EXPECT_EQ(scene.sys.GetBodies().size(), 0u);
    EXPECT_FALSE(scene.s.Registered());
    EXPECT_EQ(scene.sys.ComputeCollisions(), 0u);
    ExpectCollisionInNewSystem(scene.s);
}

TEST(ChSystemRemoveItems, clear) {
    Scene scene;
    ASSERT_EQ(scene.sys.ComputeCollisions(), 1u);

    scene.sys.Clear();
    EXPECT_FALSE(scene.s.Registered());
    EXPECT_EQ(scene.sys.ComputeCollisions(), 0u);
    ExpectCollisionInNewSystem(scene.s);
}

TEST(ChSystemRemoveItems, sub_assembly) {
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    auto sub = chrono_types::make_shared<ChAssembly>();
    sys.AddOtherPhysicsItem(sub);
    Spheres s;
    sub->AddBody(s.a);
    sub->AddBody(s.b);
    sys.DoStepDynamics(1e-3);
    ASSERT_EQ(sys.ComputeCollisions(), 1u);

    // Removal of a single body from the sub-assembly
    sub->RemoveBody(s.a);
    EXPECT_FALSE(s.a->GetCollisionModel()->HasImplementation());
    EXPECT_EQ(sys.ComputeCollisions(), 0u);

    // Removal of all bodies from the sub-assembly
    sub->AddBody(s.a);
    sys.GetCollisionSystem()->BindItem(s.a);
    ASSERT_EQ(sys.ComputeCollisions(), 1u);
    sub->RemoveAllBodies();
    EXPECT_FALSE(s.Registered());
    EXPECT_EQ(sys.ComputeCollisions(), 0u);

    // Removal of the sub-assembly itself
    sub->AddBody(s.a);
    sub->AddBody(s.b);
    sys.GetCollisionSystem()->BindItem(sub);
    ASSERT_EQ(sys.ComputeCollisions(), 1u);
    sys.RemoveOtherPhysicsItem(sub);
    EXPECT_FALSE(s.Registered());
    EXPECT_EQ(sys.ComputeCollisions(), 0u);
}

// Bodies that outlive their system must have their collision models detached from the destroyed implementations.
void ExpectDetachedAfterDestruction(ChCollisionSystem::Type type) {
    Spheres s;
    {
        ChSystemNSC sys;
        sys.SetCollisionSystemType(type);
        sys.AddBody(s.a);
        sys.AddBody(s.b);
        sys.DoStepDynamics(1e-3);
        ASSERT_TRUE(s.Registered());
    }
    EXPECT_FALSE(s.Registered());
    ExpectCollisionInNewSystem(s);
}

TEST(ChSystemRemoveItems, destroyed_system) {
    ExpectDetachedAfterDestruction(ChCollisionSystem::Type::BULLET);
}

#ifdef CHRONO_COLLISION
TEST(ChSystemRemoveItems, multicore_destruction) {
    // The Multicore collision system does not support the removal of collision models. Destroying a system that uses it
    // must still work (it does not remove the models one by one) and detach the models of the bodies that outlive it.
    ExpectDetachedAfterDestruction(ChCollisionSystem::Type::MULTICORE);
}
#endif
