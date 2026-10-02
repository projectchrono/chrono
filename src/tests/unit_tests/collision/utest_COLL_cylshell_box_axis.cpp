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
// Regression for the local-Z cylindrical shell axis in Bullet shell-box contact.
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/collision/ChCollisionShapeCylindricalShell.h"
#include "chrono/physics/ChSystemNSC.h"

using namespace chrono;

TEST(BulletCollision, CylindricalShellBoxLocalZAxis) {
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // ChSystem construction resets these defaults. Use nominal geometry.
    ChCollisionModel::SetDefaultSuggestedEnvelope(0);
    ChCollisionModel::SetDefaultSuggestedMargin(0);

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    floor->SetPos(ChVector3d(0, 0, -0.1));
    floor->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(mat, 2, 2, 0.2));
    floor->EnableCollision(true);
    sys.AddBody(floor);

    auto shell = chrono_types::make_shared<ChBody>();
    shell->SetPos(ChVector3d(0, 0, 0.070 - 0.00015));
    // Local Z is the public shell axis; rotate it onto the horizontal world Y axis.
    shell->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCylindricalShell>(mat, 0.070, 0.030),
                             ChFramed(VNULL, QuatFromAngleX(-CH_PI / 2)));
    shell->EnableCollision(true);
    sys.AddBody(shell);

    sys.GetCollisionSystem()->Initialize();
    sys.GetCollisionSystem()->BindAll();
    EXPECT_EQ(shell->GetCollisionModel()->GetEnvelope(), 0);
    EXPECT_EQ(floor->GetCollisionModel()->GetEnvelope(), 0);
    EXPECT_GT(sys.ComputeCollisions(), 0u);
}
