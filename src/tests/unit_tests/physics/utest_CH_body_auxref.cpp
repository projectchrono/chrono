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
// Author: Radu Serban
// =============================================================================
//
// Unit test for consistency of the REF and COM frames of a ChBodyAuxRef when
// setting body position and velocity.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystemNSC.h"

using namespace chrono;

static const double tol = 1e-10;

static void TestVector(const ChVector3d& v1, const ChVector3d& v2) {
    ASSERT_NEAR(v1.x(), v2.x(), tol);
    ASSERT_NEAR(v1.y(), v2.y(), tol);
    ASSERT_NEAR(v1.z(), v2.z(), tol);
}

// Check that the stored REF frame is consistent with the COM frame (i.e., what ChBodyAuxRef::Update would compute).
static void CheckRefFrame(const ChBodyAuxRef& body) {
    ChFrameMoving<> expected = body.TransformLocalToParent(ChFrameMoving<>(body.GetFrameRefToCOM()));
    const auto& ref = body.GetFrameRefToAbs();
    TestVector(ref.GetPos(), expected.GetPos());
    TestVector(ref.GetPosDt(), expected.GetPosDt());
    TestVector(ref.GetAngVelParent(), expected.GetAngVelParent());
    TestVector(ref.GetAngVelParent(), body.GetAngVelParent());
}

// Test a case with REF at origin, COM offset along x, body spinning about z so that the REF point is at rest.
TEST(ChBodyAuxRef, velocity_setters) {
    ChSystemNSC sys;
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    sys.AddBody(body);

    body->SetFrameCOMToRef(ChFramed(ChVector3d(27.3, 0, 0)));
    body->SetFrameRefToAbs(ChFramed());
    body->SetLinVel(ChVector3d(0, 28.665, 0));
    body->SetAngVelParent(ChVector3d(0, 0, 1.05));

    CheckRefFrame(*body);
    TestVector(body->GetFrameRefToAbs().GetPosDt(), VNULL);

    sys.Setup();
    sys.Update(sys.GetChTime());
    CheckRefFrame(*body);
    TestVector(body->GetFrameRefToAbs().GetPosDt(), VNULL);
}

// Check that setting the body position after its velocity does not discard the REF frame velocity.
TEST(ChBodyAuxRef, position_after_velocity) {
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetFrameCOMToRef(ChFramed(ChVector3d(1, 2, 3), QuatFromAngleZ(0.3)));
    body->SetLinVel(ChVector3d(1, -2, 0.5));
    body->SetAngVelLocal(ChVector3d(0.2, -0.4, 1.1));
    CheckRefFrame(*body);

    body->SetFrameRefToAbs(ChFramed(ChVector3d(-1, 0, 4), QuatFromAngleX(0.7)));
    CheckRefFrame(*body);

    body->SetFrameCOMToAbs(ChFramed(ChVector3d(2, 1, 0), QuatFromAngleY(-0.2)));
    CheckRefFrame(*body);

    body->SetPos(ChVector3d(5, 5, 5));
    CheckRefFrame(*body);

    body->SetRot(QuatFromAngleZ(1.2));
    CheckRefFrame(*body);
}
