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
// Authors: Kostiantyn Dementiev
// =============================================================================
//
// Unit test for Bullet ray casts against a connected triangle mesh.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <limits>

#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystemSMC.h"

#include "gtest/gtest.h"

using namespace chrono;

// -----------------------------------------------------------------------------

const double half_side = 0.1;  // cube of side 0.2 m

// Add a fixed, rotated cube mesh
static std::shared_ptr<ChBody> AddCube(ChSystemSMC& sys, double envelope, double radius) {
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    const double a = half_side;
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    for (auto v : {ChVector3d(-a, -a, -a), ChVector3d(a, -a, -a), ChVector3d(a, a, -a), ChVector3d(-a, a, -a),  //
                   ChVector3d(-a, -a, a), ChVector3d(a, -a, a), ChVector3d(a, a, a), ChVector3d(-a, a, a)})
        mesh->GetCoordsVertices().push_back(v);
    for (auto f : {ChVector3i(0, 2, 1), ChVector3i(0, 3, 2), ChVector3i(4, 5, 6), ChVector3i(4, 6, 7),  //
                   ChVector3i(0, 1, 5), ChVector3i(0, 5, 4), ChVector3i(1, 2, 6), ChVector3i(1, 6, 5),  //
                   ChVector3i(2, 3, 7), ChVector3i(2, 7, 6), ChVector3i(3, 0, 4), ChVector3i(3, 4, 7)})
        mesh->GetIndicesVertices().push_back(f);

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    auto model = chrono_types::make_shared<ChCollisionModel>();
    model->SetEnvelope((float)envelope);
    model->AddShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, mesh, false, false, radius));

    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->SetRot(QuatFromAngleZ(CH_PI / 6) * QuatFromAngleY(CH_PI / 12));  // turned 30 deg, tilted 15 deg
    body->AddCollisionModel(model);
    body->EnableCollision(true);
    sys.AddBody(body);
    sys.DoStepDynamics(1e-3);

    return body;
}

// Lowest crossing of the vertical segment (x, y, z0)-(x, y, z1) with the cube faces, or NaN
static double ExactLowestHit(const ChBody& body, double x, double y, double z0, double z1) {
    const double a = half_side;
    const ChVector3d o = body.TransformPointParentToLocal(ChVector3d(x, y, z0));
    const ChVector3d d = body.TransformDirectionParentToLocal(ChVector3d(0, 0, z1 - z0));
    double tmin = 0, tmax = 1;
    for (int i = 0; i < 3; i++) {
        if (std::abs(d[i]) < 1e-14) {
            if (std::abs(o[i]) > a)
                return std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        double t1 = (-a - o[i]) / d[i];
        double t2 = (a - o[i]) / d[i];
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));
    }
    if (tmin > tmax)
        return std::numeric_limits<double>::quiet_NaN();
    return z0 + tmin * (z1 - z0);
}

// Distance from a point (absolute frame) to the solid cube
static double DistanceToCube(const ChBody& body, const ChVector3d& p) {
    const double a = half_side;
    const ChVector3d q = body.TransformPointParentToLocal(p);
    const ChVector3d c(std::clamp(q.x(), -a, a), std::clamp(q.y(), -a, a), std::clamp(q.z(), -a, a));
    return (q - c).Length();
}

// -----------------------------------------------------------------------------

TEST(ChCollisionSystemBullet, RayHitConnectedMesh) {
    ChSystemSMC sys;
    auto body = AddCube(sys, 0.0, 0.0);
    auto coll_sys = sys.GetCollisionSystem();

    int num_hits = 0;
    for (int i = 0; i <= 160; i++) {
        double x = -0.2 + 0.4 * i / 160;
        double y = 0.01;
        ChVector3d from(x, y, -0.5);
        ChVector3d to(x, y, 0.5);
        double z_exact = ExactLowestHit(*body, x, y, -0.5, 0.5);

        ChCollisionSystem::ChRayhitResult r1;
        ChCollisionSystem::ChRayhitResult r2;
        coll_sys->RayHit(from, to, r1);
        coll_sys->RayHit(from, to, body->GetCollisionModel().get(), r2);

        ASSERT_EQ(r1.hit, !std::isnan(z_exact)) << "x = " << x;
        ASSERT_EQ(r2.hit, !std::isnan(z_exact)) << "x = " << x;
        if (!r1.hit)
            continue;
        num_hits++;
        EXPECT_NEAR(r1.abs_hitPoint.z(), z_exact, 1e-5) << "x = " << x;
        EXPECT_NEAR(r2.abs_hitPoint.z(), z_exact, 1e-5) << "x = " << x;
        EXPECT_LT(r1.abs_hitNormal.z(), 0) << "x = " << x;
    }
    EXPECT_EQ(num_hits, 113);
}

TEST(ChCollisionSystemBullet, RayHitConnectedMeshMargin) {
    const double envelope = 0.02;
    const double radius = 0.005;

    ChSystemSMC sys;
    auto body = AddCube(sys, envelope, radius);
    auto coll_sys = sys.GetCollisionSystem();

    int num_hits = 0;
    for (int i = 0; i <= 160; i++) {
        double x = -0.2 + 0.4 * i / 160;
        double y = 0.01;
        ChVector3d from(x, y, -0.5);
        ChVector3d to(x, y, 0.5);

        ChCollisionSystem::ChRayhitResult r;
        coll_sys->RayHit(from, to, r);
        if (!std::isnan(ExactLowestHit(*body, x, y, -0.5, 0.5)))
            ASSERT_TRUE(r.hit) << "x = " << x;
        if (!r.hit)
            continue;
        num_hits++;

        // RayHit moves the hit point back by the envelope
        ChVector3d p = r.abs_hitPoint + r.abs_hitNormal * envelope;
        EXPECT_NEAR(p.x(), x, 1e-6) << "x = " << x;
        EXPECT_NEAR(p.y(), y, 1e-6) << "x = " << x;
        EXPECT_NEAR(DistanceToCube(*body, p), envelope + radius, 1e-5) << "x = " << x;
        EXPECT_LT(r.abs_hitNormal.z(), 0) << "x = " << x;
    }
    EXPECT_GT(num_hits, 113);
}

TEST(ChCollisionSystemBullet, RayHitConnectedMeshSharedEdges) {
    // Rays with x = y pass exactly through edges shared by two triangles
    const int n = 16;
    const double h = 2.0 / n;
    auto height = [](double x, double y) { return 0.05 * std::sin(3 * x) * std::cos(2 * y) + 0.02 * x; };

    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    for (int j = 0; j <= n; j++)
        for (int i = 0; i <= n; i++) {
            double x = -1 + i * h;
            double y = -1 + j * h;
            mesh->GetCoordsVertices().push_back(ChVector3d(x, y, height(x, y)));
        }
    for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++) {
            int v00 = j * (n + 1) + i;
            int v10 = v00 + 1;
            int v01 = v00 + n + 1;
            int v11 = v01 + 1;
            mesh->GetIndicesVertices().push_back(ChVector3i(v00, v10, v11));
            mesh->GetIndicesVertices().push_back(ChVector3i(v00, v11, v01));
        }

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    auto model = chrono_types::make_shared<ChCollisionModel>();
    model->SetEnvelope(0);
    model->AddShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, mesh, true, false, 0.0));
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->AddCollisionModel(model);
    body->EnableCollision(true);
    sys.AddBody(body);
    sys.DoStepDynamics(1e-3);

    for (int k = 0; k <= 200; k++) {
        double t = -0.995 + 1.99 * k / 200;
        ChCollisionSystem::ChRayhitResult r;
        sys.GetCollisionSystem()->RayHit(ChVector3d(t, t, 1), ChVector3d(t, t, -1), r);
        ASSERT_TRUE(r.hit) << "t = " << t;

        int i = std::min(n - 1, (int)std::floor((t + 1) / h));
        double x0 = -1 + i * h;
        double z_exact = height(x0, x0) + (t - x0) / h * (height(x0 + h, x0 + h) - height(x0, x0));
        EXPECT_NEAR(r.abs_hitPoint.z(), z_exact, 1e-5) << "t = " << t;
    }
}
