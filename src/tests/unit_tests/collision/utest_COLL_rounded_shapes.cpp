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
// Unit test for the rounded cylinder and rounded box shapes.
//
// Checks that the geometry (volume, gyration, bounding volumes) and the collision
// systems (Bullet and Multicore) consistently interpret the shape dimensions as
// the outer dimensions of the sphere-swept solid.
//
// The test can also be run manually with run-time visualization (if Chrono::VSG
// is available), in which case the probes used in the collision tests are shown
// (placed in contact with the rounded shapes, i.e., with zero gap) together with
// the collision shapes and the contact normals:
//    utest_COLL_rounded_shapes --vis [bullet|multicore]
//
// =============================================================================

#include <algorithm>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChCylinder.h"
#include "chrono/geometry/ChRoundedBox.h"
#include "chrono/geometry/ChRoundedCylinder.h"
#include "chrono/geometry/ChSphere.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "gtest/gtest.h"

using namespace chrono;

// -----------------------------------------------------------------------------

// Rounded cylinder dimensions (outer radius, outer height, sweeping sphere radius)
const double cyl_radius = 0.8;
const double cyl_height = 1.5;
const double cyl_srad = 0.25;

// Rounded box dimensions (outer lengths, sweeping sphere radius)
const ChVector3d box_size(1.0, 0.6, 1.4);
const double box_srad = 0.2;

// Probe dimensions (sphere radius and box half-length) and separation from the rounded shape.
// The tests use a positive gap (separated shapes reported as contacts because they are within the collision envelope);
// the run-time visualization uses a zero gap so that the shapes are shown touching.
const double probe_size = 0.1;
const double test_gap = 0.02;
const double vis_gap = 0.0;

// Collision envelope and margin (the envelope must be larger than the gap for contacts to be reported)
const double envelope = 0.05;
const double margin = 0.001;

// -----------------------------------------------------------------------------

enum class RoundedShape { CYLINDER, BOX };
enum class ProbeShape { SPHERE, BOX };

// A probe placed at a specified distance from the surface of a rounded shape.
// Position and normal are expressed in the frame of the rounded shape.
struct ProbeCase {
    std::string name;
    RoundedShape shape;
    ProbeShape probe;
    ChVector3d pos;     // probe center
    ChVector3d normal;  // outward unit normal to the rounded shape at the closest point
};

std::vector<ProbeCase> GetProbeCases(double gap) {
    std::vector<ProbeCase> cases;

    // Rounded cylinder (axis along Z)
    {
        double a = cyl_radius - cyl_srad;      // inner cylinder radius
        double c = cyl_height / 2 - cyl_srad;  // inner cylinder half-height
        double d = cyl_srad + probe_size + gap;
        ChVector3d n_side(1, 0, 0);
        ChVector3d n_cap(0, 0, 1);
        ChVector3d n_edge = ChVector3d(1, 0, 1).GetNormalized();
        ChVector3d n_rad = ChVector3d(-1, 1, 0).GetNormalized();
        ChVector3d n_rim = (n_rad + ChVector3d(0, 0, -1.5)).GetNormalized();

        cases.push_back({"cyl_side_sphere", RoundedShape::CYLINDER, ProbeShape::SPHERE,
                         ChVector3d(cyl_radius + probe_size + gap, 0, 0), n_side});
        cases.push_back({"cyl_cap_sphere", RoundedShape::CYLINDER, ProbeShape::SPHERE,
                         ChVector3d(0, 0, cyl_height / 2 + probe_size + gap), n_cap});
        cases.push_back({"cyl_edge_sphere", RoundedShape::CYLINDER, ProbeShape::SPHERE, ChVector3d(a, 0, c) + d * n_edge, n_edge});
        cases.push_back({"cyl_rim_sphere", RoundedShape::CYLINDER, ProbeShape::SPHERE, a * n_rad - ChVector3d(0, 0, c) + d * n_rim, n_rim});
        cases.push_back({"cyl_side_box", RoundedShape::CYLINDER, ProbeShape::BOX,
                         ChVector3d(cyl_radius + probe_size + gap, 0, 0), n_side});
        cases.push_back({"cyl_cap_box", RoundedShape::CYLINDER, ProbeShape::BOX,
                         ChVector3d(0, 0, cyl_height / 2 + probe_size + gap), n_cap});
    }

    // Rounded box
    {
        ChVector3d e = box_size / 2 - ChVector3d(box_srad);  // inner box half-lengths
        double d = box_srad + probe_size + gap;
        ChVector3d n_face(1, 0, 0);
        ChVector3d n_edge = ChVector3d(1, 1, 0).GetNormalized();
        ChVector3d n_corner = ChVector3d(1, 1, 1).GetNormalized();

        cases.push_back({"box_face_sphere", RoundedShape::BOX, ProbeShape::SPHERE,
                         ChVector3d(box_size.x() / 2 + probe_size + gap, 0, 0), n_face});
        cases.push_back({"box_edge_sphere", RoundedShape::BOX, ProbeShape::SPHERE, ChVector3d(e.x(), e.y(), 0) + d * n_edge, n_edge});
        cases.push_back({"box_corner_sphere", RoundedShape::BOX, ProbeShape::SPHERE, e + d * n_corner, n_corner});
        cases.push_back({"box_face_box", RoundedShape::BOX, ProbeShape::BOX,
                         ChVector3d(box_size.x() / 2 + probe_size + gap, 0, 0), n_face});
        cases.push_back({"box_top_box", RoundedShape::BOX, ProbeShape::BOX,
                         ChVector3d(0, 0, box_size.z() / 2 + probe_size + gap), ChVector3d(0, 0, 1)});
    }

    return cases;
}

std::shared_ptr<ChVisualMaterial> VisMaterial(const ChColor& color) {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor(color);
    return mat;
}

// Add a body with the specified rounded shape (fixed) at the given frame.
std::shared_ptr<ChBody> AddRoundedBody(ChSystem& sys, RoundedShape shape, const ChFrame<>& frame) {
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->SetPos(frame.GetPos());
    body->SetRot(frame.GetRot());
    switch (shape) {
        case RoundedShape::CYLINDER:
            utils::AddRoundedCylinderGeometry(body.get(), mat, cyl_radius, cyl_height, cyl_srad, VNULL, QUNIT, true,
                                              VisMaterial(ChColor(0.6f, 0.3f, 0.3f)));
            break;
        case RoundedShape::BOX:
            utils::AddRoundedBoxGeometry(body.get(), mat, box_size, box_srad, VNULL, QUNIT, true,
                                         VisMaterial(ChColor(0.3f, 0.3f, 0.6f)));
            break;
    }
    body->EnableCollision(true);
    sys.AddBody(body);
    return body;
}

// Add a free probe body at the location specified by the given case, relative to the given frame.
std::shared_ptr<ChBody> AddProbeBody(ChSystem& sys, const ProbeCase& pc, const ChFrame<>& frame) {
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto body = chrono_types::make_shared<ChBody>();
    // Align the box probe with the contact normal
    ChQuaterniond rot = QuatFromVec2Vec(ChVector3d(0, 0, 1), pc.normal);
    ChFrame<> probe_frame = frame * ChFrame<>(pc.pos, rot);
    body->SetPos(probe_frame.GetPos());
    body->SetRot(probe_frame.GetRot());
    switch (pc.probe) {
        case ProbeShape::SPHERE:
            utils::AddSphereGeometry(body.get(), mat, probe_size, VNULL, QUNIT, true, VisMaterial(ChColor(0.3f, 0.6f, 0.3f)));
            break;
        case ProbeShape::BOX:
            utils::AddBoxGeometry(body.get(), mat, ChVector3d(2 * probe_size), VNULL, QUNIT, true,
                                  VisMaterial(ChColor(0.3f, 0.6f, 0.3f)));
            break;
    }
    body->EnableCollision(true);
    sys.AddBody(body);
    return body;
}

void SetupSystem(ChSystemNSC& sys, ChCollisionSystem::Type type) {
    sys.SetCollisionSystemType(type);
    sys.SetGravitationalAcceleration(VNULL);
    ChCollisionModel::SetDefaultSuggestedEnvelope(envelope);
    ChCollisionModel::SetDefaultSuggestedMargin(margin);
}

// Collect the contact with the smallest distance.
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
        num_contacts++;
        if (distance < min_distance) {
            min_distance = distance;
            normal = plane_coord.GetAxisX();
        }
        return true;
    }

    int num_contacts = 0;
    double min_distance = std::numeric_limits<double>::max();
    ChVector3d normal;
};

// -----------------------------------------------------------------------------

// Evaluate volume and gyration matrix (diagonal) of a solid by midpoint integration on a regular grid.
void Integrate(const std::function<bool(const ChVector3d&)>& inside, const ChVector3d& hlen, int n, double& volume, ChVector3d& gyr) {
    ChVector3d h = (2.0 / n) * hlen;
    double cell = h.x() * h.y() * h.z();
    double count = 0;
    ChVector3d s2(0);
    for (int i = 0; i < n; i++) {
        double x = -hlen.x() + (i + 0.5) * h.x();
        for (int j = 0; j < n; j++) {
            double y = -hlen.y() + (j + 0.5) * h.y();
            for (int k = 0; k < n; k++) {
                double z = -hlen.z() + (k + 0.5) * h.z();
                if (inside(ChVector3d(x, y, z))) {
                    count += 1;
                    s2 += ChVector3d(x * x, y * y, z * z);
                }
            }
        }
    }
    volume = count * cell;
    s2 /= count;
    gyr = ChVector3d(s2.y() + s2.z(), s2.z() + s2.x(), s2.x() + s2.y());
}

TEST(RoundedShapes, cylinder_mass_properties) {
    ChRoundedCylinder cyl(cyl_radius, cyl_height, cyl_srad);
    auto J = cyl.GetGyration();

    double a = cyl_radius - cyl_srad;
    double c = cyl_height / 2 - cyl_srad;
    auto inside = [&](const ChVector3d& p) {
        double du = std::max(std::sqrt(p.x() * p.x() + p.y() * p.y()) - a, 0.0);
        double dw = std::max(std::abs(p.z()) - c, 0.0);
        return du * du + dw * dw <= cyl_srad * cyl_srad;
    };
    double volume;
    ChVector3d gyr;
    Integrate(inside, ChVector3d(cyl_radius, cyl_radius, cyl_height / 2), 200, volume, gyr);

    EXPECT_NEAR(cyl.GetVolume(), volume, 1e-3 * volume);
    EXPECT_NEAR(J(0, 0), gyr.x(), 1e-3 * gyr.x());
    EXPECT_NEAR(J(1, 1), gyr.y(), 1e-3 * gyr.y());
    EXPECT_NEAR(J(2, 2), gyr.z(), 1e-3 * gyr.z());
    EXPECT_EQ(J(0, 1), 0.0);
    EXPECT_EQ(J(0, 2), 0.0);
    EXPECT_EQ(J(1, 2), 0.0);

    // No rounding: same as a cylinder
    EXPECT_NEAR(ChRoundedCylinder::CalcVolume(0.8, 1.5, 0), ChCylinder::CalcVolume(0.8, 1.5), 1e-12);
    EXPECT_TRUE(ChRoundedCylinder::CalcGyration(0.8, 1.5, 0).isApprox(ChCylinder::CalcGyration(0.8, 1.5), 1e-12));

    // Full rounding: same as a sphere
    EXPECT_NEAR(ChRoundedCylinder::CalcVolume(0.8, 1.6, 0.8), ChSphere::CalcVolume(0.8), 1e-12);
    EXPECT_TRUE(ChRoundedCylinder::CalcGyration(0.8, 1.6, 0.8).isApprox(ChSphere::CalcGyration(0.8), 1e-12));
}

TEST(RoundedShapes, box_mass_properties) {
    ChRoundedBox box(box_size, box_srad);
    auto J = box.GetGyration();

    ChVector3d e = box_size / 2 - ChVector3d(box_srad);
    auto inside = [&](const ChVector3d& p) {
        ChVector3d d(std::max(std::abs(p.x()) - e.x(), 0.0), std::max(std::abs(p.y()) - e.y(), 0.0),
                     std::max(std::abs(p.z()) - e.z(), 0.0));
        return d.Length2() <= box_srad * box_srad;
    };
    double volume;
    ChVector3d gyr;
    Integrate(inside, box_size / 2, 200, volume, gyr);

    EXPECT_NEAR(box.GetVolume(), volume, 1e-3 * volume);
    EXPECT_NEAR(J(0, 0), gyr.x(), 1e-3 * gyr.x());
    EXPECT_NEAR(J(1, 1), gyr.y(), 1e-3 * gyr.y());
    EXPECT_NEAR(J(2, 2), gyr.z(), 1e-3 * gyr.z());

    // No rounding: same as a box
    EXPECT_NEAR(ChRoundedBox::CalcVolume(box_size, 0), ChBox::CalcVolume(box_size), 1e-12);
    EXPECT_TRUE(ChRoundedBox::CalcGyration(box_size, 0).isApprox(ChBox::CalcGyration(box_size), 1e-12));

    // Full rounding: same as a sphere
    EXPECT_NEAR(ChRoundedBox::CalcVolume(ChVector3d(1.2), 0.6), ChSphere::CalcVolume(0.6), 1e-12);
    EXPECT_TRUE(ChRoundedBox::CalcGyration(ChVector3d(1.2), 0.6).isApprox(ChSphere::CalcGyration(0.6), 1e-12));
}

TEST(RoundedShapes, bounding_volumes) {
    ChRoundedCylinder cyl(cyl_radius, cyl_height, cyl_srad);
    auto cyl_bbox = cyl.GetBoundingBox();
    ASSERT_TRUE(cyl_bbox.min.Equals(ChVector3d(-cyl_radius, -cyl_radius, -cyl_height / 2), 1e-12));
    ASSERT_TRUE(cyl_bbox.max.Equals(ChVector3d(+cyl_radius, +cyl_radius, +cyl_height / 2), 1e-12));
    double a = cyl_radius - cyl_srad;
    double c = cyl_height / 2 - cyl_srad;
    ASSERT_NEAR(cyl.GetBoundingSphereRadius(), std::sqrt(a * a + c * c) + cyl_srad, 1e-12);

    ChRoundedBox box(box_size, box_srad);
    auto box_bbox = box.GetBoundingBox();
    ASSERT_TRUE(box_bbox.min.Equals(-box_size / 2, 1e-12));
    ASSERT_TRUE(box_bbox.max.Equals(+box_size / 2, 1e-12));
    ASSERT_NEAR(box.GetBoundingSphereRadius(), (box_size / 2 - ChVector3d(box_srad)).Length() + box_srad, 1e-12);
}

TEST(RoundedShapes, box_constructors) {
    ChRoundedBox box1(box_size.x(), box_size.y(), box_size.z(), box_srad);
    ASSERT_TRUE(box1.GetLengths().Equals(box_size, 1e-12));
    ASSERT_EQ(box1.GetSphereRadius(), box_srad);

    ChRoundedBox box2(box1);
    ASSERT_TRUE(box2.GetLengths().Equals(box_size, 1e-12));
    ASSERT_EQ(box2.GetSphereRadius(), box_srad);
}

// -----------------------------------------------------------------------------

void TestContacts(ChCollisionSystem::Type type, double tolerance) {
    // Test with the rounded shape in its reference configuration and in a rotated configuration
    std::vector<ChFrame<>> frames = {
        ChFrame<>(VNULL, QUNIT),
        ChFrame<>(ChVector3d(0.3, -0.2, 0.5), QuatFromAngleAxis(CH_PI / 5, ChVector3d(1, 2, 3).GetNormalized()))  //
    };

    for (const auto& frame : frames) {
        for (const auto& pc : GetProbeCases(test_gap)) {
            SCOPED_TRACE(pc.name);

            ChSystemNSC sys;
            SetupSystem(sys, type);
            AddRoundedBody(sys, pc.shape, frame);
            AddProbeBody(sys, pc, frame);
            sys.DoStepDynamics(1e-9);

            auto reporter = chrono_types::make_shared<ContactReporter>();
            sys.GetContactContainer()->ReportAllContacts(reporter);

            ASSERT_GT(reporter->num_contacts, 0);
            EXPECT_NEAR(reporter->min_distance, test_gap, tolerance);
            // The contact normal may point from either object to the other
            ChVector3d normal = frame.TransformDirectionLocalToParent(pc.normal);
            EXPECT_NEAR(std::abs(Vdot(reporter->normal, normal)), 1.0, 1e-2);
        }
    }
}

TEST(RoundedShapes, contacts_bullet) {
    TestContacts(ChCollisionSystem::Type::BULLET, 2e-3);
}

TEST(RoundedShapes, contacts_multicore) {
#ifdef CHRONO_COLLISION
    TestContacts(ChCollisionSystem::Type::MULTICORE, 1e-5);
#else
    GTEST_SKIP() << "Chrono multicore collision system not available";
#endif
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
void Visualize(ChCollisionSystem::Type type) {
    ChSystemNSC sys;
    SetupSystem(sys, type);

    ChFrame<> cyl_frame(ChVector3d(0, -1.5, 0), QUNIT);
    ChFrame<> box_frame(ChVector3d(0, +1.5, 0), QUNIT);
    AddRoundedBody(sys, RoundedShape::CYLINDER, cyl_frame);
    AddRoundedBody(sys, RoundedShape::BOX, box_frame);
    for (const auto& pc : GetProbeCases(vis_gap))
        AddProbeBody(sys, pc, pc.shape == RoundedShape::CYLINDER ? cyl_frame : box_frame);

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle(std::string("Rounded shapes - ") + (type == ChCollisionSystem::Type::BULLET ? "Bullet" : "Multicore"));
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(5, 0, 1.5), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetCollisionVisibility(true);
    vis->SetCollisionColor(ChColor(0.9f, 0.9f, 0.1f));
    vis->SetContactNormalsVisibility(true);
    vis->SetContactNormalsScale(0.3);
    vis->Initialize();

    while (vis->Run()) {
        vis->Render();
        sys.DoStepDynamics(1e-2);
    }
}
#endif

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);

    // Run-time visualization only if explicitly requested (i.e., when the test is run manually)
    bool vis = false;
    auto vis_type = ChCollisionSystem::Type::BULLET;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--vis") == 0)
            vis = true;
        else if (std::strcmp(argv[i], "multicore") == 0)
            vis_type = ChCollisionSystem::Type::MULTICORE;
    }

    int result = RUN_ALL_TESTS();

    if (vis) {
#ifdef CHRONO_VSG
        Visualize(vis_type);
#else
        std::cout << "Run-time visualization not available." << std::endl;
#endif
    }

    return result;
}
