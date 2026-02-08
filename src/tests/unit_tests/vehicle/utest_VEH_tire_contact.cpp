// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test to check tire-terrain intersection functions
//
// =============================================================================

#include <cmath>

#include "gtest/gtest.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"

#include "../ut_utils.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::endl;

constexpr double tol = 1e-2;

// -----------------------------------------------------------------------------

// Flat terrain inclined in the x-z plane
class TestTerrain : public ChTerrain {
  public:
    TestTerrain(double angle) : sa(std::sin(angle)), ca(std::cos(angle)), ta(std::tan(angle)) {}
    virtual ChVector3d GetPoint(const ChVector3d& loc) const override { return ChVector3d(loc.x(), 0, loc.x() * ta); }
    virtual double GetHeight(const ChVector3d& loc) const override { return loc.x() * sa; }
    virtual ChVector3d GetNormal(const ChVector3d& loc) const override { return ChVector3d(-sa, 0, ca); }
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return 1.0; }

  private:
    double sa;
    double ca;
    double ta;
};

// -----------------------------------------------------------------------------

// Calculate tire-terrain intersection for given terrain inclination and intersection method
class TireCollision {
  public:
    TireCollision(ChTire::CollisionType method, double angle, bool verbose = false)
        : verbose(verbose), method(method), angle(angle) {
        std::cout << "angle: " << angle * CH_RAD_TO_DEG << " deg" << endl;
        Calculate();
    }

    void Calculate() {
        auto sa = std::sin(angle);
        auto ca = std::cos(angle);

        // Construct terrain inclined by given angle
        TestTerrain terrain(angle);

        // Set up tire disc geometry
        double r = 1.0;
        double w = 0.5;
        double delta = 0.1;
        ChVector3d center(0, 0, r - delta);
        ChVector3d normal(0, 1, 0);

        ////{
        ////    double h = r - delta;
        ////    double ct = h * ca / r;
        ////    double theta = std::acos(ct);
        ////    std::cout << "THETA = " << theta * CH_RAD_TO_DEG << std::endl;
        ////    theta *= 2;
        ////    double area = 0.5 * r * r * (theta - std::sin(theta));
        ////    std::cout << "AREA = " << area << std::endl;
        ////}

        // Calculate contact patch csys, penetration depth, and coefficient of friction
        switch (method) {
            case ChTire::CollisionType::SINGLE_POINT:
                contact = ChTire::DiscTerrainCollision1pt(terrain, center, normal, r, csys, depth, mu);
                cout << "1 point" << endl;
                break;
            case ChTire::CollisionType::FOUR_POINTS:
                contact = ChTire::DiscTerrainCollision4pt(terrain, center, normal, r, w, csys, depth, mu);
                cout << "4 points" << endl;
                break;
            case ChTire::CollisionType::ENVELOPE: {
                ChFunctionInterp area_depth;
                ChTire::ConstructAreaDepthTable(r, area_depth);
                contact =
                    ChTire::DiscTerrainCollisionEnvelope(terrain, center, normal, r, w, area_depth, csys, depth, mu);
                cout << "envelope" << endl;
                break;
            }
        }
        if (verbose) {
            cout << "  contact? " << (contact ? "yes" : "no") << endl;
            cout << "  depth:   " << depth << endl;
            cout << "  csys:    " << csys << endl;
        }

        // Terrain normal at origin of csys and z direction of csys
        terrain_normal = terrain.GetNormal(csys.pos);

        // Z axis of csys
        csys_normal = csys.rot.GetAxisZ();

        // Distance from csys origin to terrain surface
        csys_dist = std::abs(sa * csys.pos.x() - ca * csys.pos.z());

        if (verbose) {
            cout << "  terrain normal at csys origin: " << terrain_normal << endl;
            cout << "  csys Z direction:              " << csys_normal << endl;
            cout << "  csys-terrain distance:         " << csys_dist << endl;
        }
    }

    bool verbose;

    ChTire::CollisionType method;
    double angle;

    bool contact;
    ChCoordsysd csys;
    double depth;
    float mu;

    ChVector3d terrain_normal;
    ChVector3d csys_normal;
    double csys_dist;
};

// -----------------------------------------------------------------------------
// Individual test
// -----------------------------------------------------------------------------

////TEST(ChronoVehicle, disc_collision) {
////    double angle = 30 * CH_DEG_TO_RAD;
////    ChTire::CollisionType method = ChTire::CollisionType::ENVELOPE;
////
////    TireCollision test(method, angle, true);
////    auto csys = test.csys;
////    auto dist = test.csys_dist;
////    auto csys_normal = test.csys_normal;
////    auto terrain_normal = test.terrain_normal;
////
////    // Check csys origin on terrain surface
////    ASSERT_NEAR(csys.pos.y(), 0.0, tol);
////    ASSERT_NEAR(csys.pos.z(), csys.pos.x() * std::tan(angle), tol);
////    ASSERT_NEAR(dist, 0.0, tol);
////
////    // Check csys Z direction same as terrain normal
////    Assert_near(csys_normal, terrain_normal, tol);
////}

// -----------------------------------------------------------------------------
// Test suite
// -----------------------------------------------------------------------------

class TireCollisionTest : public TireCollision,
                          public ::testing::TestWithParam<std::tuple<ChTire::CollisionType, double>> {
  protected:
    TireCollisionTest() : TireCollision(std::get<0>(GetParam()), std::get<1>(GetParam()) * CH_DEG_TO_RAD) {}
};

TEST_P(TireCollisionTest, csys) {
    ASSERT_NEAR(csys.pos.y(), 0.0, tol);
    ASSERT_NEAR(csys.pos.z(), csys.pos.x() * std::tan(angle), tol);
    ASSERT_NEAR(csys_dist, 0.0, tol);
    Assert_near(csys_normal, terrain_normal, tol);
}

////INSTANTIATE_TEST_SUITE_P(ChronoVehicle,
////                         TireCollisionTest,
////                         ::testing::Combine(::testing::Values(ChTire::CollisionType::ENVELOPE),
////                                            ::testing::Values(0.0, 10.0, 20.0, 30.0)));

INSTANTIATE_TEST_SUITE_P(ChronoVehicle,
                         TireCollisionTest,
                         ::testing::Combine(::testing::Values(ChTire::CollisionType::SINGLE_POINT,
                                                              ChTire::CollisionType::FOUR_POINTS,
                                                              ChTire::CollisionType::ENVELOPE),
                                            ::testing::Values(0.0, 10.0, 20.0, 30.0)));
