// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Unit test for CompositeInertia utility.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/utils/ChCompositeInertia.h"
#include "chrono/core/ChRotation.h"

using namespace chrono;

double tol = 1e-4;

// ====================================================================================

static void TestVector(const ChVector3d& v1, const ChVector3d& v2, double tolerance) {
    ASSERT_NEAR(v1.x(), v2.x(), tolerance);
    ASSERT_NEAR(v1.y(), v2.y(), tolerance);
    ASSERT_NEAR(v1.z(), v2.z(), tolerance);
}

// ====================================================================================

TEST(CompositeInertia, hemispheres) {
    // Add top and bottom hemispheres (z direction, with center at specified height)
    double height = 2;

    double r = 1.5;
    double rho = 50;
    double mass = CH_2_3 * CH_PI * rho * std::pow(r, 3);
    double offset = (3.0 / 8.0) * r;
    double Jxx = (83.0 / 320.0) * mass * r * r;
    double Jyy = Jxx;
    double Jzz = (2.0 / 5.0) * mass * r * r;

    utils::CompositeInertia comp;

    comp.AddComponent(ChFrame<>(ChVector3d(0, 0, height + offset), ChQuaternion<>(1, 0, 0, 0)), mass,
                      ChMatrix33<>(ChVector3d(Jxx, Jyy, Jzz)));

    comp.AddComponent(ChFrame<>(ChVector3d(0, 0, height - offset), QuatFromAngleX(CH_PI)), mass,
                      ChMatrix33<>(ChVector3d(Jxx, Jyy, Jzz)));

    double c_mass = comp.GetMass();
    ChVector3d c_com = comp.GetCOM();
    ChMatrix33<> c_inertia = comp.GetInertia();

    // Inertia properties of equivalent sphere
    double s_mass = 2 * mass;
    ChVector3d s_com(0, 0, height);
    double s_J = (2.0 / 5.0) * s_mass * r * r;
    ChMatrix33<> s_inertia(ChVector3d(s_J, s_J, s_J));

    // Check
    ASSERT_NEAR(c_mass, s_mass, tol);
    TestVector(c_com, s_com, tol);
    ASSERT_NEAR((c_inertia - s_inertia).norm(), 0.0, tol);
}

// ====================================================================================

TEST(CompositeInertia, boxes) {
    ChVector3d center(1, 2, 3);
    double hx = 2;
    double hy = 4;
    double hz = 6;
    double rho = 100;

    // Create composite of multiple boxes in all 3 directions
    int nx = 5;
    int ny = 3;
    int nz = 2;

    double hx1 = hx / nx;
    double hy1 = hy / ny;
    double hz1 = hz / nz;

    double mass1 = 8 * hx1 * hy1 * hz1 * rho;
    ChMatrix33<> inertia1(ChVector3d(mass1 * (hy1 * hy1 + hz1 * hz1) / 3, mass1 * (hx1 * hx1 + hz1 * hz1) / 3,
                                     mass1 * (hx1 * hx1 + hy1 * hy1) / 3));

    utils::CompositeInertia comp;

    for (int ix = 0; ix < nx; ix++) {
        double cx = ix * 2 * (hx - hx1) / (nx - 1.0) + center.x() - hx + hx1;
        for (int iy = 0; iy < ny; iy++) {
            double cy = iy * 2 * (hy - hy1) / (ny - 1.0) + center.y() - hy + hy1;
            for (int iz = 0; iz < nz; iz++) {
                double cz = iz * 2 * (hz - hz1) / (nz - 1.0) + center.z() - hz + hz1;
                comp.AddComponent(ChFrame<>(ChVector3d(cx, cy, cz), ChQuaternion<>(1, 0, 0, 0)), mass1, inertia1);
            }
        }
    }

    double c_mass = comp.GetMass();
    ChVector3d c_com = comp.GetCOM();
    ChMatrix33<> c_inertia = comp.GetInertia();

    // Inertia properties of single box
    double b_mass = 8 * hx * hy * hz * rho;
    ChVector3d b_com = center;
    ChMatrix33<> b_inertia(ChVector3d(b_mass * (hy * hy + hz * hz) / 3, b_mass * (hx * hx + hz * hz) / 3,
                                      b_mass * (hx * hx + hy * hy) / 3));

    // Check
    ASSERT_NEAR(c_mass, b_mass, tol);
    TestVector(c_com, b_com, tol);
    ASSERT_NEAR((c_inertia - b_inertia).norm(), 0.0, tol);
}

// ====================================================================================

TEST(CompositeInertia, hollow_sphere) {
    double r_out = 2.3;
    double r_in = 2.1;
    double rho = 50;

    // Hollow sphere as a composite
    utils::CompositeInertia comp;

    double mass_out = CH_4_3 * CH_PI * rho * std::pow(r_out, 3);
    double mass_in = CH_4_3 * CH_PI * rho * std::pow(r_in, 3);

    double J_out = (2.0 / 5.0) * mass_out * r_out * r_out;
    double J_in = (2.0 / 5.0) * mass_in * r_in * r_in;

    comp.AddComponent(ChFrame<>(), mass_out, ChMatrix33<>(J_out));
    comp.AddComponent(ChFrame<>(), mass_in, ChMatrix33<>(J_in), true);

    double c_mass = comp.GetMass();
    ChVector3d c_com = comp.GetCOM();
    ChMatrix33<> c_inertia = comp.GetInertia();

    // Inertia properties of hollow sphere
    double s_mass = CH_4_3 * CH_PI * rho * (std::pow(r_out, 3) - std::pow(r_in, 3));
    double s_J =
        (2.0 / 5.0) * s_mass * (std::pow(r_out, 5) - std::pow(r_in, 5)) / (std::pow(r_out, 3) - std::pow(r_in, 3));
    ChMatrix33<> s_inertia(s_J);

    // Check
    ASSERT_NEAR(c_mass, s_mass, tol);
    ASSERT_NEAR(c_com.Length(), 0.0, tol);
    ASSERT_NEAR((c_inertia - s_inertia).norm(), 0.0, tol);
}
