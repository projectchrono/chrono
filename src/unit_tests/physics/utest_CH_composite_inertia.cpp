// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include <iostream>
#include <string>
#include <cmath>

#include "chrono/core/ChLog.h"
#include "chrono/utils/ChCompositeInertia.h"

using namespace chrono;

double tol = 1e-4;

// ====================================================================================

bool test_hemispheres() {
    GetLog() << "\nHemispheres test\n\n";

    // Add top and bottom hemispheres (z direction, with center at specified height)
    double height = 2;

    double r = 1.5;
    double rho = 50;
    double mass = (2.0 / 3.0) * CH_C_PI * rho * std::pow(r, 3);
    double offset = (3.0 / 8.0) * r;
    double Jxx = (83.0 / 320.0) * mass * r * r;
    double Jyy = Jxx;
    double Jzz = (2.0 / 5.0) * mass * r * r;

    utils::CompositeInertia comp;

    comp.AddComponent(ChFrame<>(ChVector<>(0, 0, height + offset), ChQuaternion<>(1, 0, 0, 0)), mass,
                      ChMatrix33<>(ChVector<>(Jxx, Jyy, Jzz)));

    comp.AddComponent(ChFrame<>(ChVector<>(0, 0, height - offset), Q_from_AngX(CH_C_PI)), mass,
                      ChMatrix33<>(ChVector<>(Jxx, Jyy, Jzz)));

    double c_mass = comp.GetMass();
    ChVector<> c_com = comp.GetCOM();
    ChMatrix33<> c_inertia = comp.GetInertia();

    GetLog() << "composite mass: " << c_mass << "\n";
    GetLog() << "composite COM: " << c_com << "\n";
    GetLog() << "composite inertia: " << c_inertia << "\n";

    // Inertia properties of equivalent sphere
    double s_mass = 2 * mass;
    ChVector<> s_com(0, 0, height);
    double s_J = (2.0 / 5.0) * s_mass * r * r;
    ChMatrix33<> s_inertia(ChVector<>(s_J, s_J, s_J));

    GetLog() << "sphere mass: " << s_mass << "\n";
    GetLog() << "sphere COM: " << s_com << "\n";
    GetLog() << "sphere inertia: " << s_inertia << "\n";

    // Compare results
    double err_mass = c_mass - s_mass;
    double err_com = (c_com - s_com).Length();
    double err_inertia = (c_inertia - s_inertia).NormTwo();

    GetLog() << "error mass:    " << err_mass << "\n";
    GetLog() << "error com:     " << err_com << "\n";
    GetLog() << "error inertia: " << err_inertia << "\n";

    return (err_mass <= tol && err_com <= tol && err_inertia <= tol);
}

// ====================================================================================

bool test_boxes() {
    GetLog() << "\nBoxes test\n\n";

    ChVector<> center(1, 2, 3);
    double hx = 2;
    double hy = 4;
    double hz = 6;
    double rho = 100;

    // Create composite of multipel boxes in all 3 directions
    int nx = 5;
    int ny = 3;
    int nz = 2;

    double hx1 = hx / nx;
    double hy1 = hy / ny;
    double hz1 = hz / nz;

    double mass1 = 8 * hx1 * hy1 * hz1 * rho;
    ChMatrix33<> inertia1(ChVector<>(mass1 * (hy1 * hy1 + hz1 * hz1) / 3, mass1 * (hx1 * hx1 + hz1 * hz1) / 3,
                                     mass1 * (hx1 * hx1 + hy1 * hy1) / 3));

    utils::CompositeInertia comp;

    for (int ix = 0; ix < nx; ix++) {
        double cx = ix * 2 * (hx - hx1) / (nx - 1.0) + center.x() - hx + hx1;
        for (int iy = 0; iy < ny; iy++) {
            double cy = iy * 2 * (hy - hy1) / (ny - 1.0) + center.y() - hy + hy1;
            for (int iz = 0; iz < nz; iz++) {
                double cz = iz * 2 * (hz - hz1) / (nz - 1.0) + center.z() - hz + hz1;
                comp.AddComponent(ChFrame<>(ChVector<>(cx, cy, cz), ChQuaternion<>(1, 0, 0, 0)), mass1, inertia1);
            }
        }
    }

    double c_mass = comp.GetMass();
    ChVector<> c_com = comp.GetCOM();
    ChMatrix33<> c_inertia = comp.GetInertia();

    GetLog() << "composite mass: " << c_mass << "\n";
    GetLog() << "composite COM: " << c_com << "\n";
    GetLog() << "composite inertia: " << c_inertia << "\n";

    // Inertia properties of single box
    double b_mass = 8 * hx * hy * hz * rho;
    ChVector<> b_com = center;
    ChMatrix33<> b_inertia(ChVector<>(b_mass * (hy * hy + hz * hz) / 3, b_mass * (hx * hx + hz * hz) / 3,
                                      b_mass * (hx * hx + hy * hy) / 3));

    GetLog() << "box mass: " << b_mass << "\n";
    GetLog() << "box COM: " << b_com << "\n";
    GetLog() << "box inertia: " << b_inertia << "\n";

    // Compare results
    double err_mass = c_mass - b_mass;
    double err_com = (c_com - b_com).Length();
    double err_inertia = (c_inertia - b_inertia).NormTwo();

    GetLog() << "error mass:    " << err_mass << "\n";
    GetLog() << "error com:     " << err_com << "\n";
    GetLog() << "error inertia: " << err_inertia << "\n";

    return (err_mass <= tol && err_com <= tol && err_inertia <= tol);
}

// ====================================================================================

bool test_hollow_sphere() {
    GetLog() << "\nHollow sphere test\n\n";

    double r_out = 2.3;
    double r_in = 2.1;
    double rho = 50;

    // Hollow sphere as a composite
    utils::CompositeInertia comp;

    double mass_out = (4.0 / 3.0) * CH_C_PI * rho * std::pow(r_out, 3);
    double mass_in = (4.0 / 3.0) * CH_C_PI * rho * std::pow(r_in, 3);

    double J_out = (2.0 / 5.0) * mass_out * r_out * r_out;
    double J_in = (2.0 / 5.0) * mass_in * r_in * r_in;

    comp.AddComponent(ChFrame<>(), mass_out, ChMatrix33<>(J_out));
    comp.AddComponent(ChFrame<>(), mass_in, ChMatrix33<>(J_in), true);

    double c_mass = comp.GetMass();
    ChVector<> c_com = comp.GetCOM();
    ChMatrix33<> c_inertia = comp.GetInertia();

    GetLog() << "composite mass: " << c_mass << "\n";
    GetLog() << "composite COM: " << c_com << "\n";
    GetLog() << "composite inertia: " << c_inertia << "\n";

    // Inertia properties of hollow sphere
    double s_mass = (4.0 / 3.0) * CH_C_PI * rho * (std::pow(r_out, 3) - std::pow(r_in, 3));
    double s_J =
        (2.0 / 5.0) * s_mass * (std::pow(r_out, 5) - std::pow(r_in, 5)) / (std::pow(r_out, 3) - std::pow(r_in, 3));
    ChMatrix33<> s_inertia(s_J);

    GetLog() << "sphere mass: " << s_mass << "\n";
    GetLog() << "sphere inertia: " << s_inertia << "\n";

    // Compare results
    double err_mass = c_mass - s_mass;
    double err_com = (c_com).Length();
    double err_inertia = (c_inertia - s_inertia).NormTwo();

    GetLog() << "error mass:    " << err_mass << "\n";
    GetLog() << "error com:     " << err_com << "\n";
    GetLog() << "error inertia: " << err_inertia << "\n";

    return (err_mass <= tol && err_com <= tol && err_inertia <= tol);
}

// ====================================================================================

int main(int argc, char* argv[]) {
    bool passed = true;
    passed &= test_hemispheres();
    passed &= test_boxes();
    passed &= test_hollow_sphere();

    // Return 0 if all tests passed.
    std::cout << "\n\n" << (passed ? "PASSED" : "FAILED") << std::endl;
    return !passed;
}
