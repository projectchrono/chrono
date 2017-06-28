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
// Authors: Alessandro Tasora
// =============================================================================
// Utilities for rotations in 3D. Adapted from MBDyn
// =============================================================================

#ifndef CHROTUTILS_H
#define CHROTUTILS_H

#include <vector>

#include "chrono/core/ChMatrix33.h"
#include "chrono_fea/ChApiFEA.h"

namespace chrono {
namespace fea {

/// Utility functions for rotations in 3D and their derivatives.
/// Adapted from the MBDyn library

namespace ChRotUtils {

const int COEFF_A = 1;
const int COEFF_B = 2;
const int COEFF_C = 3;
const int COEFF_D = 4;
const int COEFF_E = 5;
const int COEFF_F = 6;

const int COEFF_C_STAR = 1;
const int COEFF_E_STAR = 2;

const double SerCoeff[6][9] = {
    {1.,  // a
     -6., 120., -5040., 362880., -39916800., 6227020800., -1307674368000., 355687428096000.},
    {2.,  // b
     -24., 720., -40320., 3628800., -479001600., 87178291200., -20922789888000., 6402373705728000.},
    {6.,  // c
     -120., 5040., -362880., 39916800., -6227020800., 1307674368000., -355687428096000., 121645100408832000.},
    {-12.,  // d
     180., -6720., 453600., -47900160., 7264857600., -1494484992000., 400148356608000., -135161222676480000.},
    {-60.,  // e
     1260., -60480., 4989600., -622702080., 108972864000., -25406244864000., 7602818775552000., -2838385676206080000.},
    {90.,  // f
     -1680., 75600., -5987520., 726485760., -124540416000., 28582025472000., -8447576417280000., 3122224243826688000.}};

const double SerTrunc[6] = {9,   // for a: phi^16
                            9,   // for b: phi^16
                            9,   // for c: phi^16
                            9,   // for d: phi^16
                            9,   // for e: phi^16
                            9};  // for f: phi^16

const double SerThrsh[6] = {1.1,   // for a = coeff[0]
                            1.3,   // for b = coeff[1]
                            1.5,   // for c = coeff[2]
                            1.6,   // for d = coeff[3]
                            1.7,   // for e = coeff[4]
                            1.8};  // for f = coeff[5]

template <class T1, class T2>
void RotCo(const int cid, const T1& phi, const ChVector<>& p, T2* const cf) {
    T2 phip[10];
    T2 phi2(phi ^ phi);
    double mp(p.Length());  // (sqrt(p.Dot()));
    int k, j;

    if (mp < SerThrsh[cid - 1]) {
        phip[0] = 1.;
        for (j = 1; j <= 9; j++) {
            phip[j] = phip[j - 1] * phi2;
        }
        for (k = 0; k < cid; k++) {
            cf[k] = 0.;
            for (j = 0; j < SerTrunc[k]; j++) {
                cf[k] += phip[j] / SerCoeff[k][j];
            }
        }

        return;
    }

    const T2 ID(1.);
    T2 pd(sqrt(phi2));
    cf[0] = sin(pd) / pd;  // a = sin(phi)/phi
    if (cid == 1)
        return;
    cf[1] = (ID - cos(pd)) / phi2;  // b = (1.-cos(phi))/phi2
    if (cid == 2)
        return;
    cf[2] = (ID - cf[0]) / phi2;  // c = (1.-a)/phi2
    if (cid == 3)
        return;
    cf[3] = (cf[0] - (cf[1] * 2.)) / phi2;  // d = (a-2*b)/phi2
    if (cid == 4)
        return;
    cf[4] = (cf[1] - (cf[2] * 3.)) / phi2;  // e = (b-3*c)/phi2
    if (cid == 5)
        return;
    cf[5] = (cf[2] - cf[1] - (cf[3] * 4.)) / phi2;  // f = (c-b-4*d)/phi2
    // if (cid == 6) return; inutile
    return;
};

// Coefficients:            up to a     (COEFF_A)
template <class T1, class T2>
void CoeffA(const T1& phi, const ChVector<>& p, T2* const coeff) {
    RotCo(COEFF_A, phi, p, coeff);
};

// Coefficients:            up to b     (COEFF_B)
template <class T1, class T2>
void CoeffB(const T1& phi, const ChVector<>& p, T2* const coeff) {
    RotCo(COEFF_B, phi, p, coeff);
};

// Coefficients:            up to c     (COEFF_C)
template <class T1, class T2>
void CoeffC(const T1& phi, const ChVector<>& p, T2* const coeff) {
    RotCo(COEFF_C, phi, p, coeff);
};

// Coefficients:            up to d     (COEFF_D)
template <class T1, class T2>
void CoeffD(const T1& phi, const ChVector<>& p, T2* const coeff) {
    RotCo(COEFF_D, phi, p, coeff);
};

// Coefficients:            up to e     (COEFF_E)
template <class T1, class T2>
void CoeffE(const T1& phi, const ChVector<>& p, T2* const coeff) {
    RotCo(COEFF_E, phi, p, coeff);
};

// Coefficients:            up to f     (COEFF_F)
template <class T1, class T2>
void CoeffF(const T1& phi, const ChVector<>& p, T2* const coeff) {
    RotCo(COEFF_F, phi, p, coeff);
};

// Starred coefficients:    up to c*    (COEFF_C_STAR)
// Coefficients:            up to d     (COEFF_D)
template <class T1, class T2>
void CoeffCStar(const T1& phi, const ChVector<>& p, T2* const coeff, T2* const coeffs) {
    RotCo(COEFF_D, phi, p, coeff);
    coeffs[0] = -coeff[3] / (coeff[1] * 2.);
};

// Starred coefficients:    up to e*    (COEFF_E_STAR)
// Coefficients:            up to f     (COEFF_F)
template <class T1, class T2>
void CoeffEStar(const T1& phi, const ChVector<>& p, T2* const coeff, T2* const coeffs) {
    RotCo(COEFF_F, phi, p, coeff);
    coeffs[0] = -coeff[3] / (coeff[1] * 2.);
    coeffs[1] = -(coeff[4] + coeff[5]) / (coeff[1] * 4.);
};

/// Compute the rotation matrix Phi from Euler Rogriguez's parameters phi
static ChMatrix33<> Rot(const ChVector<>& phi) {
    double coeff[COEFF_B];

    CoeffB(phi, phi, coeff);

    ChMatrix33<> Eye(1);
    ChMatrix33<> Phix;
    Phix.Set_X_matrix(phi * coeff[0]);
    ChMatrix33<> Phi(Eye + Phix); /* I + c[0] * phi x */
    ChMatrix33<> pxpx;
    pxpx.Set_XY_matrix(phi, phi * coeff[1]);
    Phi += pxpx; /* += c[1] * phi x phi x */

    return Phi;
}

/// Compute a G matrix from Euler Rogriguez's parameters Phi
/// G defined in such a way that dPhi * PhiT = G * dphi
static ChMatrix33<> DRot(const ChVector<>& phi) {
    double coeff[COEFF_C];

    CoeffC(phi, phi, coeff);

    ChMatrix33<> Eye(1);
    ChMatrix33<> Phix;
    Phix.Set_X_matrix(phi * coeff[1]);
    ChMatrix33<> Ga(Eye + Phix); /* I + c[1] * phi x */
    ChMatrix33<> pxpx;
    pxpx.Set_XY_matrix(phi, phi * coeff[2]);
    Ga += pxpx; /* += c[2] * phi x phi x */

    return Ga;
}

/// Compute rotation matrix Phi and Ga matrix
/// from Euler Rogriguez's parameters Phi
static void RotAndDRot(const ChVector<>& phi, ChMatrix33<>& Phi, ChMatrix33<>& Ga) {
    double coeff[COEFF_C];

    CoeffC(phi, phi, coeff);

    ChMatrix33<> Eye(1);
    ChMatrix33<> Phix;
    Phix.Set_X_matrix(phi * coeff[0]);
    Phi = (Eye + Phix); /* I + c[0] * phi x */
    ChMatrix33<> pxpx;
    pxpx.Set_XY_matrix(phi, phi * coeff[1]);
    Phi += pxpx; /* += c[1] * phi x phi x */

    Phix.Set_X_matrix(phi * coeff[1]);
    Ga = (Eye + Phix); /* I + c[1] * phi x */
    pxpx.Set_XY_matrix(phi, phi * coeff[2]);
    Ga += pxpx; /* += c[2] * phi x phi x */

    return;
}

/// Compute the inverse transpose of G matrix from
/// Euler Rogriguez's parameters Phi
static ChMatrix33<> DRot_IT(const ChVector<>& phi) {
    double coeff[COEFF_D], coeffs[COEFF_C_STAR];

    CoeffCStar(phi, phi, coeff, coeffs);

    ChMatrix33<> Eye(1);
    ChMatrix33<> Phix;
    Phix.Set_X_matrix(phi * 0.5);
    ChMatrix33<> GaIT(Eye + Phix); /* I +  0.5 phi x */
    ChMatrix33<> pxpx;
    pxpx.Set_XY_matrix(phi, phi * coeffs[0]);
    GaIT += pxpx;

    return GaIT;
}

/// Compute the inverse of G matrix from Euler Rogriguez's parameters Phi
static ChMatrix33<> DRot_I(const ChVector<>& phi) {
    double coeff[COEFF_D], coeffs[COEFF_C_STAR];

    CoeffCStar(phi, phi, coeff, coeffs);

    ChMatrix33<> Eye(1);
    ChMatrix33<> Phix;
    Phix.Set_X_matrix(phi * (-0.5));
    ChMatrix33<> GaI(Eye + Phix); /* I +  -0.5 phi x */
    ChMatrix33<> pxpx;
    pxpx.Set_XY_matrix(phi, phi * coeffs[0]);
    GaI += pxpx;

    return GaI;
}

/// Compute inverse transpose ot rotation matrix Phi and Ga matrix
/// given Euler Rogriguez's parameters Phi
static void RotAndDRot_IT(const ChVector<>& phi, ChMatrix33<>& PhiIT, ChMatrix33<>& GaIT) {
    double coeff[COEFF_D], coeffs[COEFF_C_STAR];

    CoeffCStar(phi, phi, coeff, coeffs);

    ChMatrix33<> Eye(1);
    ChMatrix33<> Phix;
    Phix.Set_X_matrix(phi * coeff[0]);
    PhiIT = Eye + Phix;
    ChMatrix33<> pxpx;
    pxpx.Set_XY_matrix(phi, phi * coeffs[1]);
    PhiIT += pxpx;

    Phix.Set_X_matrix(phi * 0.5);
    GaIT = Eye + Phix;
    pxpx.Set_XY_matrix(phi, phi * coeffs[0]);
    GaIT += pxpx;

    return;
}

/// Compute Euler Rogriguez's parameters phi from rotation matrix Phi
static ChVector<> VecRot(const ChMatrix33<>& Phi) {
    double a, cosphi, sinphi;
    ChVector<> unit;

    cosphi = (Phi.GetTrace() - 1.) / 2.;
    if (cosphi > 0.) {
        unit = Phi.GetAx();
        sinphi = unit.Length();
        double phi = atan2(sinphi, cosphi);
        CoeffA(ChVector<>(phi, 0., 0.), ChVector<>(phi, 0., 0.), &a);
        unit /= a;
    } else {
        // -1 <= cosphi <= 0
        ChMatrix33<> eet(Phi.GetSymm());
        eet(0, 0) -= cosphi;
        eet(1, 1) -= cosphi;
        eet(2, 2) -= cosphi;
        int maxcol = 0;
        ChVector<> col = eet.Get_A_Xaxis();
        if (eet(1, 1) > eet(0, 0)) {
            maxcol = 1;
            col = eet.Get_A_Yaxis();
        }
        if (eet(2, 2) > eet(maxcol, maxcol)) {
            maxcol = 2;
            col = eet.Get_A_Zaxis();
        }
        unit = (col / sqrt(eet(maxcol, maxcol) * (1. - cosphi)));
        ChMatrix33<> unitx;
        unitx.Set_X_matrix(unit);
        sinphi = -(unitx * Phi).GetTrace() / 2.;
        unit *= atan2(sinphi, cosphi);
    }
    return unit;
}

/// Compute, given Euler Rogriguez's parameters phi, a L matrix such that
/// dG * a = L(phi, a) * dphi
static ChMatrix33<> Elle(const ChVector<>& phi, const ChVector<>& a) {
    double coeff[COEFF_E];
    CoeffE(phi, phi, coeff);

    ChMatrix33<> L;
    L.Set_X_matrix(a * (-coeff[1]));
    ChMatrix33<> mx;
    mx.Set_XY_matrix(phi, a * coeff[2]);
    L -= mx;
    mx.Set_X_matrix(Vcross(phi, a * coeff[2]));
    L -= mx;
    L += TensorProduct(Vcross(phi, a), phi * coeff[3]);
    mx.Set_XY_matrix(phi, phi);
    L += TensorProduct(mx * a, phi * coeff[4]);

    return L;
}

}  // end of namespace ChRotUtils

}  // end of namespace fea
}  // end of namespace chrono

#endif
