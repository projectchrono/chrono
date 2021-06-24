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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

//#define BEAM_VERBOSE
#include "chrono/core/ChQuadrature.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"

namespace chrono {
namespace fea {
void ChElementBeamTaperedTimoshenkoFPM::ShapeFunctionsTimoshenkoFPM(ShapeFunctionGroupFPM& NB, double eta) {
    // The shape functions have referenced two papers below, especially the first one:
    // Alexander R.Stäblein,and Morten H.Hansen.
    // "Timoshenko beam element with anisotropic cross-sectional properties." %
    // ECCOMAS Congress 2016, VII European Congress on Computational Methods in Applied Sciences and Engineering.
    // Crete Island, Greece, 5 - 10 June 2016
    // 
    // Taeseong Kim, Anders M.Hansen,and Kim Branner.
    // "Development of an anisotropic beam finite element for composite wind turbine blades in multibody system." 
    // Renewable Energy 59(2013) : 172 - 183.
    
    // eta = 2 * x/L;
    // x = (-L/2, L/2),  hence eta = (-1, 1)
    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;
    double eta1 = (eta + 1) / 2.0;
    double eta2 = eta1 * eta1;
    double eta3 = eta2 * eta1;

    ChMatrixNM<double, 6, 14> Ax;    
    Ax.setZero();
    ChMatrixNM<double, 6, 14> dAx;
    dAx.setZero();
    ChMatrixNM<double, 14, 14> Ex;
    Ex.setZero();
    ChMatrixNM<double, 14, 14> Ex_inv;
    Ex_inv.setZero();
    ChMatrixNM<double, 6, 12> Nx;
    Nx.setZero();
    ChMatrixNM<double, 6, 12> dNx;
    dNx.setZero();

    // The coefficient matrix of the displacements and rotations with respect to the shape function coefficient vector c_v
    // note: the shape function coefficient vector is as below:
    // c_v = [c1 c2 c3 c4 c5 c6 c7 c8 c9 c10 c13 c14 c11 c12].';  // notice the order of c11 c12 c13 c14
    Ax.row(0) << L * eta1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ax.row(1) << 0, 0, LLL * eta3, LL * eta2, L * eta1, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    Ax.row(2) << 0, 0, 0, 0, 0, 0, LLL * eta3, LL * eta2, L * eta1, 1, 0, 0, 0, 0;
    Ax.row(3) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, L * eta1, 1;
    Ax.row(4) << 0, 0, 0, 0, 0, 0, -3 * LL * eta2, -2 * L * eta1, -1, 0, 1, 0, 0, 0;
    Ax.row(5) << 0, 0, 3 * LL * eta2, 2 * L * eta1, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0;

    // The derivative of Ax
    dAx.row(0) << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0 ;
    dAx.row(1) << 0, 0, 3 * LL * eta2, 2 * L * eta1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    dAx.row(2) << 0, 0, 0, 0, 0, 0, 3 * LL * eta2, 2 * L * eta1, 1, 0, 0, 0, 0, 0;
    dAx.row(3) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    dAx.row(4) << 0, 0, 0, 0, 0, 0, -6 * L * eta1, -2, 0, 0, 0, 0, 0, 0;
    dAx.row(5) << 0, 0, 6 * L * eta1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // A temporary transformation matrix
    ChMatrixNM<double, 14, 12> Ttemp;
    Ttemp.setZero();
    Ttemp.block<12, 12>(2, 0).setIdentity();

    // the cross-sectional material stiffness matrix Klaw along beam element may be variant
    // due to different Klaw at two ends of tapered-sectional beam
    ChMatrixNM<double, 6, 6> Klaw_point = this->tapered_section_fpm->GetKlawAtPoint(eta);
    //double k11 = Klaw_point(0, 0);
    double k12 = Klaw_point(0, 1);
    double k13 = Klaw_point(0, 2);
    //double k14 = Klaw_point(0, 3);
    //double k15 = Klaw_point(0, 4);
    //double k16 = Klaw_point(0, 5);
    double k22 = Klaw_point(1, 1);
    double k23 = Klaw_point(1, 2);
    double k24 = Klaw_point(1, 3);
    double k25 = Klaw_point(1, 4);
    double k26 = Klaw_point(1, 5);
    double k33 = Klaw_point(2, 2);
    double k34 = Klaw_point(2, 3);
    double k35 = Klaw_point(2, 4);
    double k36 = Klaw_point(2, 5);
    //double k44 = Klaw_point(3, 3);
    //double k45 = Klaw_point(3, 4);
    //double k46 = Klaw_point(3, 5);
    double k55 = Klaw_point(4, 4);
    double k56 = Klaw_point(4, 5);
    double k66 = Klaw_point(5, 5);
    
    // The coefficient matrix of the equilibrium and compatibility equations
    // with respect to the shape function coefficient vector c_v
    Ex.row(0) << -k13, 0, 6 * k56 - 3 * L * k36 - 3 * L * eta * k36, -2 * k36, 0, 0,
        3 * L * k35 - 6 * k55 + 3 * L * eta * k35, 2 * k35, 0, 0, -k33, -k23, -k34, 0;
    Ex.row(1) << k12, 0, 6 * k66 + 3 * L * k26 + 3 * L * eta * k26, 2 * k26, 0, 0, 
        -6 * k56 - 3 * L * k25 - 3 * L * eta * k25, -2 * k25, 0, 0, k23, k22, k24, 0;
    Ex.row(2) << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(3) << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    Ex.row(5) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    Ex.row(6) << 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0;
    Ex.row(7) << 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0;
    Ex.row(8) << L, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(9) << 0, 0, LLL, LL, L, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(10) << 0, 0, 0, 0, 0, 0, LLL, LL, L, 1, 0, 0, 0, 0;
    Ex.row(11) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, L, 1;
    Ex.row(12) << 0, 0, 0, 0, 0, 0, -3 * LL, -2 * L, -1, 0, 1, 0, 0, 0;
    Ex.row(13) << 0, 0, 3 * LL, 2 * L, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0;

    // The inverse of Ex
    Ex_inv = Ex.inverse();

    // The shape function matrix at dimensionless position eta
    Nx = Ax * Ex_inv * Ttemp;
    // and its derivative
    dNx = dAx * Ex_inv * Ttemp;  // DO NOT add Ax * dEx_inv * Ttemp, see the first paper please.

    // A temporary matrix
    ChMatrixNM<double, 6, 6> TNtemp;
    TNtemp.setZero();
    TNtemp(1, 5) = -1.0;
    TNtemp(2, 4) = 1.0;

    // The strain displacement matrix at dimensionless position eta
    ChMatrixNM<double, 6, 12> Bx = dNx + TNtemp * Nx;

    // return result
    NB = std::make_tuple(Nx, Bx);
}


void ChElementBeamTaperedTimoshenkoFPM::ComputeStiffnessMatrix0() {
    assert(tapered_section_fpm);

    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;

    ChMatrixNM<double,6,6> Klaw = this->tapered_section_fpm->GetAverageFPM();
    double EA = this->tapered_section_fpm->GetAverageSectionParameters()->EA;
    double GJ = this->tapered_section_fpm->GetAverageSectionParameters()->GJ;
    double GAyy = this->tapered_section_fpm->GetAverageSectionParameters()->GAyy;
    double GAzz = this->tapered_section_fpm->GetAverageSectionParameters()->GAzz;
    double EIyy = this->tapered_section_fpm->GetAverageSectionParameters()->EIyy;
    double EIzz = this->tapered_section_fpm->GetAverageSectionParameters()->EIzz;

    double phiy = this->tapered_section_fpm->GetAverageSectionParameters()->phiy;
    double phiz = this->tapered_section_fpm->GetAverageSectionParameters()->phiz;

    double ay = 1. / (1. + phiy);
    double by = phiy * ay;
    double az = 1. / (1. + phiz);
    double bz = phiz * az;

    double k12 = Klaw(0, 1);
    double k13 = Klaw(0, 2);
    double k14 = Klaw(0, 3);
    double k15 = Klaw(0, 4);
    double k16 = Klaw(0, 5);
    double k23 = Klaw(1, 2);
    double k24 = Klaw(1, 3);
    double k25 = Klaw(1, 4);
    double k26 = Klaw(1, 5);
    double k34 = Klaw(2, 3);
    double k35 = Klaw(2, 4);
    double k36 = Klaw(2, 5);
    double k45 = Klaw(3, 4);
    double k46 = Klaw(3, 5);
    double k56 = Klaw(4, 5);
    

    //TODO: need to check whether this stiffness matrix has shear locking issue!
    Km(0, 0) = EA / L;
    Km(0, 1) = by * k12 / L;
    Km(0, 2) = bz * k13 / L;
    Km(0, 3) = k14 / L;
    Km(0, 4) = -bz * k13 / 2. + k15 / L;
    Km(0, 5) = by * k12 / 2. + k16 / L;
    Km(0, 6) = -EA / L;
    Km(0, 7) = -by * k12 / L;
    Km(0, 8) = -bz * k13 / L;
    Km(0, 9) = -k14 / L;
    Km(0, 10) = -bz * k13 / 2. - k15 / L;
    Km(0, 11) = by * k12 / 2. - k16 / L;
    Km(1, 1) = 12. * EIzz * ay / LLL;
    Km(1, 2) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Km(1, 3) = by * k24 / L;
    Km(1, 4) = (L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Km(1, 5) = 6. * EIzz * ay / LL + k26 * by / L;
    Km(1, 6) = -by * k12 / L;
    Km(1, 7) = -12. * EIzz * ay / LLL;
    Km(1, 8) = -Km(1, 2);
    Km(1, 9) = -by * k24 / L;
    Km(1, 10) = (-L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Km(1, 11) = 6. * EIzz * ay / LL - k26 * by / L;
    Km(2, 2) = 12. * EIyy * az / LLL;
    Km(2, 3) = bz * k34 / L;
    Km(2, 4) = k35 * bz / L - 6. * EIyy * az / LL;
    Km(2, 5) = (L * bz * k36 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Km(2, 6) = -bz * k13 / L;
    Km(2, 7) = (-by * bz * k23 * LL + 12. * ay * az * k56) / LLL;
    Km(2, 8) = -12. * EIyy * az / LLL;
    Km(2, 9) = -bz * k34 / L;
    Km(2, 10) = -6. * EIyy * az / LL - k35 * bz / L;
    Km(2, 11) = -(L * bz * k36 + 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Km(3, 3) = GJ / L;
    Km(3, 4) = k45 / L - bz * k34 / 2.;
    Km(3, 5) = k46 / L + by * k24 / 2.;
    Km(3, 6) = -k14 / L;
    Km(3, 7) = -by * k24 / L;
    Km(3, 8) = -bz * k34 / L;
    Km(3, 9) = -GJ / L;
    Km(3, 10) = -k45 / L - bz * k34 / 2.;
    Km(3, 11) = -k46 / L + by * k24 / 2.;
    Km(4, 4) = EIyy * (4. + phiz) * az / L - bz * k35;
    Km(4, 5) = -L * by * bz * k23 / 4. + by * k25 / 2. - bz * k36 / 2. + (3. * ay * az + 1.) * k56 / L;
    Km(4, 6) = bz * k13 / 2. - k15 / L;
    Km(4, 7) = by * bz * k23 / 2. - (L * by * k25 + 6. * ay * az * k56) / LL;
    Km(4, 8) = 6. * EIyy * az / LL - k35 * bz / L;
    Km(4, 9) = bz * k34 / 2. - k45 / L;
    Km(4, 10) = EIyy * (2. - phiz) * az / L;
    Km(4, 11) = -L * by * bz * k23 / 4. + by * k25 / 2. + bz * k36 / 2. + (3. * ay * az - 1.) * k56 / L;
    Km(5, 5) = EIzz * (4. + phiy) * ay / L + by * k26;
    Km(5, 6) = -by * k12 / 2. - k16 / L;
    Km(5, 7) = -6. * EIzz * ay / LL - k26 * by / L;
    Km(5, 8) = -(L * bz * k36 - 6. * ay * az * k56) / LL - by * bz * k23 / 2.;
    Km(5, 9) = -by * k24 / 2. - k46 / L;
    Km(5, 10) = (3. * ay * az - 1.) * k56 / L - by * k25 / 2. - bz * k36 / 2. - L * bz * by * k23 / 4.;
    Km(5, 11) = EIzz * (2. - phiy) * ay / L;
    Km(6, 6) = EA / L;
    Km(6, 7) = by * k12 / L;
    Km(6, 8) = bz * k13 / L;
    Km(6, 9) = k14 / L;
    Km(6, 10) = bz * k13 / 2. + k15 / L;
    Km(6, 11) = k16 / L - by * k12 / 2.;
    Km(7, 7) = 12. * EIzz * ay / LLL;
    Km(7, 8) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Km(7, 9) = by * k24 / L;
    Km(7, 10) = (L * by * k25 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Km(7, 11) = -6. * EIzz * ay / LL + k26 * by / L;
    Km(8, 8) = 12. * EIyy * az / LLL;
    Km(8, 9) = bz * k34 / L;
    Km(8, 10) = 6. * EIyy * az / LL + k35 * bz / L;
    Km(8, 11) = (L * bz * k36 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Km(9, 9) = GJ / L;
    Km(9, 10) = bz * k34 / 2. + k45 / L;
    Km(9, 11) = k46 / L - by * k24 / 2.;
    Km(10, 10) = bz * k35 + EIyy * (4. + phiz) * az / L;
    Km(10, 11) = bz * k36 / 2. - by * k25 / 2. - L * by * bz * k23 / 4. + (3. * ay * az + 1.) * k56 / L;
    Km(11, 11) = EIzz * (4. + phiy) * ay / L - by * k26;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = 0; c < r; c++)
            Km(r, c) = Km(c, r);

    Km = this->T.transpose() * Km * this->T;
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeDampingMatrix0() {
    assert(tapered_section_fpm);

    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;

    double mbx = this->tapered_section_fpm->GetAverageSectionParameters()->rdamping_coeff.bx;
    double mby = this->tapered_section_fpm->GetAverageSectionParameters()->rdamping_coeff.by;
    double mbz = this->tapered_section_fpm->GetAverageSectionParameters()->rdamping_coeff.bz;
    double mbt = this->tapered_section_fpm->GetAverageSectionParameters()->rdamping_coeff.bt;
    ChMatrixNM<double, 6, 6> mb;
    mb.setIdentity();
    mb(0, 0) = mbx;
    mb(1, 1) = mby;
    mb(2, 2) = mbz;
    mb(3, 3) = mbt;
    mb(4, 4) = mbz;
    mb(5, 5) = mby;

    ChMatrixNM<double, 6, 6> Klaw = this->tapered_section_fpm->GetAverageFPM();
    ChMatrixNM<double, 6, 6> Rlaw = mb.transpose() * Klaw * mb;  // material damping matrix

    double rEA = Rlaw(0,0);
    double rGAyy = Rlaw(1, 1);
    double rGAzz = Rlaw(2, 2);
    double rGJ = Rlaw(3, 3);
    double rEIyy = Rlaw(4, 4);
    double rEIzz = Rlaw(5, 5);

    double phiy = this->tapered_section_fpm->GetAverageSectionParameters()->phiy;
    double phiz = this->tapered_section_fpm->GetAverageSectionParameters()->phiz;

    double ay = 1. / (1. + phiy);
    double by = phiy * ay;
    double az = 1. / (1. + phiz);
    double bz = phiz * az;


    double k12 = Rlaw(0, 1);
    double k13 = Rlaw(0, 2);
    double k14 = Rlaw(0, 3);
    double k15 = Rlaw(0, 4);
    double k16 = Rlaw(0, 5);
    double k23 = Rlaw(1, 2);
    double k24 = Rlaw(1, 3);
    double k25 = Rlaw(1, 4);
    double k26 = Rlaw(1, 5);
    double k34 = Rlaw(2, 3);
    double k35 = Rlaw(2, 4);
    double k36 = Rlaw(2, 5);
    double k45 = Rlaw(3, 4);
    double k46 = Rlaw(3, 5);
    double k56 = Rlaw(4, 5);

    Rm(0, 0) = rEA / L;
    Rm(0, 1) = by * k12 / L;
    Rm(0, 2) = bz * k13 / L;
    Rm(0, 3) = k14 / L;
    Rm(0, 4) = -bz * k13 / 2. + k15 / L;
    Rm(0, 5) = by * k12 / 2. + k16 / L;
    Rm(0, 6) = -rEA / L;
    Rm(0, 7) = -by * k12 / L;
    Rm(0, 8) = -bz * k13 / L;
    Rm(0, 9) = -k14 / L;
    Rm(0, 10) = -bz * k13 / 2. - k15 / L;
    Rm(0, 11) = by * k12 / 2. - k16 / L;
    Rm(1, 1) = 12. * rEIzz * ay / LLL;
    Rm(1, 2) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Rm(1, 3) = by * k24 / L;
    Rm(1, 4) = (L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Rm(1, 5) = 6. * rEIzz * ay / LL + k26 * by / L;
    Rm(1, 6) = -by * k12 / L;
    Rm(1, 7) = -12. * rEIzz * ay / LLL;
    Rm(1, 8) = -Rm(1, 2);
    Rm(1, 9) = -by * k24 / L;
    Rm(1, 10) = (-L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Rm(1, 11) = 6. * rEIzz * ay / LL - k26 * by / L;
    Rm(2, 2) = 12. * rEIyy * az / LLL;
    Rm(2, 3) = bz * k34 / L;
    Rm(2, 4) = k35 * bz / L - 6. * rEIyy * az / LL;
    Rm(2, 5) = (L * bz * k36 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Rm(2, 6) = -bz * k13 / L;
    Rm(2, 7) = (-by * bz * k23 * LL + 12. * ay * az * k56) / LLL;
    Rm(2, 8) = -12. * rEIyy * az / LLL;
    Rm(2, 9) = -bz * k34 / L;
    Rm(2, 10) = -6. * rEIyy * az / LL - k35 * bz / L;
    Rm(2, 11) = -(L * bz * k36 + 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Rm(3, 3) = rGJ / L;
    Rm(3, 4) = k45 / L - bz * k34 / 2.;
    Rm(3, 5) = k46 / L + by * k24 / 2.;
    Rm(3, 6) = -k14 / L;
    Rm(3, 7) = -by * k24 / L;
    Rm(3, 8) = -bz * k34 / L;
    Rm(3, 9) = -rGJ / L;
    Rm(3, 10) = -k45 / L - bz * k34 / 2.;
    Rm(3, 11) = -k46 / L + by * k24 / 2.;
    Rm(4, 4) = rEIyy * (4. + phiz) * az / L - bz * k35;
    Rm(4, 5) = -L * by * bz * k23 / 4. + by * k25 / 2. - bz * k36 / 2. + (3. * ay * az + 1.) * k56 / L;
    Rm(4, 6) = bz * k13 / 2. - k15 / L;
    Rm(4, 7) = by * bz * k23 / 2. - (L * by * k25 + 6. * ay * az * k56) / LL;
    Rm(4, 8) = 6. * rEIyy * az / LL - k35 * bz / L;
    Rm(4, 9) = bz * k34 / 2. - k45 / L;
    Rm(4, 10) = rEIyy * (2. - phiz) * az / L;
    Rm(4, 11) = -L * by * bz * k23 / 4. + by * k25 / 2. + bz * k36 / 2. + (3. * ay * az - 1.) * k56 / L;
    Rm(5, 5) = rEIzz * (4. + phiy) * ay / L + by * k26;
    Rm(5, 6) = -by * k12 / 2. - k16 / L;
    Rm(5, 7) = -6. * rEIzz * ay / LL - k26 * by / L;
    Rm(5, 8) = -(L * bz * k36 - 6. * ay * az * k56) / LL - by * bz * k23 / 2.;
    Rm(5, 9) = -by * k24 / 2. - k46 / L;
    Rm(5, 10) = (3. * ay * az - 1.) * k56 / L - by * k25 / 2. - bz * k36 / 2. - L * bz * by * k23 / 4.;
    Rm(5, 11) = rEIzz * (2. - phiy) * ay / L;
    Rm(6, 6) = rEA / L;
    Rm(6, 7) = by * k12 / L;
    Rm(6, 8) = bz * k13 / L;
    Rm(6, 9) = k14 / L;
    Rm(6, 10) = bz * k13 / 2. + k15 / L;
    Rm(6, 11) = k16 / L - by * k12 / 2.;
    Rm(7, 7) = 12. * rEIzz * ay / LLL;
    Rm(7, 8) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Rm(7, 9) = by * k24 / L;
    Rm(7, 10) = (L * by * k25 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Rm(7, 11) = -6. * rEIzz * ay / LL + k26 * by / L;
    Rm(8, 8) = 12. * rEIyy * az / LLL;
    Rm(8, 9) = bz * k34 / L;
    Rm(8, 10) = 6. * rEIyy * az / LL + k35 * bz / L;
    Rm(8, 11) = (L * bz * k36 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Rm(9, 9) = rGJ / L;
    Rm(9, 10) = bz * k34 / 2. + k45 / L;
    Rm(9, 11) = k46 / L - by * k24 / 2.;
    Rm(10, 10) = bz * k35 + rEIyy * (4. + phiz) * az / L;
    Rm(10, 11) = bz * k36 / 2. - by * k25 / 2. - L * by * bz * k23 / 4. + (3. * ay * az + 1.) * k56 / L;
    Rm(11, 11) = rEIzz * (4. + phiy) * ay / L - by * k26;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = 0; c < r; c++)
            Rm(r, c) = Rm(c, r);

    Rm = this->T.transpose() * Rm * this->T;
}

/// This class defines the calculations for the integrand of 
/// the cross-sectional stiffness/damping/mass matrices
class BeamTaperedTimoshenkoFPM : public ChIntegrable1D<ChMatrixNM<double, 12, 12>> {
  public:
    BeamTaperedTimoshenkoFPM(ChElementBeamTaperedTimoshenkoFPM* element, const int option)
        : m_element(element), m_choice_KiRiMi(option) {}
    ~BeamTaperedTimoshenkoFPM() {}

    void SetChoiceKiRiMi(int mv) { m_choice_KiRiMi = mv; }
    int GetChoiceKiRiMi() { return m_choice_KiRiMi; }

  private:
    ChElementBeamTaperedTimoshenkoFPM* m_element;
    // 0: stiffness matrix
    // 1: damping matrix
    // 2: mass matix
    int m_choice_KiRiMi = 0;

    virtual void Evaluate(ChMatrixNM<double, 12, 12>& result, const double x) override;
};

void BeamTaperedTimoshenkoFPM::Evaluate(ChMatrixNM<double, 12, 12>& result,const double x) {
    ChElementBeamTaperedTimoshenkoFPM::ShapeFunctionGroupFPM NxBx;
    double eta = x;
    m_element->ShapeFunctionsTimoshenkoFPM(NxBx,eta);
    auto tapered_section_fpm = m_element->GetTaperedSection();
    auto Klaw_point = tapered_section_fpm->GetKlawAtPoint(eta);
    auto Rlaw_point = tapered_section_fpm->GetRlawAtPoint(eta);
    auto Mlaw_point = tapered_section_fpm->GetMlawAtPoint(eta);

    // shape function matrix
    ChMatrixNM<double, 6, 12> Nx = std::get<0>(NxBx);
    // strain-displacement relation matrix
    ChMatrixNM<double, 6, 12> Bx = std::get<1>(NxBx);

    switch (m_choice_KiRiMi) {
        case 0:
            result = Bx.transpose() * Klaw_point * Bx;
            break;
        case 1:
            result = Bx.transpose() * Rlaw_point * Bx; // modified Rayleigh damping model
            break;
        case 2:
            result = Nx.transpose() * Mlaw_point * Nx;
            break;
        default:
            std::cout << "Please input the correct option: 0,1,2" <<std::endl;
            return;
    }
};

void ChElementBeamTaperedTimoshenkoFPM::ComputeStiffnessMatrix() {
    // Calculate the local element stiffness matrix via Guass integration
    this->Km.setZero();
    BeamTaperedTimoshenkoFPM myformula(this,0);  // 0: stiffness matrix
    ChMatrixNM<double, 12, 12> TempStiffnessMatrix;
    TempStiffnessMatrix.setZero();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(TempStiffnessMatrix,  // result of integration will go there
                                                          myformula,       // formula to integrate
                                                          -1, 1,           // x limits
                                                          guass_order      // order of integration
    );

    this->Km = this->T.transpose() * TempStiffnessMatrix * this->T;
}


void ChElementBeamTaperedTimoshenkoFPM::ComputeDampingMatrix() {
    // Calculate the local element damping matrix via Guass integration
    this->Rm.setZero();
    BeamTaperedTimoshenkoFPM myformula(this, 1);  // 1: damping matrix
    ChMatrixNM<double, 12, 12> TempDampingMatrix;
    TempDampingMatrix.setZero();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(TempDampingMatrix,  // result of integration will go there
                                                          myformula,            // formula to integrate
                                                          -1, 1,                // x limits
                                                          guass_order           // order of integration
    );

    this->Rm = this->T.transpose() * TempDampingMatrix * this->T;
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeConsistentMassMatrix() {
    // Calculate the local element mass matrix via Guass integration
    this->M.setZero();
    BeamTaperedTimoshenkoFPM myformula(this, 2);  // 2: mass matrix
    ChMatrixNM<double, 12, 12> TempMassMatrix;
    TempMassMatrix.setZero();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(TempMassMatrix,  // result of integration will go there
                                                          myformula,          // formula to integrate
                                                          -1, 1,              // x limits
                                                          guass_order         // order of integration
    );
    this->M = TempMassMatrix;
    // If the cross-sectional mass properties are gives at the mass center, 
    // then it should be transformed to the centerline firstly, 
    // this is handled in the Class ChBeamSectionTimoshenkoAdvancedGenericFPM. NOT HERE.
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeMassMatrix() {
    // Compute mass matrix of local element
    // It could be lumped or consistent mass matrix, depends on SetLumpedMassMatrix(true/false)
    if (this->tapered_section_fpm->GetLumpedMassMatrixType()) {
        // If it is lumped mass matrix, you need to multiple 0.5 * length to obtain the final mass matrix
        // For consistent mass matrix, don't need to multiple anything.
        this->tapered_section_fpm->ComputeInertiaMatrix(this->M);
    } else {
        ComputeConsistentMassMatrix();
    }
}


void ChElementBeamTaperedTimoshenkoFPM::SetupInitial(ChSystem* system) {
    assert(tapered_section_fpm);

    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
    this->mass = this->length / 2 * this->tapered_section_fpm->GetSectionA()->GetMassPerUnitLength() +
                 this->length / 2 * this->tapered_section_fpm->GetSectionB()->GetMassPerUnitLength();

    // Compute initial rotation
    ChMatrix33<> A0;
    ChVector<> mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
    ChVector<> myele = nodes[0]->GetX0().GetA().Get_A_Yaxis();
    A0.Set_A_Xdir(mXele, myele);
    q_element_ref_rot = A0.Get_A_quaternion();

    // Compute transformation matrix
    ComputeTransformMatrix();

    // Compute local mass matrix:
    ComputeMassMatrix();

    // Compute local stiffness matrix:
    ComputeStiffnessMatrix();

    // Compute local geometric stiffness matrix normalized by pull force P: Kg/P
    ComputeGeometricStiffnessMatrix();

    // Compute local damping matrix:
    ComputeDampingMatrix();
}


void ChElementBeamTaperedTimoshenkoFPM::EvaluateSectionDisplacement(const double eta,
                                                                    ChVector<>& u_displ,
                                                                    ChVector<>& u_rotaz) {

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);
    // No transformation for the displacement of two nodes,
    // so the section displacement is evaluated at the centerline of beam

    ShapeFunctionGroupFPM NxBx;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    // the shape function matrix
    ChMatrixNM<double, 6, 12> Nx = std::get<0>(NxBx);

    // the displacements and rotations, as a vector
    ChVectorDynamic<> u_vector = Nx * displ;

    u_displ.x() = u_vector(0);
    u_displ.y() = u_vector(1);
    u_displ.z() = u_vector(2);
    u_rotaz.x() = u_vector(3);
    u_rotaz.y() = u_vector(4);
    u_rotaz.z() = u_vector(5);

}

void ChElementBeamTaperedTimoshenkoFPM::EvaluateSectionForceTorque(const double eta,
                                                                ChVector<>& Fforce,
                                                                ChVector<>& Mtorque) {
    assert(tapered_section_fpm);

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    // transform the displacement of two nodes to elastic axis
    ChVectorDynamic<> displ_ec = this->T * displ;  

    ShapeFunctionGroupFPM NxBx;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    // the strain displacement matrix B:
    ChMatrixNM<double, 6, 12> Bx = std::get<1>(NxBx);

    // generalized strains/curvatures;
    ChVectorN<double, 6> sect_ek = Bx * displ_ec;

    // 6*6 fully populated constitutive matrix of the beam:
    ChMatrixNM<double, 6, 6> Klaw_d = this->tapered_section_fpm->GetKlawAtPoint(eta);

    ChMatrixDynamic<> Teta;
    ComputeTransformMatrixAtPoint(Teta, eta);

    // ..unrolled rotated constitutive matrix..
    ChMatrixNM<double, 6, 6> Klaw_r;
    Klaw_r.setZero();
    Klaw_r = Teta.transpose() * Klaw_d;

    // .. compute wrench = Klaw_r * sect_ek
    ChVectorN<double, 6> wrench = Klaw_r * sect_ek;
    Fforce = wrench.segment(0, 3);
    Mtorque = wrench.segment(3, 3);

}

void ChElementBeamTaperedTimoshenkoFPM::ComputeNF(const double U,
                                               ChVectorDynamic<>& Qi,
                                               double& detJ,
                                               const ChVectorDynamic<>& F,
                                               ChVectorDynamic<>* state_x,
                                               ChVectorDynamic<>* state_w) {
    ShapeFunctionGroupFPM NxBx;
    // the shape function matrix
    ChMatrixNM<double, 6, 12> Nx;

    double eta = -1;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    Nx = std::get<0>(NxBx);
    Qi.head(6) = Nx.transpose() * F;

    eta = 1;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    Nx = std::get<0>(NxBx);
    Qi.tail(6) = Nx.transpose() * F;

    // eta = 2*x/L;
    // ---> Deta/dx = 2./L;
    // ---> detJ = dx/Deta = L/2.;
    detJ = this->GetRestLength() / 2.0;

    /* old code for Euler beam
    Qi(0) = N(0) * F(0);  // Nx1 * Fx
    Qi(1) = N(1) * F(1) + N(6) * F(5);  // Ny1 * Fy + dN_ua * Mz
    Qi(2) = N(1) * F(2) - N(6) * F(4);  // Ny1 * Fz - dN_ua * My
    Qi(3) = N(0) * F(3);                // Nx1 * Mx
    Qi(4) = -N(2) * F(2) + N(8) * F(4);  // - Nr1 * Fz + dN_ra * My
    Qi(5) = N(2) * F(1) + N(8) * F(5);   // Nr1 * Fy +  dN_ra * Mz

    Qi(6) = N(3) * F(0);  // Nx2 * Fx
    Qi(7) = N(4) * F(1) + N(7) * F(5);  // Ny2 * Fy + dN_ub * Mz
    Qi(8) = N(4) * F(2) - N(7) * F(4);  // Ny2 * Fz - dN_ub * My
    Qi(9) = N(3) * F(3);                // Nx2 * Mx
    Qi(10) = -N(5) * F(2) + N(9) * F(4);  // - Nr2 * Fz + dN_rb * My
    Qi(11) = N(5) * F(1) + N(9) * F(5);   // Nr2 * Fy + dN_rb * Mz
    */
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeNF(const double U,
                                               const double V,
                                               const double W,
                                               ChVectorDynamic<>& Qi,
                                               double& detJ,
                                               const ChVectorDynamic<>& F,
                                               ChVectorDynamic<>* state_x,
                                               ChVectorDynamic<>* state_w) {
    this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
    detJ /= 4.0;  // because volume
}


}  // end namespace fea
}  // end namespace chrono
