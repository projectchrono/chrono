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

#include <cmath>

#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/core/ChMatrixMBD.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"

#include "chrono/utils/ChUtils.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------

void ChElasticityCosserat::ComputeStiffnessMatrix(ChMatrix66d& K,
                                                  const ChVector3d& strain_e,
                                                  const ChVector3d& strain_k) {
    double epsi = 1e-6;
    double invepsi = 1.0 / epsi;
    ChVector3d astress_n;
    ChVector3d astress_m;
    ChVector3d bstress_n;
    ChVector3d bstress_m;
    ChVector3d strain_e_inc = strain_e;
    ChVector3d strain_k_inc = strain_k;
    this->ComputeStress(astress_n, astress_m, strain_e, strain_k);
    for (int i = 0; i < 3; ++i) {
        strain_e_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        K.block(0, i, 3, 1) = (bstress_n - astress_n).eigen() * invepsi;
        K.block(3, i, 3, 1) = (bstress_m - astress_m).eigen() * invepsi;
        strain_e_inc[i] -= epsi;
    }
    for (int i = 0; i < 3; ++i) {
        strain_k_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        K.block(0, i + 3, 3, 3) = (bstress_n - astress_n).eigen() * invepsi;
        K.block(3, i + 3, 3, 3) = (bstress_m - astress_m).eigen() * invepsi;
        strain_k_inc[i] -= epsi;
    }
}

// -----------------------------------------------------------------------------

ChElasticityCosseratSimple::ChElasticityCosseratSimple()
    : E(0.01e9)  // default E stiffness (almost rubber)
{
    SetShearModulusFromPoisson(0.3);      // default G (low poisson ratio)
    SetAsRectangularSection(0.01, 0.01);  // defaults Area, Ixx, Iyy, Ks_y, Ks_z, J
}

void ChElasticityCosseratSimple::SetAsRectangularSection(double width_y, double width_z) {
    this->A = width_y * width_z;
    this->Izz = (1.0 / 12.0) * width_z * std::pow(width_y, 3);
    this->Iyy = (1.0 / 12.0) * width_y * std::pow(width_z, 3);

    // use Roark's formulas for torsion of rectangular sect:
    double t = std::min(width_y, width_z);
    double b = std::max(width_y, width_z);
    this->J = b * std::pow(t, 3) * (CH_1_3 - 0.210 * (t / b) * (1.0 - (1.0 / 12.0) * std::pow((t / b), 4)));

    // set Ks using Timoshenko-Gere formula for solid rect.shapes
    double poisson = this->E / (2.0 * this->G) - 1.0;
    this->Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
    this->Ks_z = this->Ks_y;
}

void ChElasticityCosseratSimple::SetAsCircularSection(double diameter) {
    this->A = CH_PI * std::pow((0.5 * diameter), 2);
    this->Izz = (CH_PI / 4.0) * std::pow((0.5 * diameter), 4);
    this->Iyy = Izz;

    // exact expression for circular beam J = Ixx ,
    // where for polar theorem Ixx = Izz+Iyy
    this->J = Izz + Iyy;

    // set Ks using Timoshenko-Gere formula for solid circular shape
    double poisson = this->E / (2.0 * this->G) - 1.0;
    this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
    this->Ks_z = this->Ks_y;
}

void ChElasticityCosseratSimple::ComputeStress(ChVector3d& stress_n,
                                               ChVector3d& stress_m,
                                               const ChVector3d& strain_n,
                                               const ChVector3d& strain_m) {
    stress_n.x() = E * A * strain_n.x();
    stress_n.y() = Ks_y * G * A * strain_n.y();
    stress_n.z() = Ks_z * G * A * strain_n.z();
    stress_m.x() = G * J * strain_m.x();
    stress_m.y() = E * Iyy * strain_m.y();
    stress_m.z() = E * Izz * strain_m.z();
}

void ChElasticityCosseratSimple::ComputeStiffnessMatrix(ChMatrix66d& K,
                                                        const ChVector3d& strain_n,
                                                        const ChVector3d& strain_m) {
    K.setZero(6, 6);
    K(0, 0) = E * A;
    K(1, 1) = Ks_y * G * A;
    K(2, 2) = Ks_z * G * A;
    K(3, 3) = G * J;
    K(4, 4) = E * Iyy;
    K(5, 5) = E * Izz;
}

// -----------------------------------------------------------------------------

ChElasticityCosseratGeneric::ChElasticityCosseratGeneric() {
    mE.setIdentity();  // default E stiffness: diagonal 1.
}

void ChElasticityCosseratGeneric::ComputeStress(ChVector3d& stress_n,
                                                ChVector3d& stress_m,
                                                const ChVector3d& strain_n,
                                                const ChVector3d& strain_m) {
    ChVectorN<double, 6> mstrain;
    ChVectorN<double, 6> mstress;
    mstrain.segment(0, 3) = strain_n.eigen();
    mstrain.segment(3, 3) = strain_m.eigen();
    mstress = this->mE * mstrain;
    stress_n = mstress.segment(0, 3);
    stress_m = mstress.segment(3, 3);
}

void ChElasticityCosseratGeneric::ComputeStiffnessMatrix(ChMatrix66d& K,
                                                         const ChVector3d& strain_n,
                                                         const ChVector3d& strain_m) {
    K = this->mE;
}

// -----------------------------------------------------------------------------

ChElasticityCosseratAdvanced::ChElasticityCosseratAdvanced() : alpha(0), Cy(0), Cz(0), beta(0), Sy(0), Sz(0) {}

void ChElasticityCosseratAdvanced::ComputeStress(ChVector3d& stress_n,
                                                 ChVector3d& stress_m,
                                                 const ChVector3d& strain_n,
                                                 const ChVector3d& strain_m) {
    double cos_alpha = std::cos(alpha);
    double sin_alpha = std::sin(alpha);
    double a11 = E * A;
    double a22 = E * (Iyy * std::pow(cos_alpha, 2.) + Izz * std::pow(sin_alpha, 2.) + Cz * Cz * A);
    double a33 = E * (Izz * std::pow(cos_alpha, 2.) + Iyy * std::pow(sin_alpha, 2.) + Cy * Cy * A);
    double a12 = Cz * E * A;
    double a13 = -Cy * E * A;
    double a23 = (E * Iyy - E * Izz) * cos_alpha * sin_alpha - E * Cy * Cz * A;
    stress_n.x() = a11 * strain_n.x() + a12 * strain_m.y() + a13 * strain_m.z();
    stress_m.y() = a12 * strain_n.x() + a22 * strain_m.y() + a23 * strain_m.z();
    stress_m.z() = a13 * strain_n.x() + a23 * strain_m.y() + a33 * strain_m.z();
    double cos_beta = std::cos(beta);
    double sin_beta = std::sin(beta);
    double KsyGA = Ks_y * G * A;
    double KszGA = Ks_z * G * A;
    double s11 = KsyGA * std::pow(cos_beta, 2.) + KszGA * std::pow(sin_beta, 2.);
    double s22 = KsyGA * std::pow(sin_beta, 2.) + KszGA * std::pow(cos_beta, 2.);  // ..+s_loc_12*sin(beta)*cos(beta);
    double s33 = G * J + Sz * Sz * KsyGA + Sy * Sy * KszGA;
    double s12 = (KszGA - KsyGA) * sin_beta * cos_beta;
    double s13 = Sy * KszGA * sin_beta - Sz * KsyGA * cos_beta;
    double s23 = Sy * KszGA * cos_beta + Sz * KsyGA * sin_beta;
    stress_n.y() = s11 * strain_n.y() + s12 * strain_n.z() + s13 * strain_m.x();
    stress_n.z() = s12 * strain_n.y() + s22 * strain_n.z() + s23 * strain_m.x();
    stress_m.x() = s13 * strain_n.y() + s23 * strain_n.z() + s33 * strain_m.x();
}

void ChElasticityCosseratAdvanced::ComputeStiffnessMatrix(ChMatrix66d& K,
                                                          const ChVector3d& strain_n,
                                                          const ChVector3d& strain_m) {
    K.setZero(6, 6);
    double cos_alpha = std::cos(alpha);
    double sin_alpha = std::sin(alpha);
    double a11 = E * A;
    double a22 = E * (Iyy * std::pow(cos_alpha, 2.) + Izz * std::pow(sin_alpha, 2.) + Cz * Cz * A);
    double a33 = E * (Izz * std::pow(cos_alpha, 2.) + Iyy * std::pow(sin_alpha, 2.) + Cy * Cy * A);
    double a12 = Cz * E * A;
    double a13 = -Cy * E * A;
    double a23 = (E * Iyy - E * Izz) * cos_alpha * sin_alpha - E * Cy * Cz * A;
    double cos_beta = std::cos(beta);
    double sin_beta = std::sin(beta);
    double KsyGA = Ks_y * G * A;
    double KszGA = Ks_z * G * A;
    double s11 = KsyGA * std::pow(cos_beta, 2.) + KszGA * std::pow(sin_beta, 2.);
    double s22 = KsyGA * std::pow(sin_beta, 2.) + KszGA * std::pow(cos_beta, 2.);  // ..+s_loc_12*sin(beta)*cos(beta);
    double s33 = G * J + Sz * Sz * KsyGA + Sy * Sy * KszGA;
    double s12 = (KszGA - KsyGA) * sin_beta * cos_beta;
    double s13 = Sy * KszGA * sin_beta - Sz * KsyGA * cos_beta;
    double s23 = Sy * KszGA * cos_beta + Sz * KsyGA * sin_beta;

    K(0, 0) = a11;
    K(0, 4) = a12;
    K(0, 5) = a13;
    K(1, 1) = s11;
    K(1, 2) = s12;
    K(1, 3) = s13;
    K(2, 1) = s12;
    K(2, 2) = s22;
    K(2, 3) = s23;
    K(3, 1) = s13;
    K(3, 2) = s23;
    K(3, 3) = s33;
    K(4, 0) = a12;
    K(4, 4) = a22;
    K(4, 5) = a23;
    K(5, 0) = a13;
    K(5, 4) = a23;
    K(5, 5) = a33;
}

// -----------------------------------------------------------------------------

void ChElasticityCosseratAdvancedGeneric::ComputeStress(ChVector3d& stress_n,
                                                        ChVector3d& stress_m,
                                                        const ChVector3d& strain_n,
                                                        const ChVector3d& strain_m) {
    double cos_alpha = std::cos(alpha);
    double sin_alpha = std::sin(alpha);
    double a11 = this->Ax;
    double a22 = this->Byy * std::pow(cos_alpha, 2.) + this->Bzz * std::pow(sin_alpha, 2.) + Cz * Cz * this->Ax;
    double a33 = this->Bzz * std::pow(cos_alpha, 2.) + this->Byy * std::pow(sin_alpha, 2.) + Cy * Cy * this->Ax;
    double a12 = Cz * this->Ax;
    double a13 = -Cy * this->Ax;
    double a23 = (this->Byy - this->Bzz) * cos_alpha * sin_alpha - Cy * Cz * this->Ax;
    stress_n.x() = a11 * strain_n.x() + a12 * strain_m.y() + a13 * strain_m.z();
    stress_m.y() = a12 * strain_n.x() + a22 * strain_m.y() + a23 * strain_m.z();
    stress_m.z() = a13 * strain_n.x() + a23 * strain_m.y() + a33 * strain_m.z();
    double cos_beta = std::cos(beta);
    double sin_beta = std::sin(beta);
    double KsyGA = this->Hyy;
    double KszGA = this->Hzz;
    double s11 = KsyGA * std::pow(cos_beta, 2.) + KszGA * std::pow(sin_beta, 2.);
    double s22 = KsyGA * std::pow(sin_beta, 2.) + KszGA * std::pow(cos_beta, 2.);  // ..+s_loc_12*sin(beta)*cos(beta);
    double s33 = this->Txx + Sz * Sz * KsyGA + Sy * Sy * KszGA;
    double s12 = (KszGA - KsyGA) * sin_beta * cos_beta;
    double s13 = Sy * KszGA * sin_beta - Sz * KsyGA * cos_beta;
    double s23 = Sy * KszGA * cos_beta + Sz * KsyGA * sin_beta;
    stress_n.y() = s11 * strain_n.y() + s12 * strain_n.z() + s13 * strain_m.x();
    stress_n.z() = s12 * strain_n.y() + s22 * strain_n.z() + s23 * strain_m.x();
    stress_m.x() = s13 * strain_n.y() + s23 * strain_n.z() + s33 * strain_m.x();
}

void ChElasticityCosseratAdvancedGeneric::ComputeStiffnessMatrix(ChMatrix66d& K,
                                                                 const ChVector3d& strain_n,
                                                                 const ChVector3d& strain_m) {
    K.setZero(6, 6);
    double cos_alpha = std::cos(alpha);
    double sin_alpha = std::sin(alpha);
    double a11 = this->Ax;
    double a22 = this->Byy * std::pow(cos_alpha, 2.) + this->Bzz * std::pow(sin_alpha, 2.) + Cz * Cz * this->Ax;
    double a33 = this->Bzz * std::pow(cos_alpha, 2.) + this->Byy * std::pow(sin_alpha, 2.) + Cy * Cy * this->Ax;
    double a12 = Cz * this->Ax;
    double a13 = -Cy * this->Ax;
    double a23 = (this->Byy - this->Bzz) * cos_alpha * sin_alpha - Cy * Cz * this->Ax;
    double cos_beta = std::cos(beta);
    double sin_beta = std::sin(beta);
    double KsyGA = this->Hyy;
    double KszGA = this->Hzz;
    double s11 = KsyGA * std::pow(cos_beta, 2.) + KszGA * std::pow(sin_beta, 2.);
    double s22 = KsyGA * std::pow(sin_beta, 2.) + KszGA * std::pow(cos_beta, 2.);  // ..+s_loc_12*sin(beta)*cos(beta);
    double s33 = this->Txx + Sz * Sz * KsyGA + Sy * Sy * KszGA;
    double s12 = (KszGA - KsyGA) * sin_beta * cos_beta;
    double s13 = Sy * KszGA * sin_beta - Sz * KsyGA * cos_beta;
    double s23 = Sy * KszGA * cos_beta + Sz * KsyGA * sin_beta;

    K(0, 0) = a11;
    K(0, 4) = a12;
    K(0, 5) = a13;
    K(1, 1) = s11;
    K(1, 2) = s12;
    K(1, 3) = s13;
    K(2, 1) = s12;
    K(2, 2) = s22;
    K(2, 3) = s23;
    K(3, 1) = s13;
    K(3, 2) = s23;
    K(3, 3) = s33;
    K(4, 0) = a12;
    K(4, 4) = a22;
    K(4, 5) = a23;
    K(5, 0) = a13;
    K(5, 4) = a23;
    K(5, 5) = a33;
}

// -----------------------------------------------------------------------------

void ChElasticityCosseratAdvancedGenericFPM::ComputeTransformMatrix() {
    // Initialization of the transformation matrix
    this->T.setIdentity();

    // In case the section is rotated:
    ChMatrix33<> RotsectA;
    RotsectA.SetFromCardanAnglesXYZ(ChVector3d(this->alpha, 0, 0));

    // In case the shear axis is rotated:
    ChMatrix33<> RotShearA;
    RotShearA.SetFromCardanAnglesXYZ(ChVector3d(this->beta, 0, 0));

    ChMatrix66d RotA;
    RotA.setZero();
    RotA.row(0) << RotsectA(0, 0), 0, 0, 0, RotsectA(0, 1), RotsectA(0, 2);
    RotA.row(1) << 0, RotShearA(0, 0), RotShearA(0, 1), RotShearA(0, 2), 0, 0;
    RotA.row(2) << 0, RotShearA(1, 0), RotShearA(1, 1), RotShearA(1, 2), 0, 0;
    RotA.row(3) << 0, RotShearA(2, 0), RotShearA(2, 1), RotShearA(2, 2), 0, 0;
    RotA.row(4) << RotsectA(1, 0), 0, 0, 0, RotsectA(1, 1), RotsectA(1, 2);
    RotA.row(5) << RotsectA(2, 0), 0, 0, 0, RotsectA(2, 1), RotsectA(2, 2);

    // In case the Elastic reference is offset to the centerline:
    ChMatrix66d Tc;
    Tc.setIdentity();
    Tc(0, 4) = Cz;
    Tc(0, 5) = -Cy;
    Tc(1, 3) = -Cz;
    Tc(2, 3) = Cy;

    // In case the Shear center is offset to the centerline:
    ChMatrix66d Ts;
    Ts.setIdentity();
    Ts(1, 3) = -Sz;
    Ts(2, 3) = Sy;

    this->T = RotA * Ts * Tc;
}

void ChElasticityCosseratAdvancedGenericFPM::UpdateStiffnessMatrix() {
    if (!updated) {  // do it only once
        // compute T
        ComputeTransformMatrix();
        // update Klaw after setting section rotation and EC/SC offset
        this->Klaw = this->T.transpose() * this->Klaw * this->T;
        updated = true;
    }
}

void ChElasticityCosseratAdvancedGenericFPM::ComputeStress(ChVector3d& stress_n,
                                                           ChVector3d& stress_m,
                                                           const ChVector3d& strain_n,
                                                           const ChVector3d& strain_m) {
    ChVectorN<double, 6> mstrain;
    ChVectorN<double, 6> mstress;
    mstrain.segment(0, 3) = strain_n.eigen();
    mstrain.segment(3, 3) = strain_m.eigen();
    mstress = this->Klaw * mstrain;
    stress_n = mstress.segment(0, 3);
    stress_m = mstress.segment(3, 3);
}

void ChElasticityCosseratAdvancedGenericFPM::ComputeStiffnessMatrix(ChMatrix66d& K,
                                                                    const ChVector3d& strain_n,
                                                                    const ChVector3d& strain_m) {
    K = this->Klaw;
}

// -----------------------------------------------------------------------------

void ChElasticityCosseratMesh::SetAsRectangularSection(double width_y, double width_z) {
    this->vertexes.clear();
    this->vertexes.push_back(ChVector2d(width_y * 0.5, width_z * 0.5));
    this->vertexes.push_back(ChVector2d(width_y * 0.5, -width_z * 0.5));
    this->vertexes.push_back(ChVector2d(-width_y * 0.5, -width_z * 0.5));
    this->vertexes.push_back(ChVector2d(-width_y * 0.5, width_z * 0.5));

    this->triangles.clear();
    this->triangles.push_back(ChVector3i(0, 1, 2));
    this->triangles.push_back(ChVector3i(0, 2, 3));

    // set Ks using Timoshenko-Gere formula for solid rect.shapes
    /*
    double poisson = this->E / (2.0 * this->G) - 1.0;
    this->Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
    this->Ks_z = this->Ks_y;
    */
}

void ChElasticityCosseratMesh::SetAsCircularSection(double diameter) {
    this->vertexes.clear();
    this->triangles.clear();

    double rad = diameter * 0.5;
    this->vertexes.push_back(ChVector2d(0, 0));
    this->vertexes.push_back(ChVector2d(rad, 0));
    int ntri = 12;
    for (int i = 0; i < ntri; ++i) {
        double alpha = (i + 1) * (CH_2PI / (double)ntri);
        this->vertexes.push_back(ChVector2d(rad * std::cos(alpha), rad * std::sin(alpha)));
        this->triangles.push_back(ChVector3i(0, i + 1, i + 2));
    }

    // set Ks using Timoshenko-Gere formula for solid circular shape
    /*
    double poisson = this->E / (2.0 * this->G) - 1.0;
    this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
    this->Ks_z = this->Ks_y;
    */
}

void ChElasticityCosseratMesh::ComputeStress(ChVector3d& stress_n,
                                             ChVector3d& stress_m,
                                             const ChVector3d& strain_n,
                                             const ChVector3d& strain_m) {
    int nv = (int)this->vertexes.size();
    int nt = (int)this->triangles.size();

    // temp per-vertex data for point strains:
    std::vector<double> epsilon_xx(nv);
    std::vector<double> gamma_xy(nv);
    std::vector<double> gamma_xz(nv);

    // temp per-vertex data for point stresses:
    std::vector<double> sigma_xx(nv);
    std::vector<double> sigma_xy(nv);
    std::vector<double> sigma_xz(nv);

    double warp_dy = 0;  // to do
    double warp_dz = 0;  // to do

    for (int i = 0; i < nv; ++i) {
        std::shared_ptr<ChSectionMaterial> mmat;
        if (materials.size() == 1)
            mmat = materials[0];
        else
            mmat = materials[i];

        double vy = vertexes[i][0];
        double vz = vertexes[i][1];
        epsilon_xx[i] = strain_n.x() + strain_m.y() * vz - strain_m.z() * vy;
        gamma_xy[i] = strain_n.y() + strain_m.x() * (warp_dy - vz);
        gamma_xz[i] = strain_n.z() + strain_m.x() * (warp_dz + vy);
        // simple linear elastic model:
        sigma_xx[i] = mmat->E * epsilon_xx[i];
        sigma_xy[i] = mmat->G * gamma_xy[i];
        sigma_xz[i] = mmat->G * gamma_xz[i];
    }

    // integrate on triangles, assuming linear interpolation of vertex values
    stress_n = VNULL;
    stress_m = VNULL;
    for (int t = 0; t < nt; ++t) {
        size_t iv1 = triangles[t].x();
        size_t iv2 = triangles[t].y();
        size_t iv3 = triangles[t].z();
        double y1 = this->vertexes[iv1][0];
        double z1 = this->vertexes[iv1][1];
        double y2 = this->vertexes[iv2][0];
        double z2 = this->vertexes[iv2][1];
        double y3 = this->vertexes[iv3][0];
        double z3 = this->vertexes[iv3][1];

        double A = fabs(0.5 * ((y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2))));

        double s1 = sigma_xx[iv1];
        double s2 = sigma_xx[iv2];
        double s3 = sigma_xx[iv3];
        double sxz1 = sigma_xz[iv1];
        double sxz2 = sigma_xz[iv2];
        double sxz3 = sigma_xz[iv3];
        double sxy1 = sigma_xy[iv1];
        double sxy2 = sigma_xy[iv2];
        double sxy3 = sigma_xy[iv3];

        stress_n.x() += CH_1_3 * A * (s1 + s2 + s3);
        stress_n.y() += CH_1_3 * A * (sxy1 + sxy2 + sxy3);
        stress_n.z() += CH_1_3 * A * (sxz1 + sxz2 + sxz3);

        stress_m.x() +=
            2. * A *
            ((sxz1 * y1) / 12. + (sxz1 * y2) / 24. + (sxz2 * y1) / 24. + (sxz1 * y3) / 24. + (sxz2 * y2) / 12. +
             (sxz3 * y1) / 24. + (sxz2 * y3) / 24. + (sxz3 * y2) / 24. + (sxz3 * y3) / 12. - (sxy1 * z1) / 12. -
             (sxy1 * z2) / 24. - (sxy2 * z1) / 24. - (sxy1 * z3) / 24. - (sxy2 * z2) / 12. - (sxy3 * z1) / 24. -
             (sxy2 * z3) / 24. - (sxy3 * z2) / 24. - (sxy3 * z3) / 12.);
        stress_m.y() += 2. * A *
                        ((s1 * z1) / 12. + (s1 * z2) / 24. + (s2 * z1) / 24. + (s1 * z3) / 24. + (s2 * z2) / 12. +
                         (s3 * z1) / 24. + (s2 * z3) / 24. + (s3 * z2) / 24. + (s3 * z3) / 12.);
        stress_m.z() -= 2. * A *
                        ((s1 * y1) / 12. + (s1 * y2) / 24. + (s2 * y1) / 24. + (s1 * y3) / 24. + (s2 * y2) / 12. +
                         (s3 * y1) / 24. + (s2 * y3) / 24. + (s3 * y2) / 24. + (s3 * y3) / 12.);
    }
}

// -----------------------------------------------------------------------------

ChPlasticityCosserat::ChPlasticityCosserat() : section(nullptr), nr_yeld_tolerance(1e-7), nr_yeld_maxiters(5) {}

void ChPlasticityCosserat::ComputeStiffnessMatrixElastoplastic(ChMatrix66d& K,
                                                               const ChVector3d& strain_n,
                                                               const ChVector3d& strain_m,
                                                               const ChBeamMaterialInternalData& data) {
    ChVector3d astress_n;
    ChVector3d astress_m;
    ChVector3d me_strain_n_new;  // needed only as placeholder
    ChVector3d me_strain_m_new;  // needed only as placeholder

    std::vector<std::unique_ptr<ChBeamMaterialInternalData>> a_plastic_data;
    this->CreatePlasticityData(1, a_plastic_data);
    std::vector<std::unique_ptr<ChBeamMaterialInternalData>> b_plastic_data;
    this->CreatePlasticityData(1, b_plastic_data);

    bool in_plastic = ComputeStressWithReturnMapping(astress_n, astress_m, me_strain_n_new, me_strain_m_new,
                                                     *a_plastic_data[0], strain_n, strain_m, data);

    if (!in_plastic) {
        // if no return mapping is needed at this strain state, just use elastic matrix:
        return this->section->GetElasticity()->ComputeStiffnessMatrix(K, strain_n, strain_m);
    } else {
        // if return mapping is needed at this strain state, compute the elastoplastic stiffness by brute force BDF
        double epsi = 1e-6;
        double invepsi = 1.0 / epsi;
        ChVector3d bstress_n;
        ChVector3d bstress_m;
        ChVector3d strain_n_inc = strain_n;
        ChVector3d strain_m_inc = strain_m;
        for (int i = 0; i < 3; ++i) {
            strain_n_inc[i] += epsi;
            this->ComputeStressWithReturnMapping(bstress_n, bstress_m, me_strain_n_new, me_strain_m_new,
                                                 *b_plastic_data[0], strain_n_inc, strain_m_inc, data);
            K.block(0, i, 3, 1) = (bstress_n - astress_n).eigen() * invepsi;
            K.block(3, i, 3, 1) = (bstress_m - astress_m).eigen() * invepsi;
            strain_n_inc[i] -= epsi;
        }
        for (int i = 0; i < 3; ++i) {
            strain_m_inc[i] += epsi;
            this->ComputeStressWithReturnMapping(bstress_n, bstress_m, me_strain_n_new, me_strain_m_new,
                                                 *b_plastic_data[0], strain_n_inc, strain_m_inc, data);
            K.block(0, i + 3, 3, 1) = (bstress_n - astress_n).eigen() * invepsi;
            K.block(3, i + 3, 3, 1) = (bstress_m - astress_m).eigen() * invepsi;
            strain_m_inc[i] -= epsi;
        }
    }
}

void ChPlasticityCosserat::CreatePlasticityData(
    int numpoints,
    std::vector<std::unique_ptr<ChBeamMaterialInternalData>>& plastic_data) {
    plastic_data.resize(numpoints);
    for (int i = 0; i < numpoints; ++i) {
        plastic_data[i] = std::unique_ptr<ChBeamMaterialInternalData>(new ChBeamMaterialInternalData());
    }
}

// -----------------------------------------------------------------------------

ChPlasticityCosseratLumped::ChPlasticityCosseratLumped() {
    // Default: linear isotropic constant hardening
    n_yeld_x = chrono_types::make_shared<ChFunctionConst>(1000);
    n_beta_x = chrono_types::make_shared<ChFunctionConst>(0);
    n_yeld_y = chrono_types::make_shared<ChFunctionConst>(1000);
    n_beta_y = chrono_types::make_shared<ChFunctionConst>(0);
    n_yeld_z = chrono_types::make_shared<ChFunctionConst>(1000);
    n_beta_z = chrono_types::make_shared<ChFunctionConst>(0);
    n_yeld_Mx = chrono_types::make_shared<ChFunctionConst>(1000);
    n_beta_Mx = chrono_types::make_shared<ChFunctionConst>(0);
    n_yeld_My = chrono_types::make_shared<ChFunctionConst>(1000);
    n_beta_My = chrono_types::make_shared<ChFunctionConst>(0);
    n_yeld_Mz = chrono_types::make_shared<ChFunctionConst>(1000);
    n_beta_Mz = chrono_types::make_shared<ChFunctionConst>(0);
}

bool ChPlasticityCosseratLumped::ComputeStressWithReturnMapping(ChVector3d& stress_n,
                                                                ChVector3d& stress_m,
                                                                ChVector3d& e_strain_e_new,
                                                                ChVector3d& e_strain_k_new,
                                                                ChBeamMaterialInternalData& data_new,
                                                                const ChVector3d& tot_strain_e,
                                                                const ChVector3d& tot_strain_k,
                                                                const ChBeamMaterialInternalData& data) {
    auto mydata = dynamic_cast<const ChInternalDataLumpedCosserat*>(&data);
    auto mydata_new = dynamic_cast<ChInternalDataLumpedCosserat*>(&data_new);

    if (!mydata)
        throw std::invalid_argument(
            "ComputeStressWithReturnMapping cannot cast data to ChInternalDataLumpedCosserat*.");

    // Implement return mapping for a simple 1D plasticity model.

    // Compute the elastic trial stress:
    e_strain_e_new = tot_strain_e - mydata->p_strain_e;
    e_strain_k_new = tot_strain_k - mydata->p_strain_k;
    // double p_strain_acc = mydata->p_strain_acc;
    this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);

    // axial direction
    {
        double strain_yeld_x = this->n_yeld_x->GetVal(mydata->p_strain_acc_e.x());     //<<<< sigma_y(p_strain_acc)
        double eta_x = stress_n.x() - this->n_beta_x->GetVal(mydata->p_strain_e.x());  //<<<< beta(p_strain_e)
        double Fyeld_x = fabs(eta_x) - strain_yeld_x;                                  //<<<<  Phi(sigma,p_strain_acc)

        if (Fyeld_x > 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_e.x() = mydata->p_strain_acc_e.x();
            mydata_new->p_strain_e.x() = mydata->p_strain_e.x();
            int iters = 0;
            while ((Fyeld_x > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_x = stress_n.x() / e_strain_e_new.x();  // instead of costly evaluation of Km, =dstress/dstrain
                double H = this->n_beta_x->GetDer(mydata->p_strain_e.x()) +
                           this->n_yeld_x->GetDer(mydata->p_strain_acc_e.x());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_x / (-E_x - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_e.x() += dDgamma;
                e_strain_e_new.x() -= dDgamma * chrono::ChSignum(stress_n.x());
                mydata_new->p_strain_e.x() += dDgamma * chrono::ChSignum(stress_n.x());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_x = this->n_yeld_x->GetVal(mydata_new->p_strain_acc_e.x());     //<<<< sigma_y(p_strain_acc)
                eta_x = stress_n.x() - this->n_beta_x->GetVal(mydata_new->p_strain_e.x());  //<<<< beta(p_strain_acc)
                Fyeld_x = fabs(eta_x) - strain_yeld_x;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // shear direction
    {
        double strain_yeld_y = this->n_yeld_y->GetVal(mydata->p_strain_acc_e.y());
        double eta_y = stress_n.y() - this->n_beta_y->GetVal(mydata->p_strain_e.y());
        double Fyeld_y = fabs(eta_y) - strain_yeld_y;  //<<<<  Phi(sigma,p_strain_acc)

        if (Fyeld_y < 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_e.y() = mydata->p_strain_acc_e.y();
            mydata_new->p_strain_e.y() = mydata->p_strain_e.y();
            int iters = 0;
            while ((Fyeld_y > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_y = stress_n.y() / e_strain_e_new.y();  // instead of costly evaluation of Km, =dstress/dstrain
                double H = this->n_beta_y->GetDer(mydata->p_strain_e.y()) +
                           this->n_yeld_y->GetDer(mydata->p_strain_acc_e.y());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_y / (-E_y - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_e.y() += dDgamma;
                e_strain_e_new.y() -= dDgamma * chrono::ChSignum(stress_n.y());
                mydata_new->p_strain_e.y() += dDgamma * chrono::ChSignum(stress_n.y());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_y = this->n_yeld_y->GetVal(mydata_new->p_strain_acc_e.y());     //<<<< sigma_y(p_strain_acc)
                eta_y = stress_n.y() - this->n_beta_y->GetVal(mydata_new->p_strain_e.y());  //<<<< beta(p_strain_acc)
                Fyeld_y = fabs(eta_y) - strain_yeld_y;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // shear direction
    {
        double strain_yeld_z = this->n_yeld_z->GetVal(mydata->p_strain_acc_e.z());
        double eta_z = stress_n.z() - this->n_beta_z->GetVal(mydata->p_strain_e.z());
        double Fyeld_z = fabs(eta_z) - strain_yeld_z;  //<<<<  Phi(sigma,p_strain_acc)

        if (Fyeld_z > 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_e.z() = mydata->p_strain_acc_e.z();
            mydata_new->p_strain_e.z() = mydata->p_strain_e.z();
            int iters = 0;
            while ((Fyeld_z > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_z = stress_n.z() / e_strain_e_new.z();  // instead of costly evaluation of Km, =dstress/dstrain
                double H = this->n_beta_z->GetDer(mydata->p_strain_e.z()) +
                           this->n_yeld_z->GetDer(mydata->p_strain_acc_e.z());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_z / (-E_z - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_e.z() += dDgamma;
                e_strain_e_new.z() -= dDgamma * chrono::ChSignum(stress_n.z());
                mydata_new->p_strain_e.z() += dDgamma * chrono::ChSignum(stress_n.z());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_z = this->n_yeld_z->GetVal(mydata_new->p_strain_acc_e.z());     //<<<< sigma_y(p_strain_acc)
                eta_z = stress_n.z() - this->n_beta_z->GetVal(mydata_new->p_strain_e.z());  //<<<< beta(p_strain_acc)
                Fyeld_z = fabs(eta_z) - strain_yeld_z;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // torsion direction
    {
        double strain_yeld_Mx = this->n_yeld_Mx->GetVal(mydata->p_strain_acc_k.x());
        double eta_Mx = stress_m.x() - this->n_beta_Mx->GetVal(mydata->p_strain_k.x());
        double Fyeld_Mx = fabs(eta_Mx) - strain_yeld_Mx;

        if (Fyeld_Mx > 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_k.x() = mydata->p_strain_acc_k.x();
            mydata_new->p_strain_k.x() = mydata->p_strain_k.x();
            int iters = 0;
            while ((Fyeld_Mx > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_Mx = stress_m.x() / e_strain_k_new.x();  // instead of costly evaluation of Km,
                                                                  // =dstress/dstrain
                double H = this->n_beta_Mx->GetDer(mydata->p_strain_k.x()) +
                           this->n_yeld_Mx->GetDer(mydata->p_strain_acc_k.x());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_Mx / (-E_Mx - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_k.x() += dDgamma;
                e_strain_k_new.x() -= dDgamma * chrono::ChSignum(stress_m.x());
                mydata_new->p_strain_k.x() += dDgamma * chrono::ChSignum(stress_m.x());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_Mx = this->n_yeld_Mx->GetVal(mydata_new->p_strain_acc_k.x());  //<<<< sigma_y(p_strain_acc)
                eta_Mx = stress_m.x() - this->n_beta_Mx->GetVal(mydata_new->p_strain_k.x());  //<<<< beta(p_strain_acc)
                Fyeld_Mx = fabs(eta_Mx) - strain_yeld_Mx;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // bending y direction
    {
        double strain_yeld_My = this->n_yeld_My->GetVal(mydata->p_strain_acc_k.y());
        double eta_My = stress_m.y() - this->n_beta_My->GetVal(mydata->p_strain_k.y());
        double Fyeld_My = fabs(eta_My) - strain_yeld_My;

        if (Fyeld_My > 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_k.y() = mydata->p_strain_acc_k.y();
            mydata_new->p_strain_k.y() = mydata->p_strain_k.y();
            int iters = 0;
            while ((Fyeld_My > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_My = stress_m.y() / e_strain_k_new.y();  // instead of costly evaluation of Km,
                                                                  // =dstress/dstrain
                double H = this->n_beta_My->GetDer(mydata->p_strain_k.y()) +
                           this->n_yeld_My->GetDer(mydata->p_strain_acc_k.y());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_My / (-E_My - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_k.y() += dDgamma;
                e_strain_k_new.y() -= dDgamma * chrono::ChSignum(stress_m.y());
                mydata_new->p_strain_k.y() += dDgamma * chrono::ChSignum(stress_m.y());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_My = this->n_yeld_My->GetVal(mydata_new->p_strain_acc_k.y());  //<<<< sigma_y(p_strain_acc)
                eta_My = stress_m.y() - this->n_beta_My->GetVal(mydata_new->p_strain_k.y());  //<<<< beta(p_strain_acc)
                Fyeld_My = fabs(eta_My) - strain_yeld_My;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // bending z direction
    {
        double strain_yeld_Mz = this->n_yeld_Mz->GetVal(mydata->p_strain_acc_k.z());
        double eta_Mz = stress_m.z() - this->n_beta_Mz->GetVal(mydata->p_strain_k.z());
        double Fyeld_Mz = fabs(eta_Mz) - strain_yeld_Mz;

        if (Fyeld_Mz > 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_k.z() = mydata->p_strain_acc_k.z();
            mydata_new->p_strain_k.z() = mydata->p_strain_k.z();
            int iters = 0;
            while ((Fyeld_Mz > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_Mz = stress_m.z() / e_strain_k_new.z();  // instead of costly evaluation of Km,
                                                                  // =dstress/dstrain
                double H = this->n_beta_Mz->GetDer(mydata->p_strain_k.z()) +
                           this->n_yeld_Mz->GetDer(mydata->p_strain_acc_k.z());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_Mz / (-E_Mz - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_k.z() += dDgamma;
                e_strain_k_new.z() -= dDgamma * chrono::ChSignum(stress_m.z());
                mydata_new->p_strain_k.z() += dDgamma * chrono::ChSignum(stress_m.z());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_Mz = this->n_yeld_Mz->GetVal(mydata_new->p_strain_acc_k.z());  //<<<< sigma_y(p_strain_acc)
                eta_Mz = stress_m.z() - this->n_beta_Mz->GetVal(mydata_new->p_strain_k.z());  //<<<< beta(p_strain_acc)
                Fyeld_Mz = fabs(eta_Mz) - strain_yeld_Mz;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }
    // te scalar plastic accumulator in this case is the sum of the single accumulators of the various degreees of
    // freedom of the beam:
    mydata_new->p_strain_acc = mydata_new->p_strain_acc_e.x() + mydata_new->p_strain_acc_e.y() +
                               mydata_new->p_strain_acc_e.z() + mydata_new->p_strain_acc_k.x() +
                               mydata_new->p_strain_acc_k.y() + mydata_new->p_strain_acc_k.z();

    return true;
};

void ChPlasticityCosseratLumped::CreatePlasticityData(
    int numpoints,
    std::vector<std::unique_ptr<ChBeamMaterialInternalData>>& plastic_data) {
    plastic_data.resize(numpoints);
    for (int i = 0; i < numpoints; ++i) {
        plastic_data[i] = std::unique_ptr<ChBeamMaterialInternalData>(new ChInternalDataLumpedCosserat());
    }
}

// -----------------------------------------------------------------------------

void ChDampingCosserat::ComputeDampingMatrix(ChMatrix66d& R, const ChVector3d& dstrain_e, const ChVector3d& dstrain_k) {
    double epsi = 1e-6;
    double invepsi = 1.0 / epsi;
    ChVector3d astress_n;
    ChVector3d astress_m;
    ChVector3d bstress_n;
    ChVector3d bstress_m;
    ChVector3d strain_e_inc = dstrain_e;
    ChVector3d strain_k_inc = dstrain_k;
    this->ComputeStress(astress_n, astress_m, dstrain_e, dstrain_k);
    for (int i = 0; i < 3; ++i) {
        strain_e_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        R.block(0, i, 3, 1) = (bstress_n - astress_n).eigen() * invepsi;
        R.block(3, i, 3, 1) = (bstress_m - astress_m).eigen() * invepsi;
    }
    for (int i = 0; i < 3; ++i) {
        strain_k_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        R.block(0, i + 3, 3, 1) = (bstress_n - astress_n).eigen() * invepsi;
        R.block(3, i + 3, 3, 1) = (bstress_m - astress_m).eigen() * invepsi;
        strain_k_inc[i] -= epsi;
    }
}

// -----------------------------------------------------------------------------

void ChDampingCosseratLinear::ComputeStress(ChVector3d& stress_n,
                                            ChVector3d& stress_m,
                                            const ChVector3d& dstrain_e,
                                            const ChVector3d& dstrain_k) {
    stress_n.x() = dstrain_e.x() * R_e.x();
    stress_n.y() = dstrain_e.y() * R_e.y();
    stress_n.z() = dstrain_e.z() * R_e.z();
    stress_m.x() = dstrain_k.x() * R_k.x();
    stress_m.y() = dstrain_k.y() * R_k.y();
    stress_m.z() = dstrain_k.z() * R_k.z();
}

void ChDampingCosseratLinear::ComputeDampingMatrix(ChMatrix66d& R,
                                                   const ChVector3d& dstrain_e,
                                                   const ChVector3d& dstrain_k) {
    R.setZero();
    R(0, 0) = R_e.x();
    R(1, 1) = R_e.y();
    R(2, 2) = R_e.z();
    R(3, 3) = R_k.x();
    R(4, 4) = R_k.y();
    R(5, 5) = R_k.z();
}

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

ChDampingCosseratRayleigh::ChDampingCosseratRayleigh(std::shared_ptr<ChElasticityCosserat> melasticity,
                                                     const double& mbeta) {
    this->beta = mbeta;
    this->section_elasticity = melasticity;
    this->updated = false;
}

void ChDampingCosseratRayleigh::ComputeStress(ChVector3d& stress_n,
                                              ChVector3d& stress_m,
                                              const ChVector3d& dstrain_e,
                                              const ChVector3d& dstrain_k) {
    if (!this->updated) {
        this->UpdateStiffnessModel();
        this->updated = true;
    }
    ChVectorN<double, 6> mdstrain;
    ChVectorN<double, 6> mstress;
    mdstrain.segment(0, 3) = dstrain_e.eigen();
    mdstrain.segment(3, 3) = dstrain_k.eigen();
    mstress = this->beta * this->E_const * mdstrain;
    stress_n = mstress.segment(0, 3);
    stress_m = mstress.segment(3, 3);
}

void ChDampingCosseratRayleigh::ComputeDampingMatrix(ChMatrix66d& R,
                                                     const ChVector3d& dstrain_e,
                                                     const ChVector3d& dstrain_k) {
    R = this->beta * this->E_const;
}

void ChDampingCosseratRayleigh::UpdateStiffnessModel() {
    // Precompute and store the stiffness matrix into E_const, assuming
    // initial zero stress and zero strain, and use it as constant E from now on
    // (for many stiffness models, this is constant anyway)
    this->section_elasticity->ComputeStiffnessMatrix(this->E_const, VNULL, VNULL);
}

//-----------------------------------------------------------------------------

void ChInertiaCosserat::ComputeInertiaDampingMatrix(
    ChMatrix66d& Ri,      // 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
    const ChVector3d& mW  // current angular velocity of section, in material frame
) {
    double Delta = 1e-8;  // magic number, todo: parametrize or #define
    Ri.setZero();

    if (compute_inertia_damping_matrix == false)
        return;

    // Fi=Fia+Fiv, where Fia depends on acceleration only, so restrict to Fiv quadratic terms for numerical
    // differentiation. Also we assume first three columns of Ri are null because Fiv does not depend on linear
    // velocity. Quadratic terms (gyro, centrifugal) at current state:
    ChVectorN<double, 6> Fi0;
    ChVector3d mF, mT;
    this->ComputeQuadraticTerms(mF, mT, mW);
    Fi0.segment(0, 3) = mF.eigen();
    Fi0.segment(3, 3) = mT.eigen();
    // dw_x
    ChVectorN<double, 6> Fi_dw;
    this->ComputeQuadraticTerms(mF, mT, mW + ChVector3d(Delta, 0, 0));
    Fi_dw.segment(0, 3) = mF.eigen();
    Fi_dw.segment(3, 3) = mT.eigen();
    Ri.block(0, 3, 6, 1) = (Fi_dw - Fi0) * (1.0 / Delta);
    // dw_y
    this->ComputeQuadraticTerms(mF, mT, mW + ChVector3d(0, Delta, 0));
    Fi_dw.segment(0, 3) = mF.eigen();
    Fi_dw.segment(3, 3) = mT.eigen();
    Ri.block(0, 4, 6, 1) = (Fi_dw - Fi0) * (1.0 / Delta);
    // dw_z
    this->ComputeQuadraticTerms(mF, mT, mW + ChVector3d(0, 0, Delta));
    Fi_dw.segment(0, 3) = mF.eigen();
    Fi_dw.segment(3, 3) = mT.eigen();
    Ri.block(0, 5, 6, 1) = (Fi_dw - Fi0) * (1.0 / Delta);
}

void ChInertiaCosserat::ComputeInertiaStiffnessMatrix(
    ChMatrix66d& Ki,          // 6x6 sectional inertial-stiffness matrix values here
    const ChVector3d& mWvel,  // current angular velocity of section, in material frame
    const ChVector3d& mWacc,  // current angular acceleration of section, in material frame
    const ChVector3d& mXacc   // current acceleration of section, in material frame (not absolute!)
) {
    double Delta = 1e-8;  // magic number, todo: parametrize or #define
    Ki.setZero();

    if (compute_inertia_stiffness_matrix == false)
        return;

    // We assume first three columns of Ki are null because Fi does not depend on displacement.
    // We compute Ki by numerical differentiation.

    ChVector3d mF, mT;
    this->ComputeInertialForce(mF, mT, mWvel, mWacc, mXacc);
    ChVectorN<double, 6> Fi0;
    Fi0.segment(0, 3) = mF.eigen();
    Fi0.segment(3, 3) = mT.eigen();

    ChVectorN<double, 6> Fi_dr;
    ChVectorN<double, 6> drFi;

    // dr_x
    ChStarMatrix33<> rot_lx(ChVector3d(Delta, 0, 0));
    rot_lx.diagonal().setOnes();
    this->ComputeInertialForce(
        mF, mT,
        mWvel,  // or rot_lx.transpose()*mWvel,  if abs. ang.vel is constant during rot.increments
        mWacc,  // or rot_lx.transpose()*mWacc,  if abs. ang.vel is constant during rot.increments
        rot_lx.transpose() * mXacc);
    Fi_dr.segment(0, 3) = mF.eigen();
    Fi_dr.segment(3, 3) = mT.eigen();
    drFi.segment(0, 3) = rot_lx * Fi0.segment(0, 3);
    drFi.segment(3, 3) = Fi0.segment(3, 3);
    Ki.block(0, 3, 6, 1) = (Fi_dr - Fi0) * (1.0 / Delta) + (drFi - Fi0) * (1.0 / Delta);

    // dr_y
    ChStarMatrix33<> rot_ly(ChVector3d(0, Delta, 0));
    rot_ly.diagonal().setOnes();
    this->ComputeInertialForce(mF, mT, mWvel, mWacc, rot_ly.transpose() * mXacc);
    Fi_dr.segment(0, 3) = mF.eigen();
    Fi_dr.segment(3, 3) = mT.eigen();
    drFi.segment(0, 3) = rot_ly * Fi0.segment(0, 3);
    drFi.segment(3, 3) = Fi0.segment(3, 3);
    Ki.block(0, 4, 6, 1) = (Fi_dr - Fi0) * (1.0 / Delta) + (drFi - Fi0) * (1.0 / Delta);

    // dr_z
    ChStarMatrix33<> rot_lz(ChVector3d(0, 0, Delta));
    rot_lz.diagonal().setOnes();
    this->ComputeInertialForce(mF, mT, mWvel, mWacc, rot_lz.transpose() * mXacc);
    Fi_dr.segment(0, 3) = mF.eigen();
    Fi_dr.segment(3, 3) = mT.eigen();
    drFi.segment(0, 3) = rot_lz * Fi0.segment(0, 3);
    drFi.segment(3, 3) = Fi0.segment(3, 3);
    Ki.block(0, 5, 6, 1) = (Fi_dr - Fi0) * (1.0 / Delta) + (drFi - Fi0) * (1.0 / Delta);
}

void ChInertiaCosserat::ComputeInertialForce(
    ChVector3d& mFi,          // total inertial force returned here
    ChVector3d& mTi,          // total inertial torque returned here
    const ChVector3d& mWvel,  // current angular velocity of section, in material frame
    const ChVector3d& mWacc,  // current angular acceleration of section, in material frame
    const ChVector3d& mXacc   // current acceleration of section, in material frame (not absolute!)
) {
    // Default implementation as Fi = [Mi]*{xacc,wacc}+{mF_quadratic,mT_quadratic}
    // but if possible implement it in children classes with ad-hoc faster formulas.
    ChMatrix66d Mi;
    this->ComputeInertiaMatrix(Mi);
    ChVectorN<double, 6> xpp;
    xpp.segment(0, 3) = mXacc.eigen();
    xpp.segment(3, 3) = mWacc.eigen();
    ChVectorN<double, 6> Fipp = Mi * xpp;  // [Mi]*{xacc,wacc}
    ChVector3d mF_quadratic;
    ChVector3d mT_quadratic;
    this->ComputeQuadraticTerms(mF_quadratic, mT_quadratic, mWvel);  // {mF_quadratic,mT_quadratic}
    mFi = ChVector3d(Fipp.segment(0, 3)) + mF_quadratic;
    mTi = ChVector3d(Fipp.segment(3, 3)) + mT_quadratic;
}

//-----------------------------------------------------------------------------

void ChInertiaCosseratSimple::ComputeInertiaMatrix(ChMatrix66d& M) {
    M.setZero();
    M(0, 0) = this->GetMassPerUnitLength();
    M(1, 1) = this->GetMassPerUnitLength();
    M(2, 2) = this->GetMassPerUnitLength();
    M(3, 3) = this->GetInertiaJxxPerUnitLength();
    M(4, 4) = this->GetInertiaJyyPerUnitLength();
    M(5, 5) = this->GetInertiaJzzPerUnitLength();
}

void ChInertiaCosseratSimple::ComputeInertiaDampingMatrix(
    ChMatrix66d& Ri,      // 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
    const ChVector3d& mW  // current angular velocity of section, in material frame
) {
    Ri.setZero();
    if (compute_inertia_damping_matrix == false)
        return;
    if (this->compute_Ri_Ki_by_num_diff)
        return ChInertiaCosserat::ComputeInertiaDampingMatrix(Ri, mW);

    ChStarMatrix33<> wtilde(mW);  // [w~]
    ChMatrix33<> mI(ChVector3d(this->GetInertiaJxxPerUnitLength(), this->GetInertiaJyyPerUnitLength(),
                               this->GetInertiaJzzPerUnitLength()));  // [I]  here just a diagonal inertia
    Ri.block<3, 3>(3, 3) = wtilde * mI - ChStarMatrix33<>(mI * mW);   // Ri = [0, 0; 0, [w~][I] - [([I]*w)~]  ]
}

void ChInertiaCosseratSimple::ComputeInertiaStiffnessMatrix(
    ChMatrix66d& Ki,          // 6x6 sectional inertial-stiffness matrix values here
    const ChVector3d& mWvel,  // current angular velocity of section, in material frame
    const ChVector3d& mWacc,  // current angular acceleration of section, in material frame
    const ChVector3d& mXacc   // current acceleration of section, in material frame
) {
    Ki.setZero();
    if (compute_inertia_stiffness_matrix == false)
        return;
    if (this->compute_Ri_Ki_by_num_diff)
        return ChInertiaCosserat::ComputeInertiaStiffnessMatrix(Ki, mWvel, mWacc, mXacc);
    // null [Ki^] (but only for the case where angular speeds and accelerations are assumed to corotate with local
    // frames.
}

void ChInertiaCosseratSimple::ComputeQuadraticTerms(
    ChVector3d& mF,       // centrifugal term (if any) returned here
    ChVector3d& mT,       // gyroscopic term  returned here
    const ChVector3d& mW  // current angular velocity of section, in material frame
) {
    mF = VNULL;
    mT = Vcross(mW, ChVector3d(this->GetInertiaJxxPerUnitLength() * mW.x(), this->GetInertiaJyyPerUnitLength() * mW.y(),
                               this->GetInertiaJzzPerUnitLength() * mW.z()));
}

void ChInertiaCosseratSimple::SetAsRectangularSection(double width_y, double width_z, double density) {
    this->A = width_y * width_z;
    this->Izz = (1.0 / 12.0) * width_z * std::pow(width_y, 3);
    this->Iyy = (1.0 / 12.0) * width_y * std::pow(width_z, 3);
    this->rho = density;
}

void ChInertiaCosseratSimple::SetAsCircularSection(double diameter, double density) {
    this->A = CH_PI * std::pow((0.5 * diameter), 2);
    this->Izz = (CH_PI / 4.0) * std::pow((0.5 * diameter), 4);
    this->Iyy = Izz;
    this->rho = density;
}

// -----------------------------------------------------------------------------

void ChInertiaCosseratAdvanced::ComputeInertiaMatrix(ChMatrix66d& M) {
    M.setZero();
    M(0, 0) = this->mu;
    M(1, 1) = this->mu;
    M(2, 2) = this->mu;

    M(3, 1) = -this->mu * this->cm_z;
    M(3, 2) = this->mu * this->cm_y;
    M(4, 0) = this->mu * this->cm_z;
    M(5, 0) = -this->mu * this->cm_y;

    M(1, 3) = -this->mu * this->cm_z;
    M(2, 3) = this->mu * this->cm_y;
    M(0, 4) = this->mu * this->cm_z;
    M(0, 5) = -this->mu * this->cm_y;

    M(3, 3) = this->Jyy + this->Jzz;
    M(4, 4) = this->Jyy;
    M(5, 5) = this->Jzz;
    M(4, 5) = -this->Jyz;
    M(5, 4) = -this->Jyz;
}

void ChInertiaCosseratAdvanced::ComputeInertiaDampingMatrix(
    ChMatrix66d& Ri,      // 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
    const ChVector3d& mW  // current angular velocity of section, in material frame
) {
    Ri.setZero();
    if (compute_inertia_damping_matrix == false)
        return;
    if (this->compute_Ri_Ki_by_num_diff)
        return ChInertiaCosserat::ComputeInertiaDampingMatrix(Ri, mW);

    ChStarMatrix33<> wtilde(mW);  // [w~]
    ChVector3d mC(0, this->cm_y, this->cm_z);
    ChStarMatrix33<> ctilde(mC);  // [c~]
    ChMatrix33<> mI;
    mI << this->Jyy + this->Jzz, 0, 0, 0, this->Jyy, -this->Jyz, 0, -this->Jyz, this->Jzz;
    //  Ri = [0,  m*[w~][c~]' + m*[([w~]*c)~]'  ; 0 , [w~][I] - [([I]*w)~]  ]
    Ri.block<3, 3>(0, 3) = this->mu * (wtilde * ctilde.transpose() + (ChStarMatrix33<>(wtilde * mC)).transpose());
    Ri.block<3, 3>(3, 3) = wtilde * mI - ChStarMatrix33<>(mI * mW);
}

void ChInertiaCosseratAdvanced::ComputeInertiaStiffnessMatrix(
    ChMatrix66d& Ki,          // 6x6 sectional inertial-stiffness matrix values here
    const ChVector3d& mWvel,  // current angular velocity of section, in material frame
    const ChVector3d& mWacc,  // current angular acceleration of section, in material frame
    const ChVector3d& mXacc   // current acceleration of section, in material frame
) {
    Ki.setZero();
    if (compute_inertia_stiffness_matrix == false)
        return;
    if (this->compute_Ri_Ki_by_num_diff)
        return ChInertiaCosserat::ComputeInertiaStiffnessMatrix(Ki, mWvel, mWacc, mXacc);

    ChStarMatrix33<> wtilde(mWvel);  // [w~]
    ChStarMatrix33<> atilde(mWacc);  // [a~]
    ChVector3d mC(0, this->cm_y, this->cm_z);
    ChStarMatrix33<> ctilde(mC);  // [c~]
    ChMatrix33<> mI;
    mI << this->Jyy + this->Jzz, 0, 0, 0, this->Jyy, -this->Jyz, 0, -this->Jyz, this->Jzz;
    // case A: where absolute ang.vel and ang.acc are constant if the frame rotates (as in Bauchau)
    // and the local ang.vel and ang.acc will counterrotate:
    // for mixed absolute (translation) and local (rotation) bases one has:
    // Ki_al = [0,  m*([a~]+[w~][w~])[c~]'; 0,  m*[c~][xpp~] + [I][a~]  + [w~]([I][w~] - [([I]*w)~]) +[([w~][I]*w)~]  ]
    /*
    Ki.block<3, 3>(0, 3) = this->mu * (atilde + wtilde* wtilde) * ctilde.transpose();
    Ki.block<3, 3>(3, 3) = this->mu * ctilde * ChStarMatrix33<>(mXacc)
                          + (mI * atilde )
                          + wtilde * (mI * wtilde  - ChStarMatrix33<>(mI * mWvel))
                          + ChStarMatrix33<>(wtilde*(mI*mWvel));
    */
    // case B: where local ang.vel and ang.acc are constant if the frame rotates (as in Chrono)
    // and the absolute ang.vel and ang.acc will rotate:
    // for mixed absolute (translation) and local (rotation) bases one has:
    // Ki_al = [0,  -m*[([a~]c)~] -m*[([w~][w~]c)~] ; 0,  m*[c~][xpp~] ]
    Ki.block<3, 3>(0, 3) =
        -this->mu * ChStarMatrix33<>(atilde * mC) - this->mu * ChStarMatrix33<>(wtilde * (wtilde * mC));
    Ki.block<3, 3>(3, 3) = this->mu * ctilde * ChStarMatrix33<>(mXacc);
}

void ChInertiaCosseratAdvanced::ComputeQuadraticTerms(
    ChVector3d& mF,       // centrifugal term (if any) returned here
    ChVector3d& mT,       // gyroscopic term  returned here
    const ChVector3d& mW  // current angular velocity of section, in material frame
) {
    // F_centrifugal = density_per_unit_length w X w X c
    mF = this->mu * Vcross(mW, Vcross(mW, ChVector3d(0, cm_y, cm_z)));

    // unroll the product [J] * w  in the expression w X [J] * w  as 4 values of [J] are zero anyway
    mT = Vcross(mW, ChVector3d(this->GetInertiaJxxPerUnitLength() * mW.x(), this->Jyy * mW.y() - this->Jyz * mW.z(),
                               this->Jzz * mW.z() - this->Jyz * mW.y()));
}

void ChInertiaCosseratAdvanced::SetMainInertiasInMassReference(double Jmyy, double Jmzz, double phi) {
    double cc = std::pow(std::cos(-phi), 2);
    double ss = std::pow(std::sin(-phi), 2);
    double cs = std::cos(-phi) * std::sin(-phi);
    // generic 2x2 tensor rotation
    double Tyy_rot = cc * Jmyy + ss * Jmzz;  // + 2 * Jmyz * cs;
    double Tzz_rot = ss * Jmyy + cc * Jmzz;  // - 2 * Jmyz * cs;
    double Tyz_rot = (Jmzz - Jmyy) * cs;     // +Jmyz * (cc - ss);
    // add inertia transport
    this->Jyy = Tyy_rot + this->mu * this->cm_z * this->cm_z;
    this->Jzz = Tzz_rot + this->mu * this->cm_y * this->cm_y;
    this->Jyz = -(Tyz_rot - this->mu * this->cm_z * this->cm_y);  // note minus, per definition of Jyz
}

void ChInertiaCosseratAdvanced::GetMainInertiasInMassReference(double& Jmyy, double& Jmzz, double& phi) {
    // remove inertia transport
    double Tyy_rot = this->Jyy - this->mu * this->cm_z * this->cm_z;
    double Tzz_rot = this->Jzz - this->mu * this->cm_y * this->cm_y;
    double Tyz_rot = -this->Jyz + this->mu * this->cm_z * this->cm_y;
    // tensor de-rotation up to principal axes
    double argum = std::pow((Tyy_rot - Tzz_rot) * 0.5, 2) + std::pow(Tyz_rot, 2);
    if (argum <= 0) {
        phi = 0;
        Jmyy = 0.5 * (Tzz_rot + Tyy_rot);
        Jmzz = 0.5 * (Tzz_rot + Tyy_rot);
        return;
    }
    double discr = std::sqrt(std::pow((Tyy_rot - Tzz_rot) * 0.5, 2) + std::pow(Tyz_rot, 2));
    phi = -0.5 * std::atan2(Tyz_rot / discr, (Tzz_rot - Tyy_rot) / (2. * discr));
    Jmyy = 0.5 * (Tzz_rot + Tyy_rot) - discr;
    Jmzz = 0.5 * (Tzz_rot + Tyy_rot) + discr;
}

void ChInertiaCosseratAdvanced::SetInertiasPerUnitLength(double Jyy_moment, double Jzz_moment, double Jyz_moment) {
    this->Jyy = Jyy_moment;
    this->Jzz = Jzz_moment;
    this->Jyz = Jyz_moment;
}

// -----------------------------------------------------------------------------

ChBeamSectionCosserat::ChBeamSectionCosserat(std::shared_ptr<ChInertiaCosserat> minertia,
                                             std::shared_ptr<ChElasticityCosserat> melasticity,
                                             std::shared_ptr<ChPlasticityCosserat> mplasticity,
                                             std::shared_ptr<ChDampingCosserat> mdamping) {
    this->SetInertia(minertia);

    this->SetElasticity(melasticity);

    if (mplasticity)
        this->SetPlasticity(mplasticity);

    if (mdamping)
        this->SetDamping(mdamping);
}

void ChBeamSectionCosserat::ComputeStress(ChVector3d& stress_n,
                                          ChVector3d& stress_m,
                                          const ChVector3d& strain_e,
                                          const ChVector3d& strain_k,
                                          ChBeamMaterialInternalData* mdata_new,
                                          const ChBeamMaterialInternalData* mdata) {
    if (!plasticity || !mdata || !mdata_new)
        this->elasticity->ComputeStress(stress_n, stress_m, strain_e, strain_k);
    else {
        ChVector3d e_strain_e;  // probably not needed as computable later as e_strain_e = strain_e - data.p_strain_e
        ChVector3d e_strain_k;  // probably not needed   "  "
        this->plasticity->ComputeStressWithReturnMapping(stress_n, stress_m, e_strain_e, e_strain_k, *mdata_new,
                                                         strain_e, strain_k, *mdata);
    }
}

void ChBeamSectionCosserat::ComputeStiffnessMatrix(ChMatrix66d& K,
                                                   const ChVector3d& strain_e,
                                                   const ChVector3d& strain_k,
                                                   const ChBeamMaterialInternalData* mdata) {
    if (!plasticity || !mdata)
        this->elasticity->ComputeStiffnessMatrix(K, strain_e, strain_k);
    else {
        this->plasticity->ComputeStiffnessMatrixElastoplastic(K, strain_e, strain_k, *mdata);
    }
}

void ChBeamSectionCosserat::SetElasticity(std::shared_ptr<ChElasticityCosserat> melasticity) {
    elasticity = melasticity;
    elasticity->section = this;
}

void ChBeamSectionCosserat::SetPlasticity(std::shared_ptr<ChPlasticityCosserat> mplasticity) {
    plasticity = mplasticity;
    plasticity->section = this;
}

void ChBeamSectionCosserat::SetDamping(std::shared_ptr<ChDampingCosserat> mdamping) {
    damping = mdamping;
    damping->section = this;
}

void ChBeamSectionCosserat::SetInertia(std::shared_ptr<ChInertiaCosserat> minertia) {
    inertia = minertia;
    inertia->section = this;
}

// -----------------------------------------------------------------------------

ChBeamSectionCosseratEasyRectangular::ChBeamSectionCosseratEasyRectangular(
    double width_y,  // width of section in y direction
    double width_z,  // width of section in z direction
    double E,        // Young modulus
    double G,        // shear modulus
    double density   // volumetric density (ex. in SI units: [kg/m])
) {
    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(E);
    melasticity->SetShearModulus(G);
    melasticity->SetAsRectangularSection(width_y, width_z);
    this->SetElasticity(melasticity);

    auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia->SetAsRectangularSection(width_y, width_z, density);
    this->SetInertia(minertia);

    auto mdrawshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(width_y, width_z);
    this->SetDrawShape(mdrawshape);
}

ChBeamSectionCosseratEasyCircular::ChBeamSectionCosseratEasyCircular(
    double diameter,  // diameter
    double E,         // Young modulus
    double G,         // shear modulus
    double density    // volumetric density (ex. in SI units: [kg/m])
) {
    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(E);
    melasticity->SetShearModulus(G);
    melasticity->SetAsCircularSection(diameter);
    this->SetElasticity(melasticity);

    auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia->SetAsCircularSection(diameter, density);
    this->SetInertia(minertia);

    auto mdrawshape = chrono_types::make_shared<ChBeamSectionShapeCircular>(diameter / 2, 10);
    this->SetDrawShape(mdrawshape);
}

// ------------------------------------------------------------------------------

}  // end namespace fea
}  // end namespace chrono
