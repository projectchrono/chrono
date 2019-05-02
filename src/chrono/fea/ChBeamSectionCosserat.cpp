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

#include "chrono/fea/ChBeamSectionCosserat.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------

void ChElasticityCosserat::ComputeStiffnessMatrix(ChMatrixDynamic<>& K,
                                                  const ChVector<>& strain_e,
                                                  const ChVector<>& strain_k) {
    double epsi = 1e-6;
    double invepsi = 1.0 / epsi;
    ChVector<> astress_n;
    ChVector<> astress_m;
    ChVector<> bstress_n;
    ChVector<> bstress_m;
    ChVector<> strain_e_inc = strain_e;
    ChVector<> strain_k_inc = strain_k;
    this->ComputeStress(astress_n, astress_m, strain_e, strain_k);
    for (int i = 0; i < 3; ++i) {
        strain_e_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        K.PasteVector((bstress_n - astress_n) * invepsi, 0, i);
        K.PasteVector((bstress_m - astress_m) * invepsi, 3, i);
        strain_e_inc[i] -= epsi;
    }
    for (int i = 0; i < 3; ++i) {
        strain_k_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        K.PasteVector((bstress_n - astress_n) * invepsi, 0, i + 3);
        K.PasteVector((bstress_m - astress_m) * invepsi, 3, i + 3);
        strain_k_inc[i] -= epsi;
    }
}

// -----------------------------------------------------------------------------

ChElasticityCosseratSimple::ChElasticityCosseratSimple()
    : E(0.01e9),      // default E stiffness (almost rubber)
      rdamping(0.01)  // default Rayleigh damping
{
    SetGwithPoissonRatio(0.3);            // default G (low poisson ratio)
    SetAsRectangularSection(0.01, 0.01);  // defaults Area, Ixx, Iyy, Ks_y, Ks_z, J
}

void ChElasticityCosseratSimple::SetAsRectangularSection(double width_y, double width_z) {
    this->Izz = (1.0 / 12.0) * width_z * pow(width_y, 3);
    this->Iyy = (1.0 / 12.0) * width_y * pow(width_z, 3);

    // use Roark's formulas for torsion of rectangular sect:
    double t = ChMin(width_y, width_z);
    double b = ChMax(width_y, width_z);
    this->J = b * pow(t, 3) * ((1.0 / 3.0) - 0.210 * (t / b) * (1.0 - (1.0 / 12.0) * pow((t / b), 4)));

    // set Ks using Timoshenko-Gere formula for solid rect.shapes
    double poisson = this->E / (2.0 * this->G) - 1.0;
    this->Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
    this->Ks_z = this->Ks_y;
}

void ChElasticityCosseratSimple::SetAsCircularSection(double diameter) {
    this->Izz = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);
    this->Iyy = Izz;

    // exact expression for circular beam J = Ixx ,
    // where for polar theorem Ixx = Izz+Iyy
    this->J = Izz + Iyy;

    // set Ks using Timoshenko-Gere formula for solid circular shape
    double poisson = this->E / (2.0 * this->G) - 1.0;
    this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
    this->Ks_z = this->Ks_y;
}

void ChElasticityCosseratSimple::ComputeStress(ChVector<>& stress_n,
                                               ChVector<>& stress_m,
                                               const ChVector<>& strain_n,
                                               const ChVector<>& strain_m) {
    stress_n.x() = E * section->Area * strain_n.x();
    stress_n.y() = Ks_y * G * section->Area * strain_n.y();
    stress_n.z() = Ks_z * G * section->Area * strain_n.z();
    stress_m.x() = G * J * strain_m.x();
    stress_m.y() = E * Iyy * strain_m.y();
    stress_m.z() = E * Izz * strain_m.z();
}

void ChElasticityCosseratSimple::ComputeStiffnessMatrix(ChMatrixDynamic<>& K,
                                                        const ChVector<>& strain_n,
                                                        const ChVector<>& strain_m) {
    K.Reset(6, 6);
    K(0, 0) = E * section->Area;
    K(1, 1) = Ks_y * G * section->Area;
    K(2, 2) = Ks_z * G * section->Area;
    K(3, 3) = G * J;
    K(4, 4) = E * Iyy;
    K(5, 5) = E * Izz;
}

// -----------------------------------------------------------------------------

ChElasticityCosseratGeneric::ChElasticityCosseratGeneric() {
    mE.SetIdentity();                     // default E stiffness: diagonal 1.
    SetAsRectangularSection(0.01, 0.01);  // defaults Area, Ixx, Iyy, Ks_y, Ks_z, J
}

void ChElasticityCosseratGeneric::SetAsRectangularSection(double width_y, double width_z) {
    double E = 1;
    double G = 1;

    double Izz = (1.0 / 12.0) * width_z * pow(width_y, 3);
    double Iyy = (1.0 / 12.0) * width_y * pow(width_z, 3);

    // use Roark's formulas for torsion of rectangular sect:
    double t = ChMin(width_y, width_z);
    double b = ChMax(width_y, width_z);
    double J = b * pow(t, 3) * ((1.0 / 3.0) - 0.210 * (t / b) * (1.0 - (1.0 / 12.0) * pow((t / b), 4)));

    // set Ks using Timoshenko-Gere formula for solid rect.shapes
    double poisson = E / (2.0 * G) - 1.0;
    double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
    double Ks_z = Ks_y;

    mE(0, 0) = E * section->Area;
    mE(1, 1) = Ks_y * G * section->Area;
    mE(2, 2) = Ks_z * G * section->Area;
    mE(3, 3) = G * J;
    mE(4, 4) = E * Iyy;
    mE(5, 5) = E * Izz;
}

void ChElasticityCosseratGeneric::SetAsCircularSection(double diameter) {
    double E = 1;
    double G = 1;

    double Izz = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);
    double Iyy = Izz;

    // exact expression for circular beam J = Ixx ,
    // where for polar theorem Ixx = Izz+Iyy
    double J = Izz + Iyy;

    // set Ks using Timoshenko-Gere formula for solid circular shape
    double poisson = E / (2.0 * G) - 1.0;
    double Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
    double Ks_z = Ks_y;

    mE(0, 0) = E * section->Area;
    mE(1, 1) = Ks_y * G * section->Area;
    mE(2, 2) = Ks_z * G * section->Area;
    mE(3, 3) = G * J;
    mE(4, 4) = E * Iyy;
    mE(5, 5) = E * Izz;
}

void ChElasticityCosseratGeneric::ComputeStress(ChVector<>& stress_n,
                                                ChVector<>& stress_m,
                                                const ChVector<>& strain_n,
                                                const ChVector<>& strain_m) {
    ChMatrixNM<double, 6, 1> mstrain;
    ChMatrixNM<double, 6, 1> mstress;
    mstrain.PasteVector(strain_n, 0, 0);
    mstrain.PasteVector(strain_m, 3, 0);
    mstress.MatrMultiply(this->mE, mstrain);
    stress_n = mstress.ClipVector(0, 0);
    stress_m = mstress.ClipVector(3, 0);
}

void ChElasticityCosseratGeneric::ComputeStiffnessMatrix(ChMatrixDynamic<>& K,
                                                         const ChVector<>& strain_n,
                                                         const ChVector<>& strain_m) {
    K.CopyFromMatrix(this->mE);
}

// -----------------------------------------------------------------------------

ChElasticityCosseratAdvanced::ChElasticityCosseratAdvanced() : alpha(0), Cy(0), Cz(0), beta(0), Sy(0), Sz(0) {}

void ChElasticityCosseratAdvanced::ComputeStress(ChVector<>& stress_n,
                                                 ChVector<>& stress_m,
                                                 const ChVector<>& strain_n,
                                                 const ChVector<>& strain_m) {
    double Area = section->Area;
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double a11 = E * section->Area;
    double a22 = E * (Iyy * pow(cos_alpha, 2.) + Izz * pow(sin_alpha, 2.) + Cz * Cz * Area);
    double a33 = E * (Izz * pow(cos_alpha, 2.) + Iyy * pow(sin_alpha, 2.) + Cy * Cy * Area);
    double a12 = Cz * E * Area;
    double a13 = -Cy * E * Area;
    double a23 = (E * Iyy - E * Izz) * cos_alpha * sin_alpha - E * Cy * Cz * Area;
    stress_n.x() = a11 * strain_n.x() + a12 * strain_m.y() + a13 * strain_m.z();
    stress_m.y() = a12 * strain_n.x() + a22 * strain_m.y() + a23 * strain_m.z();
    stress_m.z() = a13 * strain_n.x() + a23 * strain_m.y() + a33 * strain_m.z();
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double KsyGA = Ks_y * G * Area;
    double KszGA = Ks_z * G * Area;
    double s11 = KsyGA * pow(cos_beta, 2.) + KszGA * pow(sin_beta, 2.);
    double s22 = KsyGA * pow(sin_beta, 2.) + KszGA * pow(cos_beta, 2.);  // ..+s_loc_12*sin(beta)*cos(beta);
    double s33 = G * J + Sz * Sz * KsyGA + Sy * Sy * KszGA;
    double s12 = (KszGA - KsyGA) * sin_beta * cos_beta;
    double s13 = Sy * KszGA * sin_beta - Sz * KsyGA * cos_beta;
    double s23 = Sy * KszGA * cos_beta + Sz * KsyGA * sin_beta;
    stress_n.y() = s11 * strain_n.y() + s12 * strain_n.z() + s13 * strain_m.x();
    stress_n.z() = s12 * strain_n.y() + s22 * strain_n.z() + s23 * strain_m.x();
    stress_m.x() = s13 * strain_n.y() + s23 * strain_n.z() + s33 * strain_m.x();
}

void ChElasticityCosseratAdvanced::ComputeStiffnessMatrix(ChMatrixDynamic<>& K,
                                                          const ChVector<>& strain_n,
                                                          const ChVector<>& strain_m) {
    K.Reset(6, 6);
    double Area = section->Area;
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double a11 = E * section->Area;
    double a22 = E * (Iyy * pow(cos_alpha, 2.) + Izz * pow(sin_alpha, 2.) + Cz * Cz * Area);
    double a33 = E * (Izz * pow(cos_alpha, 2.) + Iyy * pow(sin_alpha, 2.) + Cy * Cy * Area);
    double a12 = Cz * E * Area;
    double a13 = -Cy * E * Area;
    double a23 = (E * Iyy - E * Izz) * cos_alpha * sin_alpha - E * Cy * Cz * Area;
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double KsyGA = Ks_y * G * Area;
    double KszGA = Ks_z * G * Area;
    double s11 = KsyGA * pow(cos_beta, 2.) + KszGA * pow(sin_beta, 2.);
    double s22 = KsyGA * pow(sin_beta, 2.) + KszGA * pow(cos_beta, 2.);  // ..+s_loc_12*sin(beta)*cos(beta);
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

void ChElasticityCosseratMesh::SetAsRectangularSection(double width_y, double width_z) {
    this->vertexes.clear();
    this->vertexes.push_back(ChVector2<>(width_y * 0.5, width_z * 0.5));
    this->vertexes.push_back(ChVector2<>(width_y * 0.5, -width_z * 0.5));
    this->vertexes.push_back(ChVector2<>(-width_y * 0.5, -width_z * 0.5));
    this->vertexes.push_back(ChVector2<>(-width_y * 0.5, width_z * 0.5));

    this->triangles.clear();
    this->triangles.push_back(ChVector<int>(0, 1, 2));
    this->triangles.push_back(ChVector<int>(0, 2, 3));

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
    this->vertexes.push_back(ChVector2<>(0, 0));
    this->vertexes.push_back(ChVector2<>(rad, 0));
    int ntri = 12;
    for (int i = 0; i < ntri; ++i) {
        double alpha = (i + 1) * (CH_C_2PI / (double)ntri);
        this->vertexes.push_back(ChVector2<>(rad * cos(alpha), rad * sin(alpha)));
        this->triangles.push_back(ChVector<int>(0, i + 1, i + 2));
    }

    // set Ks using Timoshenko-Gere formula for solid circular shape
    /*
    double poisson = this->E / (2.0 * this->G) - 1.0;
    this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
    this->Ks_z = this->Ks_y;
    */
}

void ChElasticityCosseratMesh::ComputeStress(ChVector<>& stress_n,
                                             ChVector<>& stress_m,
                                             const ChVector<>& strain_n,
                                             const ChVector<>& strain_m) {
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

        stress_n.x() += (1. / 3.) * A * (s1 + s2 + s3);
        stress_n.y() += (1. / 3.) * A * (sxy1 + sxy2 + sxy3);
        stress_n.z() += (1. / 3.) * A * (sxz1 + sxz2 + sxz3);

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

void ChPlasticityCosserat::ComputeStiffnessMatrixElastoplastic(ChMatrixDynamic<>& K,
                                                               const ChVector<>& strain_n,
                                                               const ChVector<>& strain_m,
                                                               const ChBeamMaterialInternalData& data) {
    ChVector<> astress_n;
    ChVector<> astress_m;
    ChVector<> me_strain_n_new;  // needed only as placeholder
    ChVector<> me_strain_m_new;  // needed only as placeholder

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
        ChVector<> bstress_n;
        ChVector<> bstress_m;
        ChVector<> strain_n_inc = strain_n;
        ChVector<> strain_m_inc = strain_m;
        for (int i = 0; i < 3; ++i) {
            strain_n_inc[i] += epsi;
            this->ComputeStressWithReturnMapping(bstress_n, bstress_m, me_strain_n_new, me_strain_m_new,
                                                 *b_plastic_data[0], strain_n_inc, strain_m_inc, data);
            K.PasteVector((bstress_n - astress_n) * invepsi, 0, i);
            K.PasteVector((bstress_m - astress_m) * invepsi, 3, i);
            strain_n_inc[i] -= epsi;
        }
        for (int i = 0; i < 3; ++i) {
            strain_m_inc[i] += epsi;
            this->ComputeStressWithReturnMapping(bstress_n, bstress_m, me_strain_n_new, me_strain_m_new,
                                                 *b_plastic_data[0], strain_n_inc, strain_m_inc, data);
            K.PasteVector((bstress_n - astress_n) * invepsi, 0, i + 3);
            K.PasteVector((bstress_m - astress_m) * invepsi, 3, i + 3);
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
    n_yeld_x = std::make_shared<ChFunction_Const>(1000);
    n_beta_x = std::make_shared<ChFunction_Const>(0);
    n_yeld_y = std::make_shared<ChFunction_Const>(1000);
    n_beta_y = std::make_shared<ChFunction_Const>(0);
    n_yeld_z = std::make_shared<ChFunction_Const>(1000);
    n_beta_z = std::make_shared<ChFunction_Const>(0);
    n_yeld_Mx = std::make_shared<ChFunction_Const>(1000);
    n_beta_Mx = std::make_shared<ChFunction_Const>(0);
    n_yeld_My = std::make_shared<ChFunction_Const>(1000);
    n_beta_My = std::make_shared<ChFunction_Const>(0);
    n_yeld_Mz = std::make_shared<ChFunction_Const>(1000);
    n_beta_Mz = std::make_shared<ChFunction_Const>(0);
}

bool ChPlasticityCosseratLumped::ComputeStressWithReturnMapping(ChVector<>& stress_n,
                                                                ChVector<>& stress_m,
                                                                ChVector<>& e_strain_e_new,
                                                                ChVector<>& e_strain_k_new,
                                                                ChBeamMaterialInternalData& data_new,
                                                                const ChVector<>& tot_strain_e,
                                                                const ChVector<>& tot_strain_k,
                                                                const ChBeamMaterialInternalData& data) {
    auto mydata = dynamic_cast<const ChInternalDataLumpedCosserat*>(&data);
    auto mydata_new = dynamic_cast<ChInternalDataLumpedCosserat*>(&data_new);

    if (!mydata)
        throw ChException("ComputeStressWithReturnMapping cannot cast data to ChInternalDataLumpedCosserat*.");

    // Implement return mapping for a simple 1D plasticity model.

    // Compute the elastic trial stress:
    e_strain_e_new = tot_strain_e - mydata->p_strain_e;
    e_strain_k_new = tot_strain_k - mydata->p_strain_k;
    // double p_strain_acc = mydata->p_strain_acc;
    this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);

    // axial direction
    {
        double strain_yeld_x = this->n_yeld_x->Get_y(mydata->p_strain_acc_e.x());     ///<<<< sigma_y(p_strain_acc)
        double eta_x = stress_n.x() - this->n_beta_x->Get_y(mydata->p_strain_e.x());  ///<<<< beta(p_strain_e)
        double Fyeld_x = fabs(eta_x) - strain_yeld_x;                                 //<<<<  Phi(sigma,p_strain_acc)

        if (Fyeld_x > 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_e.x() = mydata->p_strain_acc_e.x();
            mydata_new->p_strain_e.x() = mydata->p_strain_e.x();
            int iters = 0;
            while ((Fyeld_x > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_x = stress_n.x() / e_strain_e_new.x();  // instead of costly evaluation of Km, =dstress/dstrain
                double H = this->n_beta_x->Get_y_dx(mydata->p_strain_e.x()) +
                           this->n_yeld_x->Get_y_dx(mydata->p_strain_acc_e.x());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_x / (-E_x - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_e.x() += dDgamma;
                e_strain_e_new.x() -= dDgamma * chrono::ChSignum(stress_n.x());
                mydata_new->p_strain_e.x() += dDgamma * chrono::ChSignum(stress_n.x());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_x = this->n_yeld_x->Get_y(mydata_new->p_strain_acc_e.x());     ///<<<< sigma_y(p_strain_acc)
                eta_x = stress_n.x() - this->n_beta_x->Get_y(mydata_new->p_strain_e.x());  ///<<<< beta(p_strain_acc)
                Fyeld_x = fabs(eta_x) - strain_yeld_x;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // shear direction
    {
        double strain_yeld_y = this->n_yeld_y->Get_y(mydata->p_strain_acc_e.y());
        double eta_y = stress_n.y() - this->n_beta_y->Get_y(mydata->p_strain_e.y());
        double Fyeld_y = fabs(eta_y) - strain_yeld_y;  //<<<<  Phi(sigma,p_strain_acc)

        if (Fyeld_y < 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_e.y() = mydata->p_strain_acc_e.y();
            mydata_new->p_strain_e.y() = mydata->p_strain_e.y();
            int iters = 0;
            while ((Fyeld_y > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_y = stress_n.y() / e_strain_e_new.y();  // instead of costly evaluation of Km, =dstress/dstrain
                double H = this->n_beta_y->Get_y_dx(mydata->p_strain_e.y()) +
                           this->n_yeld_y->Get_y_dx(mydata->p_strain_acc_e.y());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_y / (-E_y - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_e.y() += dDgamma;
                e_strain_e_new.y() -= dDgamma * chrono::ChSignum(stress_n.y());
                mydata_new->p_strain_e.y() += dDgamma * chrono::ChSignum(stress_n.y());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_y = this->n_yeld_y->Get_y(mydata_new->p_strain_acc_e.y());     ///<<<< sigma_y(p_strain_acc)
                eta_y = stress_n.y() - this->n_beta_y->Get_y(mydata_new->p_strain_e.y());  ///<<<< beta(p_strain_acc)
                Fyeld_y = fabs(eta_y) - strain_yeld_y;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // shear direction
    {
        double strain_yeld_z = this->n_yeld_z->Get_y(mydata->p_strain_acc_e.z());
        double eta_z = stress_n.z() - this->n_beta_z->Get_y(mydata->p_strain_e.z());
        double Fyeld_z = fabs(eta_z) - strain_yeld_z;  //<<<<  Phi(sigma,p_strain_acc)

        if (Fyeld_z > 0) {
            double Dgamma = 0;
            double Dgamma_old = 0;
            mydata_new->p_strain_acc_e.z() = mydata->p_strain_acc_e.z();
            mydata_new->p_strain_e.z() = mydata->p_strain_e.z();
            int iters = 0;
            while ((Fyeld_z > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
                double E_z = stress_n.z() / e_strain_e_new.z();  // instead of costly evaluation of Km, =dstress/dstrain
                double H = this->n_beta_z->Get_y_dx(mydata->p_strain_e.z()) +
                           this->n_yeld_z->Get_y_dx(mydata->p_strain_acc_e.z());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_z / (-E_z - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_e.z() += dDgamma;
                e_strain_e_new.z() -= dDgamma * chrono::ChSignum(stress_n.z());
                mydata_new->p_strain_e.z() += dDgamma * chrono::ChSignum(stress_n.z());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_z = this->n_yeld_z->Get_y(mydata_new->p_strain_acc_e.z());     ///<<<< sigma_y(p_strain_acc)
                eta_z = stress_n.z() - this->n_beta_z->Get_y(mydata_new->p_strain_e.z());  ///<<<< beta(p_strain_acc)
                Fyeld_z = fabs(eta_z) - strain_yeld_z;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // torsion direction
    {
        double strain_yeld_Mx = this->n_yeld_Mx->Get_y(mydata->p_strain_acc_k.x());
        double eta_Mx = stress_m.x() - this->n_beta_Mx->Get_y(mydata->p_strain_k.x());
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
                double H = this->n_beta_Mx->Get_y_dx(mydata->p_strain_k.x()) +
                           this->n_yeld_Mx->Get_y_dx(mydata->p_strain_acc_k.x());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_Mx / (-E_Mx - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_k.x() += dDgamma;
                e_strain_k_new.x() -= dDgamma * chrono::ChSignum(stress_m.x());
                mydata_new->p_strain_k.x() += dDgamma * chrono::ChSignum(stress_m.x());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_Mx = this->n_yeld_Mx->Get_y(mydata_new->p_strain_acc_k.x());  ///<<<< sigma_y(p_strain_acc)
                eta_Mx = stress_m.x() - this->n_beta_Mx->Get_y(mydata_new->p_strain_k.x());  ///<<<< beta(p_strain_acc)
                Fyeld_Mx = fabs(eta_Mx) - strain_yeld_Mx;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // bending y direction
    {
        double strain_yeld_My = this->n_yeld_My->Get_y(mydata->p_strain_acc_k.y());
        double eta_My = stress_m.y() - this->n_beta_My->Get_y(mydata->p_strain_k.y());
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
                double H = this->n_beta_My->Get_y_dx(mydata->p_strain_k.y()) +
                           this->n_yeld_My->Get_y_dx(mydata->p_strain_acc_k.y());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_My / (-E_My - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_k.y() += dDgamma;
                e_strain_k_new.y() -= dDgamma * chrono::ChSignum(stress_m.y());
                mydata_new->p_strain_k.y() += dDgamma * chrono::ChSignum(stress_m.y());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_My = this->n_yeld_My->Get_y(mydata_new->p_strain_acc_k.y());  ///<<<< sigma_y(p_strain_acc)
                eta_My = stress_m.y() - this->n_beta_My->Get_y(mydata_new->p_strain_k.y());  ///<<<< beta(p_strain_acc)
                Fyeld_My = fabs(eta_My) - strain_yeld_My;  //<<<<  Phi(sigma,p_strain_acc)

                ++iters;
            }
        }
    }

    // bending z direction
    {
        double strain_yeld_Mz = this->n_yeld_Mz->Get_y(mydata->p_strain_acc_k.z());
        double eta_Mz = stress_m.z() - this->n_beta_Mz->Get_y(mydata->p_strain_k.z());
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
                double H = this->n_beta_Mz->Get_y_dx(mydata->p_strain_k.z()) +
                           this->n_yeld_Mz->Get_y_dx(mydata->p_strain_acc_k.z());  //<<<<  H = dyeld/dplasticflow
                Dgamma -= Fyeld_Mz / (-E_Mz - H);
                double dDgamma = Dgamma - Dgamma_old;
                Dgamma_old = Dgamma;
                mydata_new->p_strain_acc_k.z() += dDgamma;
                e_strain_k_new.z() -= dDgamma * chrono::ChSignum(stress_m.z());
                mydata_new->p_strain_k.z() += dDgamma * chrono::ChSignum(stress_m.z());
                this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);
                // compute yeld
                strain_yeld_Mz = this->n_yeld_Mz->Get_y(mydata_new->p_strain_acc_k.z());  ///<<<< sigma_y(p_strain_acc)
                eta_Mz = stress_m.z() - this->n_beta_Mz->Get_y(mydata_new->p_strain_k.z());  ///<<<< beta(p_strain_acc)
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

void ChDampingCosserat::ComputeDampingMatrix(ChMatrixDynamic<>& R,
                                             const ChVector<>& dstrain_e,
                                             const ChVector<>& dstrain_k) {
    double epsi = 1e-6;
    double invepsi = 1.0 / epsi;
    ChVector<> astress_n;
    ChVector<> astress_m;
    ChVector<> bstress_n;
    ChVector<> bstress_m;
    ChVector<> strain_e_inc = dstrain_e;
    ChVector<> strain_k_inc = dstrain_k;
    this->ComputeStress(astress_n, astress_m, dstrain_e, dstrain_k);
    for (int i = 0; i < 3; ++i) {
        strain_e_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        R.PasteVector((bstress_n - astress_n) * invepsi, 0, i);
        R.PasteVector((bstress_m - astress_m) * invepsi, 3, i);
        strain_e_inc[i] -= epsi;
    }
    for (int i = 0; i < 3; ++i) {
        strain_k_inc[i] += epsi;
        this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
        R.PasteVector((bstress_n - astress_n) * invepsi, 0, i + 3);
        R.PasteVector((bstress_m - astress_m) * invepsi, 3, i + 3);
        strain_k_inc[i] -= epsi;
    }
}

// -----------------------------------------------------------------------------

void ChDampingCosseratLinear::ComputeStress(ChVector<>& stress_n,
                                            ChVector<>& stress_m,
                                            const ChVector<>& dstrain_e,
                                            const ChVector<>& dstrain_k) {
    stress_n.x() = dstrain_e.x() * R_e.x();
    stress_n.y() = dstrain_e.y() * R_e.y();
    stress_n.z() = dstrain_e.z() * R_e.z();
    stress_m.x() = dstrain_k.x() * R_k.x();
    stress_m.y() = dstrain_k.y() * R_k.y();
    stress_m.z() = dstrain_k.z() * R_k.z();
}

void ChDampingCosseratLinear::ComputeDampingMatrix(ChMatrixDynamic<>& R,
                                                   const ChVector<>& dstrain_e,
                                                   const ChVector<>& dstrain_k) {
    R.Reset();
    R(0, 0) = R_e.x();
    R(1, 1) = R_e.y();
    R(2, 2) = R_e.z();
    R(3, 3) = R_k.x();
    R(4, 4) = R_k.y();
    R(5, 5) = R_k.z();
}

// -----------------------------------------------------------------------------

ChBeamSectionCosserat::ChBeamSectionCosserat(std::shared_ptr<ChElasticityCosserat> melasticity) {
    this->SetElasticity(melasticity);
}

ChBeamSectionCosserat::ChBeamSectionCosserat(std::shared_ptr<ChElasticityCosserat> melasticity,
                                             std::shared_ptr<ChPlasticityCosserat> mplasticity) {
    this->SetElasticity(melasticity);
    this->SetPlasticity(mplasticity);
}

ChBeamSectionCosserat::ChBeamSectionCosserat(std::shared_ptr<ChElasticityCosserat> melasticity,
                                             std::shared_ptr<ChPlasticityCosserat> mplasticity,
                                             std::shared_ptr<ChDampingCosserat> mdamping) {
    this->SetElasticity(melasticity);

    if (mplasticity)
        this->SetPlasticity(mplasticity);

    if (mdamping)
        this->SetDamping(mdamping);
}

void ChBeamSectionCosserat::ComputeStress(ChVector<>& stress_n,
                                          ChVector<>& stress_m,
                                          const ChVector<>& strain_e,
                                          const ChVector<>& strain_k,
                                          ChBeamMaterialInternalData* mdata_new,
                                          const ChBeamMaterialInternalData* mdata) {
    if (!plasticity || !mdata || !mdata_new)
        this->elasticity->ComputeStress(stress_n, stress_m, strain_e, strain_k);
    else {
        ChVector<> e_strain_e;  // probably not needed as computable later as e_strain_e = strain_e - data.p_strain_e
        ChVector<> e_strain_k;  // probably not needed   "  "
        this->plasticity->ComputeStressWithReturnMapping(stress_n, stress_m, e_strain_e, e_strain_k, *mdata_new,
                                                         strain_e, strain_k, *mdata);
    }
}

void ChBeamSectionCosserat::ComputeStiffnessMatrix(ChMatrixDynamic<>& K,
                                                   const ChVector<>& strain_e,
                                                   const ChVector<>& strain_k,
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
    mplasticity->section = this;
}

void ChBeamSectionCosserat::SetDamping(std::shared_ptr<ChDampingCosserat> mdamping) {
    damping = mdamping;
    damping->section = this;
}

void ChBeamSectionCosserat::SetAsRectangularSection(double width_y, double width_z) {
    this->Area = width_y * width_z;
    this->is_circular = false;
    this->y_drawsize = width_y;
    this->z_drawsize = width_z;

    if (this->elasticity)
        this->elasticity->SetAsRectangularSection(width_y, width_z);
    if (this->plasticity)
        this->plasticity->SetAsRectangularSection(width_y, width_z);
    if (this->damping)
        this->damping->SetAsRectangularSection(width_y, width_z);
}

void ChBeamSectionCosserat::SetAsCircularSection(double diameter) {
    this->Area = CH_C_PI * pow((0.5 * diameter), 2);
    this->is_circular = true;
    this->SetDrawCircularRadius(diameter / 2);

    if (this->elasticity)
        this->elasticity->SetAsCircularSection(diameter);
    if (this->plasticity)
        this->plasticity->SetAsCircularSection(diameter);
    if (this->damping)
        this->damping->SetAsCircularSection(diameter);
}

// -----------------------------------------------------------------------------

}  // end namespace fea
}  // end namespace chrono
