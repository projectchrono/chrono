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

#ifndef CHTENSORS_H
#define CHTENSORS_H

#include <cstdlib>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChMath.h"

namespace chrono {
namespace fea {

/// Base class for stress and strain tensors, in compact Voight notation
/// that is with 6 components in a column. This saves some
/// memory when compared to traditional 3D tensors with three
/// rows and three columns, that are symmetric.

template <class Real = double>
class ChVoightTensor : public ChMatrixNM<Real, 6, 1> {
  public:
    /// Constructors (default empty)
    ChVoightTensor() { this->Reset(); }

    ~ChVoightTensor() {}

    /// Copy constructor, from a typical 3D rank-two stress or strain tensor (as 3x3 matrix)
    template <class RealB>
    inline ChVoightTensor(const ChMatrix33<RealB>& msource) {
        this->ConvertFromMatrix(msource);
    }

    inline Real& XX() { return ChMatrix<Real>::ElementN(0); }
    inline const Real& XX() const { return ChMatrix<Real>::ElementN(0); }

    inline Real& YY() { return ChMatrix<Real>::ElementN(1); }
    inline const Real& YY() const { return ChMatrix<Real>::ElementN(1); }

    inline Real& ZZ() { return ChMatrix<Real>::ElementN(2); }
    inline const Real& ZZ() const { return ChMatrix<Real>::ElementN(2); }

    inline Real& XY() { return ChMatrix<Real>::ElementN(3); }
    inline const Real& XY() const { return ChMatrix<Real>::ElementN(3); }

    inline Real& XZ() { return ChMatrix<Real>::ElementN(4); }
    inline const Real& XZ() const { return ChMatrix<Real>::ElementN(4); }

    inline Real& YZ() { return ChMatrix<Real>::ElementN(5); }
    inline const Real& YZ() const { return ChMatrix<Real>::ElementN(5); }

    /// Convert from a typical 3D rank-two stress or strain tensor (a 3x3 matrix)
    template <class RealB>
    void ConvertFromMatrix(const ChMatrix33<RealB>& msource) {
        XX() = (Real)msource(0, 0);
        YY() = (Real)msource(1, 1);
        ZZ() = (Real)msource(2, 2);
        XY() = (Real)msource(0, 1);
        XZ() = (Real)msource(0, 2);
        YZ() = (Real)msource(1, 2);
    }

    /// Convert to a typical 3D rank-two stress or strain tensor (a 3x3 matrix)
    template <class RealB>
    void ConvertToMatrix(ChMatrix33<RealB>& mdest) {
        mdest(0, 0) = (RealB)XX();
        mdest(1, 1) = (RealB)YY();
        mdest(2, 2) = (RealB)ZZ();
        mdest(0, 1) = (RealB)XY();
        mdest(0, 2) = (RealB)XZ();
        mdest(1, 2) = (RealB)YZ();
        mdest(1, 0) = (RealB)XY();
        mdest(2, 0) = (RealB)XZ();
        mdest(2, 1) = (RealB)YZ();
    }

    /// Compute the volumetric part of the tensor, that is
    /// the trace V =Txx+Tyy+Tzz.
    Real GetVolumetricPart() const { return XX() + YY() + ZZ(); }
    /// Compute the deviatoric part of the tensor, storing
    /// it in mdeviatoric
    void GetDeviatoricPart(ChVoightTensor<Real>& mdeviatoric) const {
        Real mM = GetVolumetricPart() / 3.0;
        mdeviatoric = *this;
        mdeviatoric.XX() -= mM;
        mdeviatoric.YY() -= mM;
        mdeviatoric.ZZ() -= mM;
    }

    /// Compute the I1 invariant
    Real GetInvariant_I1() const { return XX() + YY() + ZZ(); }

    /// Compute the I2 invariant
    Real GetInvariant_I2() const {
        return XX() * YY() + YY() * ZZ() + XX() * ZZ() - XY() * XY() - YZ() * YZ() - XZ() * XZ();
    }

    /// Compute the I3 invariant
    Real GetInvariant_I3() const {
        return XX() * YY() * ZZ() + 2 * XY() * YZ() * XZ() - XY() * XY() * ZZ() - YZ() * YZ() * XX() -
               XZ() * XZ() * YY();
    }

    /// Compute the J1 invariant of the deviatoric part (that is always 0)
    Real GetInvariant_J1() const { return 0; }

    /// Compute the J2 invariant of the deviatoric part
    Real GetInvariant_J2() const {
        return ChMax(0.0, ((pow(this->GetInvariant_I1(), 2)) / 3.0) - this->GetInvariant_I2());
    }
    /// Compute the J3 invariant of the deviatoric part
    Real GetInvariant_J3() const {
        return (pow(this->GetInvariant_I1(), 3) * (2. / 27.) -
                this->GetInvariant_I1() * this->GetInvariant_I2() * (1. / 3.) + this->GetInvariant_I3());
    }

    /// Rotate to another reference coordinate system,
    /// overwriting this tensor in place.
    void Rotate(ChMatrix33<Real> Rot) {
        ChMatrix33<Real> T;
        ChMatrix33<Real> temp;
        // do  T'= R*T*R'
        this->ConvertToMatrix(T);
        temp.MatrMultiplyT(T, Rot);
        T.MatrMultiply(Rot, temp);
        this->ConvertFromMatrix(T);  // to do, more efficient: unroll matrix multiplications and exploit T symmetry
    }

    /// Compute the eigenvalues (closed form method)
    void ComputeEigenvalues(double& e1, double& e2, double& e3) {
        double I1 = this->GetInvariant_I1();
        double I2 = this->GetInvariant_I2();
        double I3 = this->GetInvariant_I3();
        double phi =
            (1. / 3.) * acos((2. * I1 * I1 * I1 - 9. * I1 * I2 + 27. * I3) / (2. * pow((I1 * I1 - 3 * I2), (3. / 2.))));
        double k = (2. / 3.) * (sqrt(I1 * I1 - 3. * I2));
        e1 = (I1 / 3.) + k * cos(phi);
        e2 = (I1 / 3.) + k * cos(phi + (2. / 3.) * chrono::CH_C_PI);
        e3 = (I1 / 3.) + k * cos(phi + (4. / 3.) * chrono::CH_C_PI);
    }

    /// Compute the eigenvectors and the eigenvalues
    void ComputeEigenvectors(double& eigval1,
                             double& eigval2,
                             double& eigval3,
                             ChVector<Real>& eigvector1,
                             ChVector<Real>& eigvector2,
                             ChVector<Real>& eigvector3) {
        ChMatrix33<Real> A;

        // GetLog() << A << "\n  vals:" << eigval1 << "   " << eigval2 << "   " << eigval3 << "  eigvect1: \n" <<
        // eigvector1.GetNormalized();
        this->ConvertToMatrix(A);
        // GetLog() << A ;
        ChMatrix33<> vects;
        double vals[3];
        A.FastEigen(vects, vals);
        eigvector1 = vects.ClipVector(0, 0);
        eigvector2 = vects.ClipVector(0, 1);
        eigvector3 = vects.ClipVector(0, 2);
        eigval1 = vals[0];
        eigval2 = vals[1];
        eigval3 = vals[2];
        // GetLog() << "\n  vals:" << vals[0] << "   " << vals[1] << "   " << vals[2] << "  eigvect1: \n" <<
        // eigvector1.GetNormalized();
        /*

        */
    }

    /// FORMULAS THAT ARE USEFUL FOR YELD CRITERIONS:

    /// Compute the Von Mises equivalent
    double GetEquivalentVonMises() const {
        return sqrt(0.5 * (pow(this->XX() - this->YY(), 2.) + pow(this->YY() - this->ZZ(), 2.) +
                           pow(this->ZZ() - this->XX(), 2.)) +
                    3.0 * (this->XY() * this->XY() + this->XZ() * this->XZ() + this->YZ() * this->YZ()));
    }

    /// Compute the mean hydrostatic value (aka volumetric, normal)
    double GetEquivalentMeanHydrostatic() const { return (this->GetInvariant_I1() / 3.); }

    /// Compute the octahedral normal invariant (aka hydrostatic, volumetric)
    double GetEquivalentOctahedralNormal() const { return this->GetEquivalentMeanHydrostatic(); }
    /// Compute the octahedral deviatoric invariant (aka shear)
    double GetEquivalentOctahedralDeviatoric() const { return sqrt((2. / 3.) * this->GetInvariant_J2()); }
};

/// Class for stress tensors, in compact Voight notation
/// that is with 6 components in a column.

template <class Real = double>
class ChStressTensor : public ChVoightTensor<Real> {
  public:
    /// Compute the principal stresses for the given  tensor
    void ComputePrincipalStresses(double& e1, double& e2, double& e3) {
        ChVoightTensor<Real>::ComputeEigenvalues(e1, e2, e3);
    }

    /// Compute the directions of the principal stresses,
    /// i.e. three orthogonal directions for zero shear (diagonal stress)
    void ComputePrincipalStressesDirections(double& e1,
                                            double& e2,
                                            double& e3,
                                            ChVector<Real>& dir1,
                                            ChVector<Real>& dir2,
                                            ChVector<Real>& dir3) {
        ChVoightTensor<Real>::ComputeEigenvectors(e1, e2, e3, dir1, dir2, dir3);
    }
};

/// Class for strain tensors, in compact Voight notation
/// that is with 6 components in a column.

template <class Real = double>
class ChStrainTensor : public ChVoightTensor<Real> {
  public:
    /// Compute the principal strains for the given tensor
    void ComputePrincipalStrains(double& e1, double& e2, double& e3) {
        ChVoightTensor<Real>::ComputeEigenvalues(e1, e2, e3);
    }

    /// Compute the directions of the principal strain,
    /// i.e. three orthogonal directions for zero strain (diagonal strain)
    void ComputePrincipalStrainsDirections(double& e1,
                                           double& e2,
                                           double& e3,
                                           ChVector<Real>& dir1,
                                           ChVector<Real>& dir2,
                                           ChVector<Real>& dir3) {
        ChVoightTensor<Real>::ComputeEigenvectors(e1, e2, e3, dir1, dir2, dir3);
    }
};

}  // end namespace fea
}  // end namespace chrono

#endif
