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

#ifndef CH_TENSORS_H
#define CH_TENSORS_H

#include <cstdlib>
#include <cmath>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"

namespace chrono {

/// Base class for stress and strain tensors, in compact Voight notation.
/// This tensor representation uses 6 components in a column.
template <class Real = double>
class ChVoightTensor : public ChVectorN<Real, 6> {
  public:
    ChVoightTensor() : ChVectorN<Real, 6>() { this->setZero(); }

    ~ChVoightTensor() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChVoightTensor(const Eigen::MatrixBase<OtherDerived>& other) : ChVectorN<Real, 6>(other) {}

    /// Copy constructor, from a typical 3D rank-two stress or strain tensor (as 3x3 matrix).
    template <class RealB>
    inline ChVoightTensor(const ChMatrix33<RealB>& msource) {
        this->ConvertFromMatrix(msource);
    }

    /// This method allows assigning Eigen expressions to a ChVoightTensor.
    template <typename OtherDerived>
    ChVoightTensor& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 6, 1>::operator=(other);
        return *this;
    }

    inline Real& XX() { return (*this)(0); }
    inline const Real& XX() const { return (*this)(0); }

    inline Real& YY() { return (*this)(1); }
    inline const Real& YY() const { return (*this)(1); }

    inline Real& ZZ() { return (*this)(2); }
    inline const Real& ZZ() const { return (*this)(2); }

    inline Real& XY() { return (*this)(3); }
    inline const Real& XY() const { return (*this)(3); }

    inline Real& XZ() { return (*this)(4); }
    inline const Real& XZ() const { return (*this)(4); }

    inline Real& YZ() { return (*this)(5); }
    inline const Real& YZ() const { return (*this)(5); }

    /// Convert from a typical 3D rank-two stress or strain tensor (a 3x3 matrix).
    template <class RealB>
    void ConvertFromMatrix(const ChMatrix33<RealB>& msource) {
        XX() = (Real)msource(0, 0);
        YY() = (Real)msource(1, 1);
        ZZ() = (Real)msource(2, 2);
        XY() = (Real)msource(0, 1);
        XZ() = (Real)msource(0, 2);
        YZ() = (Real)msource(1, 2);
    }

    /// Convert to a typical 3D rank-two stress or strain tensor (a 3x3 matrix).
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

    /// Compute the volumetric part of the tensor, that is the trace V =Txx+Tyy+Tzz.
    Real GetVolumetricPart() const { return XX() + YY() + ZZ(); }

    /// Compute the deviatoric part of the tensor, storing it in mdeviatoric.
    void GetDeviatoricPart(ChVoightTensor<Real>& mdeviatoric) const {
        Real mM = GetVolumetricPart() * CH_1_3;
        mdeviatoric = *this;
        mdeviatoric.XX() -= mM;
        mdeviatoric.YY() -= mM;
        mdeviatoric.ZZ() -= mM;
    }

    /// Compute the I1 invariant.
    Real GetInvariant_I1() const { return XX() + YY() + ZZ(); }

    /// Compute the I2 invariant.
    Real GetInvariant_I2() const {
        return XX() * YY() + YY() * ZZ() + XX() * ZZ() - XY() * XY() - YZ() * YZ() - XZ() * XZ();
    }

    /// Compute the I3 invariant.
    Real GetInvariant_I3() const {
        return XX() * YY() * ZZ() + 2 * XY() * YZ() * XZ() - XY() * XY() * ZZ() - YZ() * YZ() * XX() -
               XZ() * XZ() * YY();
    }

    /// Compute the J1 invariant of the deviatoric part (that is always 0).
    Real GetInvariant_J1() const { return 0; }

    /// Compute the J2 invariant of the deviatoric part.
    Real GetInvariant_J2() const { return std::max(0.0, std::pow(GetInvariant_I1(), 2) * CH_1_3 - GetInvariant_I2()); }

    /// Compute the J3 invariant of the deviatoric part.
    Real GetInvariant_J3() const {
        return std::pow(GetInvariant_I1(), 3) * (2. / 27.) - GetInvariant_I1() * GetInvariant_I2() * CH_1_3 +
               GetInvariant_I3();
    }

    /// Rotate to another reference coordinate system, overwriting this tensor in place.
    void Rotate(ChMatrix33<Real> Rot) {
        ChMatrix33<Real> T;
        // do  T'= R*T*R'
        ConvertToMatrix(T);
        T = Rot * T * Rot.transpose();
        ConvertFromMatrix(T);  // to do, more efficient: unroll matrix multiplications and exploit T symmetry
    }

    /// Compute the eigenvalues (closed form method).
    void ComputeEigenvalues(double& e1, double& e2, double& e3) {
        double I1 = GetInvariant_I1();
        double I2 = GetInvariant_I2();
        double I3 = GetInvariant_I3();
        double phi = CH_1_3 * std::acos((2. * I1 * I1 * I1 - 9. * I1 * I2 + 27. * I3) /
                                           (2. * std::pow((I1 * I1 - 3 * I2), (3. / 2.))));
        double k = CH_2_3 * (std::sqrt(I1 * I1 - 3. * I2));
        e1 = (I1 * CH_1_3) + k * std::cos(phi);
        e2 = (I1 * CH_1_3) + k * std::cos(phi + CH_2_3 * chrono::CH_PI);
        e3 = (I1 * CH_1_3) + k * std::cos(phi + CH_4_3 * chrono::CH_PI);
    }

    /// Compute the eigenvectors and the eigenvalues.
    void ComputeEigenvectors(double& eigval1,
                             double& eigval2,
                             double& eigval3,
                             ChVector3<Real>& eigvector1,
                             ChVector3<Real>& eigvector2,
                             ChVector3<Real>& eigvector3) {
        ChMatrix33<Real> A;
        this->ConvertToMatrix(A);

        ChMatrix33<Real> vectors;
        ChVectorN<Real, 3> values;
        A.SelfAdjointEigenSolve(vectors, values);

        eigvector1 = vectors.col(0);
        eigvector2 = vectors.col(1);
        eigvector3 = vectors.col(3);
        eigval1 = values(0);
        eigval2 = values(1);
        eigval3 = values(2);
    }

    /// FORMULAS THAT ARE USEFUL FOR YELD CRITERIONS:

    /// Compute the Von Mises equivalent.
    double GetEquivalentVonMises() const {
        return std::sqrt(0.5 * (std::pow(XX() - YY(), 2.) + std::pow(YY() - ZZ(), 2.) + std::pow(ZZ() - XX(), 2.)) +
                         3.0 * (XY() * XY() + XZ() * XZ() + YZ() * YZ()));
    }

    /// Compute the mean hydrostatic value (aka volumetric, normal).
    double GetEquivalentMeanHydrostatic() const { return (this->GetInvariant_I1() * CH_1_3); }

    /// Compute the octahedral normal invariant (aka hydrostatic, volumetric).
    double GetEquivalentOctahedralNormal() const { return GetEquivalentMeanHydrostatic(); }

    /// Compute the octahedral deviatoric invariant (aka shear).
    double GetEquivalentOctahedralDeviatoric() const { return std::sqrt(CH_2_3 * GetInvariant_J2()); }
};

/// Class for stress tensors, in compact Voight notation that is with 6 components in a column.
template <class Real = double>
class ChStressTensor : public ChVoightTensor<Real> {
  public:
    /// Constructor (default empty).
    ChStressTensor() : ChVoightTensor<Real>() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChStressTensor(const Eigen::MatrixBase<OtherDerived>& other) : ChVoightTensor<Real>(other) {}

    /// This method allows assigning Eigen expressions to a ChStressTensor.
    template <typename OtherDerived>
    ChStressTensor& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 6, 1>::operator=(other);
        return *this;
    }

    /// Compute the principal stresses for the given tensor.
    void ComputePrincipalStresses(double& e1, double& e2, double& e3) {
        ChVoightTensor<Real>::ComputeEigenvalues(e1, e2, e3);
    }

    /// Compute the directions of the principal stresses,
    /// i.e. three orthogonal directions for zero shear (diagonal stress).
    void ComputePrincipalStressesDirections(double& e1,
                                            double& e2,
                                            double& e3,
                                            ChVector3<Real>& dir1,
                                            ChVector3<Real>& dir2,
                                            ChVector3<Real>& dir3) {
        ChVoightTensor<Real>::ComputeEigenvectors(e1, e2, e3, dir1, dir2, dir3);
    }
};

/// Class for strain tensors, in compact Voight notation that is with 6 components in a column.
template <class Real = double>
class ChStrainTensor : public ChVoightTensor<Real> {
  public:
    /// Constructor (default empty).
    ChStrainTensor() : ChVoightTensor<Real>() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChStrainTensor(const Eigen::MatrixBase<OtherDerived>& other) : ChVoightTensor<Real>(other) {}

    /// This method allows assigning Eigen expressions to a ChStrainTensor.
    template <typename OtherDerived>
    ChStrainTensor& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 6, 1>::operator=(other);
        return *this;
    }

    /// Compute the principal strains for the given tensor.
    void ComputePrincipalStrains(double& e1, double& e2, double& e3) {
        ChVoightTensor<Real>::ComputeEigenvalues(e1, e2, e3);
    }

    /// Compute the directions of the principal strain,
    /// i.e. three orthogonal directions for zero strain (diagonal strain).
    void ComputePrincipalStrainsDirections(double& e1,
                                           double& e2,
                                           double& e3,
                                           ChVector3<Real>& dir1,
                                           ChVector3<Real>& dir2,
                                           ChVector3<Real>& dir3) {
        ChVoightTensor<Real>::ComputeEigenvectors(e1, e2, e3, dir1, dir2, dir3);
    }
};

}  // end namespace chrono

#endif
