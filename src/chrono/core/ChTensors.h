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

/// Base class for storing 3x3 symmetric tensors as vectors, with the compact 
/// Voigt notation. This is often useful for storing a 3D stress or a strain.
/// This tensor representation uses 6 components in a column, corresponding
/// to the 3x3 tensor indexes in this order: 11 22 33 23 13 12.
/// NOTE 1: ordering matters! there are some books that use a different 
/// ordering for the last 3 elements.
/// NOTE 2: for stress tensors, S={S_11 S_22 S_33 S_23 S_13 S_12}, but
/// for "engineering" strain tensors, same order but E={E_11 E_22 E_33 2*E_23 2*E_13 2*E_12}
/// with the 2* factor that preserves the work-conjugacy if doing w=S^T * E,
/// where the 2* factor is automatically managed by ChStrainEngTensor subclass thanks
/// to having made the GetXY() GetXZ() GetYZ() functions as virtual.

template <class Real = double>
class ChVoigtTensor : public ChVectorN<Real, 6> {
  public:
    ChVoigtTensor() : ChVectorN<Real, 6>() { this->setZero(); }

    ~ChVoigtTensor() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChVoigtTensor(const Eigen::MatrixBase<OtherDerived>& other) : ChVectorN<Real, 6>(other) {}

    /// Copy constructor, from a typical 3D rank-two stress or strain tensor (as 3x3 matrix).
    template <class RealB>
    inline ChVoigtTensor(const ChMatrix33<RealB>& msource) {
        this->ConvertFromMatrix(msource);
    }

    /// This method allows assigning Eigen expressions to a ChVoigtTensor.
    template <typename OtherDerived>
    ChVoigtTensor& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 6, 1>::operator=(other);
        return *this;
    }

    inline Real& XX() { return (*this)(0); }
    inline const Real& XX() const { return (*this)(0); }

    inline Real& YY() { return (*this)(1); }
    inline const Real& YY() const { return (*this)(1); }

    inline Real& ZZ() { return (*this)(2); }
    inline const Real& ZZ() const { return (*this)(2); }

    virtual Real GetYZ() const = 0;
    virtual void SetYZ(const Real d) = 0;

    virtual Real GetXZ() const = 0;
    virtual void SetXZ(const Real d) = 0;

    virtual Real GetXY() const = 0;
    virtual void SetXY(const Real d) = 0;

    /// Convert from a typical 3D rank-two symmetric tensor (a 3x3 matrix), ex. stress or strain
    template <class RealB>
    void ConvertFromMatrix(const ChMatrix33<RealB>& msource) {
        XX() = (Real)msource(0, 0);
        YY() = (Real)msource(1, 1);
        ZZ() = (Real)msource(2, 2);
        SetXY((Real)msource(0, 1));
        SetXZ((Real)msource(0, 2));
        SetYZ((Real)msource(1, 2));
    }

    /// Convert to a typical 3D rank-two symmetric tensor (a 3x3 matrix), ex. stress or strain
    template <class RealB>
    void ConvertToMatrix(ChMatrix33<RealB>& mdest) const {
        mdest(0, 0) = (RealB)XX();
        mdest(1, 1) = (RealB)YY();
        mdest(2, 2) = (RealB)ZZ();
        mdest(0, 1) = (RealB)GetXY();
        mdest(0, 2) = (RealB)GetXZ();
        mdest(1, 2) = (RealB)GetYZ();
        mdest(1, 0) = (RealB)GetXY();
        mdest(2, 0) = (RealB)GetXZ();
        mdest(2, 1) = (RealB)GetYZ();
    }

    /// Compute the volumetric part of the tensor, that is the trace V =Txx+Tyy+Tzz.
    Real GetVolumetricPart() const { return XX() + YY() + ZZ(); }

    /// Compute the deviatoric part of the tensor, storing it in mdeviatoric.
    void GetDeviatoricPart(ChVoigtTensor<Real>& mdeviatoric) const {
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
        return XX() * YY() + YY() * ZZ() + XX() * ZZ() - GetXY() * GetXY() - GetYZ() * GetYZ() - GetXZ() * GetXZ();
    }

    /// Compute the I3 invariant.
    Real GetInvariant_I3() const {
        return XX() * YY() * ZZ() + 2 * GetXY() * GetYZ() * GetXZ() - GetXY() * GetXY() * ZZ() - GetYZ() * GetYZ() * XX() -
               GetXZ() * GetXZ() * YY();
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
                         3.0 * (GetXY() * GetXY() + GetXZ() * GetXZ() + GetYZ() * GetYZ()));
    }

    /// Compute the mean hydrostatic value (aka volumetric, normal).
    double GetEquivalentMeanHydrostatic() const { return (this->GetInvariant_I1() * CH_1_3); }

    /// Compute the octahedral normal invariant (aka hydrostatic, volumetric).
    double GetEquivalentOctahedralNormal() const { return GetEquivalentMeanHydrostatic(); }

    /// Compute the octahedral deviatoric invariant (aka shear).
    double GetEquivalentOctahedralDeviatoric() const { return std::sqrt(CH_2_3 * GetInvariant_J2()); }
};



/// Class for stress tensors, in compact Voigt notation that is 
/// with 6 components in a column.
/// Ordering is as in  S={S_11 S_22 S_33 S_23 S_13 S_12}
/// This is work-conjugate to ChStrainEngTensor, NOT ChStrainTensor !

template <class Real = double>
class ChStressTensor : public ChVoigtTensor<Real> {
  public:
    /// Constructor (default empty).
    ChStressTensor() : ChVoigtTensor<Real>() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChStressTensor(const Eigen::MatrixBase<OtherDerived>& other) : ChVoigtTensor<Real>(other) {}

    /// This method allows assigning Eigen expressions to a ChStressTensor.
    template <typename OtherDerived>
    ChStressTensor& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 6, 1>::operator=(other);
        return *this;
    }

    virtual Real GetYZ() const override { return (*this)(3); };
    virtual void SetYZ(const Real d) override { (*this)(3) = d; };

    virtual Real GetXZ() const override { return (*this)(4); };
    virtual void SetXZ(const Real d) override { (*this)(4) = d; };

    virtual Real GetXY() const override { return (*this)(5); };
    virtual void SetXY(const Real d) override { (*this)(5) = d; };


    /// Compute the principal stresses for the given tensor.
    void ComputePrincipalStresses(double& e1, double& e2, double& e3) {
        ChVoigtTensor<Real>::ComputeEigenvalues(e1, e2, e3);
    }

    /// Compute the directions of the principal stresses,
    /// i.e. three orthogonal directions for zero shear (diagonal stress).
    void ComputePrincipalStressesDirections(double& e1,
                                            double& e2,
                                            double& e3,
                                            ChVector3<Real>& dir1,
                                            ChVector3<Real>& dir2,
                                            ChVector3<Real>& dir3) {
        ChVoigtTensor<Real>::ComputeEigenvectors(e1, e2, e3, dir1, dir2, dir3);
    }
};

/// Class for strain tensors, in compact Voigt notation that is 
/// with 6 components in a column.
/// NOTE : this is NOT an "engineering" strain in the sense that the
/// shear components are NOT multiplied by 2 in the last 3 components, here we have:
///     E={E_11 E_22 E_33 E_23 E_13 E_12}
/// so if you need E={E_11 E_22 E_33 2*E_23 2*E_13 2*E_12} you must use
/// the alternative ChStrainEngTensor.
/// Because it misses the 2* factor, this ChStrainTensor is NOT work-conjugate 
/// to ChStressTensor, unlike ChStrainEngTensor.

template <class Real = double>
class ChStrainTensor : public ChVoigtTensor<Real> {
  public:
    /// Constructor (default empty).
    ChStrainTensor() : ChVoigtTensor<Real>() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChStrainTensor(const Eigen::MatrixBase<OtherDerived>& other) : ChVoigtTensor<Real>(other) {}

    /// This method allows assigning Eigen expressions to a ChStrainTensor.
    template <typename OtherDerived>
    ChStrainTensor& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 6, 1>::operator=(other);
        return *this;
    }

    virtual Real GetYZ() const override { return (*this)(3); };
    virtual void SetYZ(const Real d) override { (*this)(3) = d; };

    virtual Real GetXZ() const override { return (*this)(4); };
    virtual void SetXZ(const Real d) override { (*this)(4) = d; };

    virtual Real GetXY() const override { return (*this)(5); };
    virtual void SetXY(const Real d) override { (*this)(5) = d; };

    /// Compute the principal strains for the given tensor.
    void ComputePrincipalStrains(double& e1, double& e2, double& e3) {
        ChVoigtTensor<Real>::ComputeEigenvalues(e1, e2, e3);
    }

    /// Compute the directions of the principal strain,
    /// i.e. three orthogonal directions for zero strain (diagonal strain).
    void ComputePrincipalStrainsDirections(double& e1,
                                           double& e2,
                                           double& e3,
                                           ChVector3<Real>& dir1,
                                           ChVector3<Real>& dir2,
                                           ChVector3<Real>& dir3) {
        ChVoigtTensor<Real>::ComputeEigenvectors(e1, e2, e3, dir1, dir2, dir3);
    }
};


/// Class for "engineering" strain tensors, in compact Voigt notation that is
/// with 6 components in a column.
/// NOTE : for "engineering" strain tensors, same order of strain tensors but
/// assuming engineering strain convention, we store the vector
/// with the tensorial shear strain doubled, as in:
///     E={E_11 E_22 E_33 2*E_23 2*E_13 2*E_12}
/// The 2* factor that preserves the work-conjugacy if doing w=S^T * E.
/// NOTE! the 2* factor is automatically managed by ChStrainEngTensor thanks
/// to having made the GetXY() GetXZ() GetYZ() functions as virtual.
/// Therefore the 2* factor that preserves the work-conjugacy if doing w=S^T * E,
/// and this ChStrainEngTensor is work conjugate to ChStressTensor.

template <class Real = double>
class ChStrainEngTensor : public ChVoigtTensor<Real> {
  public:
    /// Constructor (default empty).
    ChStrainEngTensor() : ChVoigtTensor<Real>() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChStrainEngTensor(const Eigen::MatrixBase<OtherDerived>& other) : ChVoigtTensor<Real>(other) {}

    /// This method allows assigning Eigen expressions to a ChStrainEngTensor.
    template <typename OtherDerived>
    ChStrainEngTensor& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 6, 1>::operator=(other);
        return *this;
    }

    virtual Real GetYZ() const override { return 0.5 * (*this)(3); };
    virtual void SetYZ(const Real d) override { (*this)(3) = 2.0 * d; };

    virtual Real GetXZ() const override { return 0.5 * (*this)(4); };
    virtual void SetXZ(const Real d) override { (*this)(4) = 2.0 * d; };

    virtual Real GetXY() const override { return 0.5 * (*this)(5); };
    virtual void SetXY(const Real d) override { (*this)(5) = 2.0 * d; };

    /// Compute the principal strains for the given tensor.
    void ComputePrincipalStrains(double& e1, double& e2, double& e3) { ChVoigtTensor<Real>::ComputeEigenvalues(e1, e2, e3); }

    /// Compute the directions of the principal strain,
    /// i.e. three orthogonal directions for zero strain (diagonal strain).
    void ComputePrincipalStrainsDirections(double& e1, double& e2, double& e3, ChVector3<Real>& dir1, ChVector3<Real>& dir2, ChVector3<Real>& dir3) {
        ChVoigtTensor<Real>::ComputeEigenvectors(e1, e2, e3, dir1, dir2, dir3);
    }
};


}  // end namespace chrono

#endif
