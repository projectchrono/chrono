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

#ifndef CHBEAMSECTIONTAPEREDTIMOSHENKOFPM_H
#define CHBEAMSECTIONTAPEREDTIMOSHENKOFPM_H

#include "chrono/core/ChMath.h"
#include "chrono/fea/ChBeamSectionTaperedTimoshenko.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// For composite beams such as wind turbine blades and helicopter rotor blades, the cross-sectional
/// stiffness properties in axial, shear, bending and torsion directions are coupled with each other,
/// hence the fully-populated matrix(FPM) of cross-sectional stiffness properties is used to describe
/// this complex coupling.
/// Base class for all constitutive models of fully-populated matrix(FPM) sections of Timoshenko beams.
/// To be used with ChElementBeamTaperedTimoshenkoFPM.
class ChApi ChBeamSectionTimoshenkoAdvancedGenericFPM : public ChBeamSectionTimoshenkoAdvancedGeneric {
  protected:
    /// material fully-populated stiffness matrix of cross-section,
    /// given in centerline reference, measured along centerline main axes.
    /// The shear principal axis orientation is assumed same as elastic principal axis.
    /// So if you want to consider the different orientation of shear principal axis, please rotate the 
    /// block of shear terms from shear principal axis orientation to elastic principal axis orientation
    /// firstly (do this rotation transformation outside by yourself) and then input it here.
    ChMatrixNM<double, 6, 6> Klaw;

    /// material fully-populated mass matrix of cross-section,
    /// given in centerline reference, measured along centerline main axes
    ChMatrixNM<double, 6, 6> Mlaw;

  public:
    ChBeamSectionTimoshenkoAdvancedGenericFPM() {
        Klaw.setIdentity(6, 6);
        Mlaw.setIdentity(6, 6);
    }

    ChBeamSectionTimoshenkoAdvancedGenericFPM(
        const ChMatrixNM<double, 6, 6> mKlaw,  ///< material stiffness matrix of cross-section
        const ChMatrixNM<double, 6, 6> mMlaw,  ///< material mass matrix of cross-section
        const double malpha,                   ///< section rotation about elastic center [rad]
        const double mCy,                      ///< elastic center y displacement respect to centerline
        const double mCz,                      ///< elastic center z displacement respect to centerline
        const double mSy,                      ///< shear center y displacement respect to centerline
        const double mSz,                      ///< shear center z displacement respect to centerline
        const double mmu,                      ///< mass per unit length
        const double
            mJyy,  ///< inertia Jyy per unit lenght, in centerline reference, measured along centerline main axes
        const double
            mJzz,  ///< inertia Jzz per unit lenght, in centerline reference, measured along centerline main axes
        const double mJyz =
            0,  ///< inertia Jyz per unit lenght, in centerline reference, measured along centerline main axes const
        const double mQy =
            0,  ///< inertia Qy per unit lenght, in centerline reference, measured along centerline main axes
        const double mQz =
            0,  ///< inertia Qz per unit lenght, in centerline reference, measured along centerline main axes
        const double mMy = 0,  ///< mass center y displacement respect to centerline
        const double mMz = 0)  ///< mass center z displacement respect to centerline
    {
        Klaw = mKlaw;
        Mlaw = mMlaw;
        Ax = mKlaw(0, 0);    // axial stiffness along xx, in the diagonal elements of FPM Klaw
        GAyy = mKlaw(1, 1);  // shear stiffness along yy, in the diagonal elements of FPM Klaw
        GAzz = mKlaw(2, 2);  // shear stiffness along zz, in the diagonal elements of FPM Klaw
        Txx = mKlaw(3, 3);   // torsional stiffness about xx, in the diagonal elements of FPM Klaw
        Byy = mKlaw(4, 4);   // bending stiffness about yy, in the diagonal elements of FPM Klaw
        Bzz = mKlaw(5, 5);   // bending stiffness about zz, in the diagonal elements of FPM Klaw
        alpha = malpha;
        Cy = mCy;
        Cz = mCz;
        Sy = mSy;
        Sz = mSz;
        mu = mmu;
        Jyy = mJyy;
        Jzz = mJzz;
        Jyz = mJyz;
        Qy = mQy;
        Qz = mQz;
        My = mMy;
        Mz = mMz;
    }

    virtual ~ChBeamSectionTimoshenkoAdvancedGenericFPM() {}

    /// Set the material stiffness matrix of cross-section in fully-polulated format(FPM),
    /// and assign the traditional axial, torsional, shear and bending stiffnesses automatically.
    virtual void SetStiffnessMatrixFPM(const ChMatrixNM<double, 6, 6> mKlaw) {
        this->Klaw = mKlaw;
        this->Ax = mKlaw(0, 0);
        this->GAyy = mKlaw(1, 1);
        this->GAzz = mKlaw(2, 2);
        this->Txx = mKlaw(3, 3);
        this->Byy = mKlaw(4, 4);
        this->Bzz = mKlaw(5, 5);
    }

    /// Get the material stiffness matrix of cross-section in fully-polulated format(FPM)
    virtual ChMatrixNM<double, 6, 6>& GetStiffnessMatrixFPM() { return this->Klaw; }

    /// Set the material mass matrix of cross-section in fully-polulated format(FPM) directly.
    /// This cross-sectional mass matrix should be given in the centerline reference.
    virtual void SetMassMatrixFPM(const ChMatrixNM<double, 6, 6> mMlaw) {
        this->Mlaw = mMlaw;

        this->mu = (mMlaw(0, 0) + mMlaw(1, 1) + mMlaw(2, 2)) / 3.0;
        this->Jxx = mMlaw(3, 3);
        this->Jyy = mMlaw(4, 4);
        this->Jzz = mMlaw(5, 5);
        this->Jyz = -mMlaw(4, 5);
        this->Qy = mMlaw(0, 4);
        this->Qz = -mMlaw(0, 5);
    }

    /// Set the material mass matrix of cross-section in fully-polulated format(FPM).
    /// Assign the mass properties at mass center, then calcualte Mlaw automatically.
    virtual void SetMassMatrixFPM(double mmu,
                                  double Jmyy,
                                  double Jmzz,
                                  double Jmyz,
                                  double mass_phi,
                                  double Qmy,
                                  double Qmz,
                                  double mMy,
                                  double mMz) {
        this->mu = mmu;
        this->My = mMy;
        this->Mz = mMz;

        // transform the mass properties from mass center to centerline reference
        this->SetMainInertiasInMassReference(Jmyy, Jmzz, Jmyz, mass_phi, Qmy, Qmz);

        // fill the cross-sectional mass matrix Mlaw
        this->Mlaw.setIdentity();
        this->Mlaw.row(0) << mu, 0, 0, 0, Qy, -Qz;
        this->Mlaw.row(1) << 0, mu, 0, -Qy, 0, 0;
        this->Mlaw.row(2) << 0, 0, mu, Qz, 0, 0;
        this->Mlaw.row(3) << 0, -Qy, Qz, Jxx, 0, 0;
        this->Mlaw.row(4) << Qy, 0, 0, 0, Jyy, -Jyz;
        this->Mlaw.row(5) << -Qz, 0, 0, 0, -Jyz, Jzz;
    }

    /// Set the material mass matrix of cross-section in fully-polulated format(FPM).
    /// Assign the mass properties at centerline reference, then calcualte Mlaw automatically
    virtual void SetMassMatrixFPM(double mmu, double mJyy, double mJzz, double mJyz, double mQy, double mQz) {
        this->mu = mmu;
        this->Jxx = mJyy + mJzz;
        this->Jyy = mJyy;
        this->Jzz = mJzz;
        this->Jyz = mJyz;
        this->Qy = mQy;
        this->Qz = mQz;

        // fill the cross-sectional mass matrix Mlaw
        this->Mlaw.setIdentity();
        this->Mlaw.row(0) << mu, 0, 0, 0, Qy, -Qz;
        this->Mlaw.row(1) << 0, mu, 0, -Qy, 0, 0;
        this->Mlaw.row(2) << 0, 0, mu, Qz, 0, 0;
        this->Mlaw.row(3) << 0, -Qy, Qz, Jxx, 0, 0;
        this->Mlaw.row(4) << Qy, 0, 0, 0, Jyy, -Jyz;
        this->Mlaw.row(5) << -Qz, 0, 0, 0, -Jyz, Jzz;
    }

    /// Get the material mass matrix of cross-section in fully-polulated format(FPM).
    virtual ChMatrixNM<double, 6, 6>& GetMassMatrixFPM() { return this->Mlaw; }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Base class for all constitutive models of fully-populated matrix(FPM) sections of Tapered Timoshenko beams.
/// This tapered FPM section consists of two objects of ChBeamSectionTimoshenkoAdvancedGenericFPM.
/// To be used with ChElementBeamTaperedTimoshenkoFPM.
class ChApi ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM : public ChBeamSectionTaperedTimoshenkoAdvancedGeneric {
  public:
    ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM() {}
    virtual ~ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM() {}

    /// Set the FPM section & material of beam element at end A.
    void SetSectionA(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> my_material) {
        section_fpmA = my_material;
        sectionA = std::dynamic_pointer_cast<ChBeamSectionTimoshenkoAdvancedGeneric>(my_material);
    }

    /// Set the FPM section & material of beam element at end B.
    void SetSectionB(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> my_material) {
        section_fpmB = my_material;
        sectionB = std::dynamic_pointer_cast<ChBeamSectionTimoshenkoAdvancedGeneric>(my_material);
    }

    /// Get the FPM section & material of beam element at end A.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> GetSectionA() { return section_fpmA; }
    /// Get the FPM section & material of beam element at end B.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> GetSectionB() { return section_fpmB; }

    /// Get the average FPM(stiffness FPM) of beam element with two ends.
    ChMatrixNM<double, 6, 6> GetAverageFPM() { return average_fpm; }

    /// Get the average stiffness FPM of beam element with two ends.
    ChMatrixNM<double, 6, 6> GetAverageKlaw();

    /// Get the average mass FPM of beam element with two ends.
    ChMatrixNM<double, 6, 6> GetAverageMlaw();

    /// Get the stiffness FPM of beam element at abscissa eta.
    ChMatrixNM<double, 6, 6> GetKlawAtPoint(const double eta);

    /// Get the mass FPM of beam element at abscissa eta.
    ChMatrixNM<double, 6, 6> GetMlawAtPoint(const double eta);

    /// Get the damping FPM of beam element at abscissa eta.
    ChMatrixNM<double, 6, 6> GetRlawAtPoint(const double eta);

    /// Compute the 12x12 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix)
    virtual void ComputeInertiaMatrix(ChMatrixDynamic<>& M  ///< 12x12 sectional mass matrix values here
    );

  protected:
    /// The FPM section & material of beam element at end A.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> section_fpmA;
    /// The FPM section & material of beam element at end B.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> section_fpmB;

    /// Some important average section parameters, to calculate once, enable to access them conveniently.
    ChMatrixNM<double, 6, 6> average_fpm;

    /// Compute the average stiffness FPM of cross-section.
    virtual void ComputeAverageFPM();

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
