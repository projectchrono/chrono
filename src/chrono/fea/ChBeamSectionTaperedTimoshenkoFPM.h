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

/// Base class for all constitutive models of sections of Euler beams.
/// To be used with ChElementBeamEuler.
/// For practical purposes, either you use the concrete inherited classes like ChBeamSectionEulerSimple,
/// ChBeamSectionEulerAdvanced etc., or you inherit your class from this.
class ChApi ChBeamSectionTimoshenkoAdvancedGenericFPM : public ChBeamSectionTimoshenkoAdvancedGeneric {
protected:
	// material stiffness matrix of cross-section, 
	// given at elastic center and elastic principal axis orientation,
	// the shear principal axis orientation is assumed same as elastic one.
	ChMatrixNM<double, 6, 6> Klaw;

	ChMatrixNM<double, 6, 6> Mlaw;

  public:
    ChBeamSectionTimoshenkoAdvancedGenericFPM()
        : Klaw(ChMatrixNM<double, 6, 6>::Identity(6, 6)), 
		Mlaw(ChMatrixNM<double, 6, 6>::Identity(6, 6)) {}

	ChBeamSectionTimoshenkoAdvancedGenericFPM(
		const ChMatrixNM<double, 6, 6> mKlaw,
        const ChMatrixNM<double, 6, 6> mMlaw,
		const double malpha,  ///< section rotation about elastic center [rad]
		const double mCy,     ///< elastic center y displacement respect to centerline
		const double mCz,     ///< elastic center z displacement respect to centerline
		const double mSy,     ///< shear center y displacement respect to centerline
		const double mSz,     ///< shear center z displacement respect to centerline
		const double mmu,     ///< mass per unit length
		const double
		mJyy,  ///< inertia Jyy per unit lenght, in centerline reference, measured along centerline main axes
		const double
		mJzz,  ///< inertia Jzz per unit lenght, in centerline reference, measured along centerline main axes
		const double
		mJyz = 0,  ///< inertia Jyz per unit lenght, in centerline reference, measured along centerline main axes const
		const double mQy = 0,  ///< inertia Qy per unit lenght, in centerline reference, measured along centerline main axes
		const double mQz = 0,  ///< inertia Qz per unit lenght, in centerline reference, measured along centerline main axes
		const double mMy = 0,  ///< mass center y displacement respect to centerline
		const double mMz = 0)  ///< mass center z displacement respect to centerline
	{
		Klaw = mKlaw;
        Mlaw = mMlaw;
		Ax = mKlaw(0, 0);
		GAyy = mKlaw(1, 1);
		GAzz = mKlaw(2, 2);
		Txx = mKlaw(3, 3);
		Byy = mKlaw(4, 4);
		Bzz = mKlaw(5, 5);
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

	virtual void SetStiffnessMatrixFPM(const ChMatrixNM<double, 6, 6> mKlaw) {
		this->Klaw = mKlaw;
		this->Ax = mKlaw(0, 0);
		this->GAyy = mKlaw(1, 1);
		this->GAzz = mKlaw(2, 2);
		this->Txx = mKlaw(3, 3);
		this->Byy = mKlaw(4, 4);
		this->Bzz = mKlaw(5, 5);
	}

	virtual ChMatrixNM<double, 6, 6>& GetStiffnessMatrixFPM() { return this->Klaw; }

	// This cross-sectional mass matrix should be given at the centerline
	virtual void SetMassMatrixFPM(const ChMatrixNM<double, 6, 6> mMlaw) {
        this->Mlaw = mMlaw;

        this->mu = (mMlaw(0, 0) + mMlaw(1, 1) + mMlaw(2, 2))/3.0;
		this->Jxx = mMlaw(3, 3);
        this->Jyy = mMlaw(4, 4);
        this->Jzz = mMlaw(5, 5);
        this->Jyz = -mMlaw(4, 5);
        this->Qy = mMlaw(0, 4);
        this->Qz = -mMlaw(0, 5);
    }

	// assign the mass properties at mass center, calcualte the Mlaw automatically
	virtual void SetMassMatrixFPM(double mmu, double Jmyy, double Jmzz, double Jmyz, double mass_phi, double Qmy, double Qmz, double mMy, double mMz) {
        this->mu = mmu;
        this->My = mMy;
        this->Mz = mMz;

		// transform the mass properties from mass center to centerline
		this->SetMainInertiasInMassReference(Jmyy,Jmzz,Jmyz,mass_phi,Qmy,Qmz);
		
		// fill the cross-sectional mass matrix Mlaw
        this->Mlaw.row(0) << mu, 0, 0, 0, Qy, -Qz;
        this->Mlaw.row(1) << 0, mu, 0, -Qy, 0, 0;
        this->Mlaw.row(2) << 0, 0, mu, Qz, 0, 0;
        this->Mlaw.row(3) << 0, -Qy, Qz, Jxx, 0, 0;
        this->Mlaw.row(4) << Qy, 0, 0, 0, Jyy, -Jyz;
        this->Mlaw.row(5) << -Qz, 0, 0, 0, -Jyz, Jzz;
    }

    // assign the mass properties at centerline, calcualte the Mlaw automatically
	virtual void SetMassMatrixFPM(double mmu, double mJyy, double mJzz, double mJyz, double mQy, double mQz) {
        this->mu = mmu;
        this->Jxx = mJyy + mJzz;
        this->Jyy = mJyy;
        this->Jzz = mJzz;
        this->Jyz = mJyz;
        this->Qy = mQy;
        this->Qz = mQz;

        // fill the cross-sectional mass matrix Mlaw
        this->Mlaw.row(0) << mu, 0, 0, 0, Qy, -Qz;
        this->Mlaw.row(1) << 0, mu, 0, -Qy, 0, 0;
        this->Mlaw.row(2) << 0, 0, mu, Qz, 0, 0;
        this->Mlaw.row(3) << 0, -Qy, Qz, Jxx, 0, 0;
        this->Mlaw.row(4) << Qy, 0, 0, 0, Jyy, -Jyz;
        this->Mlaw.row(5) << -Qz, 0, 0, 0, -Jyz, Jzz;
    }

    virtual ChMatrixNM<double, 6, 6>& GetMassMatrixFPM() { return this->Mlaw; }

};

class ChApi ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM : public ChBeamSectionTaperedTimoshenkoAdvancedGeneric {
public:

	ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM() {}
	virtual ~ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM() {}


	/// Set the section & material of beam element .
	/// It is a shared property, so it can be shared between other beams.
    void SetSectionA(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> my_material) {
        section_fpmA = my_material;
        sectionA = std::dynamic_pointer_cast<ChBeamSectionTimoshenkoAdvancedGeneric>(my_material);
    }
    void SetSectionB(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> my_material) {
        section_fpmB = my_material;
        sectionB = std::dynamic_pointer_cast<ChBeamSectionTimoshenkoAdvancedGeneric>(my_material);
    }

	/// Get the section & material of the element
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> GetSectionA() { return section_fpmA; }
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> GetSectionB() { return section_fpmB; }

	ChMatrixNM<double, 6, 6> GetAverageFPM() { return average_fpm; }
    ChMatrixNM<double, 6, 6> GetAverageKlaw();
    ChMatrixNM<double, 6, 6> GetAverageMlaw();
    ChMatrixNM<double, 6, 6> GetKlawAtPoint(const double eta);
    ChMatrixNM<double, 6, 6> GetMlawAtPoint(const double eta);
    ChMatrixNM<double, 6, 6> GetRlawAtPoint(const double eta);

	/// Compute the 12x12 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix)
    virtual void ComputeInertiaMatrix(ChMatrixDynamic<>& M  ///< 12x12 sectional mass matrix values here
    );

protected:
	std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> section_fpmA;
	std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM> section_fpmB;

	// Some important average section parameters, to calculate once, enable to access them conveniently.
	ChMatrixNM<double, 6, 6> average_fpm;

	virtual void ComputeAverageFPM();

};

	/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
