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

#ifndef CHBEAMSECTIONTAPEREDTIMOSHENKO_H
#define CHBEAMSECTIONTAPEREDTIMOSHENKO_H

#include "chrono/core/ChMath.h"
#include "chrono/fea/ChBeamSectionEuler.h"

namespace chrono {
namespace fea {

// This damping model would be helpful for those anisotropic material, such as wind turbine blade.
// Refer to:
// [1]. Hansen, M. H. (2001). Anisotropic damping of Timoshenko beam elements. (Denmark. Forskningscenter Risoe.
//      Risoe-R; No. 1267(EN)).

struct DampingCoefficients {
    double bx;  ///< damping coefficient along x axis (axial)
    double by;  ///< damping coefficient along y axis (shear) and about z axis (bending)
    double bz;  ///< damping coefficient along z axis (shear) and about y axis (bending)
    double bt;  ///< damping coefficient about x axis (torsion)
};

struct AverageSectionParameters {
    double mu;
    double alpha;
    double Cy;
    double Cz;
    double Sy;
    double Sz;
    double My;
    double Mz;

    double Jyy;
    double Jzz;
    double Jyz;
    double Jxx;
    double Qy;
    double Qz;

    double mass_phi;
    double Jmyy;
    double Jmzz;
    double Jmyz;
    double Jmxx;
    double Qmy;
    double Qmz;

    double EA;
    double GJ;
    double GAyy;
    double GAzz;
    double EIyy;
    double EIzz;

    double GAmyy;
    double GAmzz;
    double EImyy;
    double EImzz;

    double phimy;
    double phimz;
    double phiy;
    double phiz;

    DampingCoefficients rdamping_coeff;
};

/// @addtogroup fea_utils
/// @{

/// Base class for all constitutive models of sections of Euler beams.
/// To be used with ChElementBeamEuler.
/// For practical purposes, either you use the concrete inherited classes like ChBeamSectionEulerSimple,
/// ChBeamSectionEulerAdvanced etc., or you inherit your class from this.
class ChApi ChBeamSectionTimoshenkoAdvancedGeneric : public ChBeamSectionRayleighAdvancedGeneric {
  protected:
    double GAyy;  // shear rigidity along yy
    double GAzz;  // shear rigidity along zz

    //   These are defined as :
    /// \f$ Q_{y} =  \int_\Omega \rho z d\Omega \f$,
    /// \f$ Q_{z} =  \int_\Omega \rho y d\Omega \f$,
    double Qy;  // mass moment of area
    double Qz;  // mass moment of area

    DampingCoefficients rdamping_coeff;

  public:
    ChBeamSectionTimoshenkoAdvancedGeneric() : GAyy(0), GAzz(0), Qy(0), Qz(0) {
        double bcoeff = 0.001;
        rdamping_coeff.bx = bcoeff;
        rdamping_coeff.by = bcoeff;
        rdamping_coeff.bz = bcoeff;
        rdamping_coeff.bt = bcoeff;
    }

    ChBeamSectionTimoshenkoAdvancedGeneric(
        const double mAx,                          ///< axial rigidity
        const double mTxx,                         ///< torsion rigidity
        const double mByy,                         ///< bending regidity about yy
        const double mBzz,                         ///< bending rigidity about zz
        const double mGAyy,                        ///< shear rigidity along yy
        const double mGAzz,                        ///< shear rigidity along zz
        const DampingCoefficients mdamping_coeff,  /// damping coefficients
        const double malpha,                       ///< section rotation about elastic center [rad]
        const double mCy,                          ///< elastic center y displacement respect to centerline
        const double mCz,                          ///< elastic center z displacement respect to centerline
        const double mSy,                          ///< shear center y displacement respect to centerline
        const double mSz,                          ///< shear center z displacement respect to centerline
        const double mmu,                          ///< mass per unit length
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
        const double mMz = 0   ///< mass center z displacement respect to centerline
        )
        : ChBeamSectionRayleighAdvancedGeneric(mAx,
                                               mTxx,
                                               mByy,
                                               mBzz,
                                               malpha,
                                               mCy,
                                               mCz,
                                               mSy,
                                               mSz,
                                               mmu,
                                               mJyy,
                                               mJzz,
                                               mJyz,
                                               mMy,
                                               mMz),
          GAyy(mGAyy),
          GAzz(mGAzz),
          Qy(mQy),
          Qz(mQz),
          rdamping_coeff(mdamping_coeff) {}

    virtual ~ChBeamSectionTimoshenkoAdvancedGeneric() {}

    /// Sets the shear rigidity, for shearing along Y axis, at shear center,
    /// usually Ayy*G for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetYshearRigidity(const double mv) { GAyy = mv; }

    /// Sets the shear rigidity, for shearing along Z axis, at shear center,
    /// usually Azz*G for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetZshearRigidity(const double mv) { GAzz = mv; }

    /// Gets the shear rigidity, for shearing along Y axis at shear center, usually Ayy*G, but might be ad hoc
    virtual double GetYshearRigidity() const { return this->GAyy; }

    /// Gets the shear rigidity, for shearing along Z axis at shear center, usually Azz*G, but might be ad hoc
    virtual double GetZshearRigidity() const { return this->GAzz; }

    virtual void SetBeamRaleyghDamping(DampingCoefficients mdamping_coeff) { this->rdamping_coeff = mdamping_coeff; }
    virtual DampingCoefficients GetBeamRaleyghDamping() const { return this->rdamping_coeff; }

    virtual void SetInertiasPerUnitLength(const double mJyy,
                                          const double mJzz,
                                          const double mJyz,
                                          const double mQy,
                                          const double mQz);

    virtual void SetMainInertiasInMassReference(const double Jmyy,
                                                const double Jmzz,
                                                const double Jmyz,
                                                const double mass_phi,
                                                const double Qmy,
                                                const double Qmz);

    virtual void GetMainInertiasInMassReference(double& Jmyy,
                                                double& Jmzz,
                                                double& Jmyz,
                                                double& mass_phi,
                                                double& Qmy,
                                                double& Qmz);

    virtual double GetInertiaJxxPerUnitLengthInMassReference() const {
        // WARNING: It is recommended to  use GetMainInertiasInMassReference() instead.
        // because the below algorithm has ingored Qmy,Qmz, although they are always tiny values.
        GetLog() << "Warm warning: it is recommended to use GetMainInertiasInMassReference() instead, "
                 << "and do calculation: Jmxx = Jmyy+Jmzz \n";
        return this->Jxx - this->mu * this->Mz * this->Mz - this->mu * this->My * this->My;
    };

    virtual double GetInertiaJyyPerUnitLength() const { return this->Jyy; }
    virtual double GetInertiaJzzPerUnitLength() const { return this->Jzz; }
    virtual double GetInertiaJyzPerUnitLength() const { return this->Jyz; }
    virtual double GetInertiaQyPerUnitLength() const { return this->Qy; }
    virtual double GetInertiaQzPerUnitLength() const { return this->Qz; }
};

class ChApi ChBeamSectionTaperedTimoshenkoAdvancedGeneric {
  public:
    ChBeamSectionTaperedTimoshenkoAdvancedGeneric()
        : length(1.0),                  // default length of two sections.
          use_lumped_mass_matrix(true)  // lumped mass matrix is used as default
    {
        this->avg_sec_par = std::make_shared<AverageSectionParameters>();
    }

    virtual ~ChBeamSectionTaperedTimoshenkoAdvancedGeneric() {}

    // Set the length of beam element with two sections
    void SetLength(double mv) { length = mv; };
    // Get the length of beam element with two sections
    double GetLength() const { return length; };

    // Set the type of mass matrix:
    // SetLumpedMassMatrixType(true) --> lumped mass matrix, which is default.
    // SetLumpedMassMatrixType(false) --> consistent mass matrix.
    void SetLumpedMassMatrixType(bool mv) { use_lumped_mass_matrix = mv; };
    bool GetLumpedMassMatrixType() const { return use_lumped_mass_matrix; };

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetSectionA(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> my_material) { sectionA = my_material; }
    void SetSectionB(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> my_material) { sectionB = my_material; }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> GetSectionA() { return sectionA; }
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> GetSectionB() { return sectionB; }

    /// Compute the 12x12 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix)
    virtual void ComputeInertiaMatrix(ChMatrixDynamic<>& M  ///< 12x12 sectional mass matrix values here
    );

    /// Get the average damping coefficient of this tapered cross-section.
    virtual DampingCoefficients GetBeamRaleyghDamping() const;

    virtual std::shared_ptr<AverageSectionParameters> GetAverageSectionParameters() const { return this->avg_sec_par; };

  protected:
    double length;

    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> sectionA;
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> sectionB;

    bool use_lumped_mass_matrix;

    virtual void ComputeLumpedInertiaMatrix(ChMatrixNM<double, 12, 12>& M  ///< 12x12 sectional mass matrix values here
    );

    // A quick algorithm to do the transformation of stiffness matrxi in case of mass axis orientation and mass cetner
    // offset, But, has been validated to be wrong.
    virtual void ComputeSimpleConsistentInertiaMatrix(
        ChMatrixNM<double, 12, 12>& M  ///< 12x12 sectional mass matrix values here
    );

    // A very generic implementation for consistent mass matrix, considering mass center offset and axis orientation.
    // The transformation is hard-coded.
    virtual void ComputeConsistentInertiaMatrix(
        ChMatrixNM<double, 12, 12>& M  ///< 12x12 sectional mass matrix values here
    );

    // Some important average section parameters, to calculate once, enable to access them conveniently.
    std::shared_ptr<AverageSectionParameters> avg_sec_par;
    virtual void ComputeAverageSectionParameters();
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
