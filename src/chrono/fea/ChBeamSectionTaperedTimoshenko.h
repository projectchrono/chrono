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

/// @addtogroup fea_utils
/// @{

/// This damping model supports you to assign different Rayleigh damping coefficients for different dimensions,
/// which would be helpful for those anisotropic material, such as wind turbine blade.
/// Note, the square of these four parameters(bx/by/bz/bt) are the beta coefficient of Rayleigh damping model of beam
/// element.
/// alpha is the mass-proportional damping coefficient. Because the mass matrix might be lumped or consistent type,
/// for the sake of simplification, the mass-proportional damping matrix is evaluated simply as Rm+=alpha*M, instead
/// of four different values as stiffness-proportional term.
/// This damping modal is used in ChElementBeamTaperedTimoshenko and ChElementBeamTaperedTimoshenkoFPM.
/// For more background theory, please refer to:
/// [1]. Hansen, M. H. (2001). Anisotropic damping of Timoshenko beam elements. (Denmark. Forskningscenter Risoe.
///      Risoe-R; No. 1267(EN)).
struct DampingCoefficients {
    double bx;           ///< damping coefficient along x axis (axial)
    double by;           ///< damping coefficient along y axis (shear) and about z axis (bending)
    double bz;           ///< damping coefficient along z axis (shear) and about y axis (bending)
    double bt;           ///< damping coefficient about x axis (torsion)
    double alpha = 0.0;  ///< mass-proportional damping coefficient, be zero as default
};

/// The aeverage section properties of tapered section could be stored in this struct.
struct AverageSectionParameters {
    double mu;     ///< mass per unit length
    double alpha;  ///< section rotation about elastic center[rad]
    double Cy;     ///< elastic center y displacement respect to centerline
    double Cz;     ///< elastic center z displacement respect to centerline
    double Sy;     ///< shear center y displacement respect to centerline
    double Sz;     ///< shear center z displacement respect to centerline
    double My;     ///< mass center y displacement respect to centerline
    double Mz;     ///< mass center z displacement respect to centerline

    double Jyy;  ///< inertia Jyy per unit lenght, in centerline reference, measured along centerline main axes
    double Jzz;  ///< inertia Jzz per unit lenght, in centerline reference, measured along centerline main axes
    double Jyz;  ///< inertia Jyz per unit lenght, in centerline reference, measured along centerline main axes
    double Jxx;  ///< inertia Jxx per unit lenght, in centerline reference, measured along centerline main axes
    double Qy;   ///< mass moment of area along yy, in centerline reference, measured along centerline main axes
    double Qz;   ///< mass moment of area along zz, in centerline reference, measured along centerline main axes

    double mass_phi;  ///< rotation angle of mass principal axis about elastic center[rad]
    double Jmyy;      ///< inertia Jyy per unit lenght, in mass center reference, measured along mass principal axes
    double Jmzz;      ///< inertia Jzz per unit lenght, in mass center reference, measured along mass principal axes
    double Jmyz;      ///< inertia Jyz per unit lenght, in mass center reference, measured along mass principal axes
    double Jmxx;      ///< inertia Jxx per unit lenght, in mass center reference, measured along mass principal axes
    double Qmy;  ///< mass moment of area along yy, in mass center reference, measured along mass principal axes, which
                 ///< is exactly zero in theory.
    double Qmz;  ///< mass moment of area along zz, in mass center reference, measured along mass principal axes, which
                 ///< is exactly zero in theory.

    double EA;    ///< axial rigidity
    double GJ;    ///< torsion rigidity
    double GAyy;  ///< shear regidity along yy, evaluated in the elastic center and elastic principal axis. GAyy=ky*G*A
    double GAzz;  ///< shear regidity along zz, evaluated in the elastic center and elastic principal axis. GAzz=kz*G*A
    double EIyy;  ///< bending regidity about yy, evaluated in the elastic center and elastic principal axis
    double EIzz;  ///< bending rigidity about zz, evaluated in the elastic center and elastic principal axis

    double GAmyy;  ///< shear regidity along yy, evaluated in the mass center and mass principal axis
    double GAmzz;  ///< shear regidity along zz, evaluated in the mass center and mass principal axis
    double EImyy;  ///< bending regidity about yy, evaluated in the mass center and mass principal axis
    double EImzz;  ///< bending rigidity about zz, evaluated in the mass center and mass principal axis

    double phimy;  ///< shear-deformation parameter, phimy = 12*EImzz/(GAmyy*L*L)
    double phimz;  ///< shear-deformation parameter, phimz = 12*EImyy/(GAmzz*L*L)
    double phiy;   ///< shear-deformation parameter, phiy = 12*EIzz/(GAyy*L*L)
    double phiz;   ///< shear-deformation parameter, phiz = 12*EIyy/(GAzz*L*L)

    DampingCoefficients rdamping_coeff;  ///< damping parameters

    /// An artificial factor to modify the damping coefficient in the shear deformation,
    /// which may be helpful to improve the numerical stability of Timoshenko beam.
    /// But you need to take care of the final structural damping ratio of your model after assigning this factor.
    double artificial_factor_for_shear_damping;
};

/// Base class for all constitutive models of sections of Timoshenko beams.
/// To be used with ChElementBeamTaperedTimoshenko.
class ChApi ChBeamSectionTimoshenkoAdvancedGeneric : public ChBeamSectionRayleighAdvancedGeneric {
  protected:
    double GAyy;  ///< shear regidity along yy, evaluated in the elastic center and elastic principal axis. GAyy=ky*G*A
    double GAzz;  ///< shear regidity along zz, evaluated in the elastic center and elastic principal axis. GAzz=kz*G*A

    ///   Mass moment of area, defined as :
    /// \f$ Q_{y} =  \int_\Omega \rho z d\Omega \f$,
    /// \f$ Q_{z} =  \int_\Omega \rho y d\Omega \f$,
    double Qy;  ///< mass moment of area along yy, in centerline reference, measured along centerline main axes
    double Qz;  ///< mass moment of area along zz, in centerline reference, measured along centerline main axes

    /// Damping parameters.
    /// You could assign different Rayleigh damping coefficients for different dimensions: axial, bending, torsion.
    DampingCoefficients rdamping_coeff;

    /// An artificial correction factor for the structural damping model in the shear deformation,
    /// as default it's 1.0 and don't have any correction effect.
    /// But when the numerical divergence occured, you could try to set this value be larger than 1.0
    /// to improve the numerical stability in long-time simulation.
    double artificial_factor_for_shear_damping = 1.0;

  public:
    ChBeamSectionTimoshenkoAdvancedGeneric() : GAyy(0), GAzz(0), Qy(0), Qz(0) {
        double bcoeff = 0.001;  // default damping coefficient for stiffness-proportional term
        rdamping_coeff.bx = bcoeff;
        rdamping_coeff.by = bcoeff;
        rdamping_coeff.bz = bcoeff;
        rdamping_coeff.bt = bcoeff;
        rdamping_coeff.alpha = 0.0;  // default alpha damping coefficient for mass-proportional term
    }

    ChBeamSectionTimoshenkoAdvancedGeneric(
        const double mAx,                          ///< axial rigidity
        const double mTxx,                         ///< torsion rigidity
        const double mByy,                         ///< bending regidity about yy
        const double mBzz,                         ///< bending rigidity about zz
        const double mGAyy,                        ///< shear rigidity along yy
        const double mGAzz,                        ///< shear rigidity along zz
        const DampingCoefficients mdamping_coeff,  ///< damping coefficients
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

    /// Sets the damping parameters of section. You have a chance to assign different coefficients for axial, bending
    /// and torsion directions. This would be helpful for those anisotropic material, such as wind turbine blade.
    virtual void SetBeamRaleyghDamping(DampingCoefficients mdamping_coeff) { this->rdamping_coeff = mdamping_coeff; }

    ///  Gets the damping parameters of section.
    virtual DampingCoefficients GetBeamRaleyghDamping() const { return this->rdamping_coeff; }

    /// Set the Jyy Jzz Jyz components of the sectional inertia per unit length,
    /// in centerline reference, measured along centerline main axes.
    /// These are defined as:
    /// \f$ J_{yy} =  \int_\Omega \rho z^2 d\Omega \f$, also Jyy = Mm(4,4)
    /// \f$ J_{zz} =  \int_\Omega \rho y^2 d\Omega \f$, also Jzz = Mm(5,5)
    /// \f$ J_{yz} =  \int_\Omega \rho y z  d\Omega \f$, also Jyz = -Mm(4,5) = -Mm(5,4)
    /// It is not needed to enter also Jxx because Jxx=(Jzz+Jyy) by the polar theorem.
    virtual void SetInertiasPerUnitLength(const double mJyy,
                                          const double mJzz,
                                          const double mJyz,
                                          const double mQy,
                                          const double mQz);

    /// Set inertia moments, per unit length, as assumed computed in the Ym Zm "mass reference"
    /// frame, ie. centered at the center of mass and rotated by phi angle to match the main axes of inertia:
    /// \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$,
    /// \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega \f$.
    /// Assuming the center of mass is already set.
    virtual void SetMainInertiasInMassReference(const double Jmyy,
                                                const double Jmzz,
                                                const double Jmyz,
                                                const double mass_phi,
                                                const double Qmy,
                                                const double Qmz);

    /// Get inertia moments, per unit length, as assumed computed in the Ym Zm "mass reference" frame, and the rotation
    /// phi of that frame,
    /// ie. inertias centered at the center of mass and rotated by phi angle to match the main axes of inertia:
    /// \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$,
    /// \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega \f$.
    /// Assuming the center of mass is already set.
    virtual void GetMainInertiasInMassReference(double& Jmyy,
                                                double& Jmzz,
                                                double& Jmyz,
                                                double& mass_phi,
                                                double& Qmy,
                                                double& Qmz);
    /// Get inertia moment per unit length Jxx_massref, as assumed computed in the "mass reference"
    /// frame, ie. centered at the center of mass
    /// NOTE: To be safe, it is recommended to  use GetMainInertiasInMassReference() instead,
    /// because Qmy,Qmz are ignored, although they are exactly zero in theory.
    virtual double GetInertiaJxxPerUnitLengthInMassReference() const {
        GetLog() << "Warm warning: it is recommended to use GetMainInertiasInMassReference() instead, "
                 << "and do calculation: Jmxx = Jmyy+Jmzz \n";
        return this->Jxx - this->mu * this->Mz * this->Mz - this->mu * this->My * this->My;
    };

    /// Get the Jyy component of the sectional inertia per unit length,
    /// in centerline reference, measured along centerline main axes.
    virtual double GetInertiaJyyPerUnitLength() const { return this->Jyy; }

    /// Get the Jzz component of the sectional inertia per unit length,
    /// in centerline reference, measured along centerline main axes.
    virtual double GetInertiaJzzPerUnitLength() const { return this->Jzz; }

    /// Get the Jyz component of the sectional inertia per unit length,
    /// in centerline reference, measured along centerline main axes.
    virtual double GetInertiaJyzPerUnitLength() const { return this->Jyz; }

    /// Get the Qy component of mass moment of area per unit length,
    /// in centerline reference, measured along centerline main axes.
    virtual double GetInertiaQyPerUnitLength() const { return this->Qy; }

    /// Get the Qz component of mass moment of area per unit length,
    /// in centerline reference, measured along centerline main axes.
    virtual double GetInertiaQzPerUnitLength() const { return this->Qz; }

    /// Set the artificial correction factor for the structural damping model in the shear deformation,
    /// as default it's 1.0 and don't have any correction effect.
    /// But when the numerical divergence occured, you could try to set this value be larger than 1.0
    /// to improve the numerical stability in long-time simulation.
    virtual void SetArtificialFactorForShearDamping(double mv) { this->artificial_factor_for_shear_damping = mv; };

    /// Get the artificial correction factor for the structural damping model in the shear deformation.
    virtual double GetArtificialFactorForShearDamping() const { return this->artificial_factor_for_shear_damping; };
};

/// Base class for all constitutive models of sections of Tapered Timoshenko beams.
/// This tapered section consists of two objects of ChBeamSectionTimoshenkoAdvancedGeneric.
/// To be used with ChElementBeamTaperedTimoshenko.
class ChApi ChBeamSectionTaperedTimoshenkoAdvancedGeneric {
  public:
    ChBeamSectionTaperedTimoshenkoAdvancedGeneric()
        : length(1.0),                  // default length of two sections.
          use_lumped_mass_matrix(true)  // lumped mass matrix is used as default
    {
        this->avg_sec_par = std::make_shared<AverageSectionParameters>();
    }

    virtual ~ChBeamSectionTaperedTimoshenkoAdvancedGeneric() {}

    /// Set the length of beam element with two sections
    void SetLength(double mv) { length = mv; };
    /// Get the length of beam element with two sections
    double GetLength() const { return length; };

    /// Set the type of mass matrix:
    /// - true: lumped mass matrix, which is default.
    /// - false: consistent mass matrix.
    void SetLumpedMassMatrixType(bool mv) { use_lumped_mass_matrix = mv; };
    /// Get the type of mass matrix：
    /// - true: lumped mass matrix.
    /// - false: consistent mass matrix.
    bool GetLumpedMassMatrixType() const { return use_lumped_mass_matrix; };

    /// Set the section & material of beam element at end A.
    void SetSectionA(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> my_material) { sectionA = my_material; }
    /// Set the section & material of beam element at end B.
    void SetSectionB(std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> my_material) { sectionB = my_material; }
    /// Get the section & material of the element at end A.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> GetSectionA() { return sectionA; }
    /// Get the section & material of the element at end B.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> GetSectionB() { return sectionB; }

    /// Compute the 12x12 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix)
    virtual void ComputeInertiaMatrix(ChMatrixDynamic<>& M  ///< 12x12 sectional mass matrix values here
    );

    /// Compute the 12x12 local inertial-damping (gyroscopic damping) matrix.
    /// The lumped format is used, need to multiple 0.5 * length to obtain the final inertial-damping matrix
    virtual void ComputeInertiaDampingMatrix(
        ChMatrixNM<double, 12, 12>& Ri,  ///< 12x12 sectional inertial-damping matrix values here
        const ChVector<>& mW_A,          ///< current angular velocity of section of node A, in material frame
        const ChVector<>& mW_B           ///< current angular velocity of section of node B, in material frame
    );

    /// Compute the 12x12 local inertial-stiffness matrix.
    /// The lumped format is used, need to multiple 0.5 * length to obtain the final inertial-stiffness matrix
    virtual void ComputeInertiaStiffnessMatrix(
        ChMatrixNM<double, 12, 12>& Ki,  ///< 12x12 sectional inertial-stiffness matrix values here
        const ChVector<>& mWvel_A,       ///< current angular velocity of section of node A, in material frame
        const ChVector<>& mWacc_A,       ///< current angular acceleration of section of node A, in material frame
        const ChVector<>& mXacc_A,       ///< current acceleration of section of node A, in material frame)
        const ChVector<>& mWvel_B,       ///< current angular velocity of section of node B, in material frame
        const ChVector<>& mWacc_B,       ///< current angular acceleration of section of node B, in material frame
        const ChVector<>& mXacc_B        ///< current acceleration of section of node B, in material frame
    );

    /// Get the average damping parameters of this tapered cross-section.
    virtual DampingCoefficients GetBeamRaleyghDamping() const;

    /// Compute the average section parameters: mass, inertia and rigidity, etc.
    virtual void ComputeAverageSectionParameters();

    /// Get the average sectional parameters(mass, inertia and rigidity, etc.) of this tapered cross-section.
    virtual std::shared_ptr<AverageSectionParameters> GetAverageSectionParameters() const { return this->avg_sec_par; };

    // Optimization flags

    /// Flag that turns on/off the computation of the [Ri] 'gyroscopic' inertial damping matrix.
    /// If false, Ri=0. Can be used for cpu speedup, profiling, tests. Default: true.
    bool compute_inertia_damping_matrix = true;

    /// Flag that turns on/off the computation of the [Ki] inertial stiffness matrix.
    /// If false, Ki=0. Can be used for cpu speedup, profiling, tests. Default: true.
    bool compute_inertia_stiffness_matrix = true;

    /// Flag for computing the Ri and Ki matrices via numerical differentiation even if
    /// an analytical expression is provided. Children calsses must take care of this. Default: false.
    bool compute_Ri_Ki_by_num_diff = false;

  protected:
    /// The length of beam element with two sections.
    double length;

    /// The section & material of beam element at end A.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> sectionA;
    /// The section & material of beam element at end B.
    std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric> sectionB;

    /// The type of mass matrix:
    /// - true: lumped mass matrix, which is default.
    /// - false: consistent mass matrix.
    bool use_lumped_mass_matrix;

    // Some important average section parameters, to calculate only once, enable to access them conveniently.
    std::shared_ptr<AverageSectionParameters> avg_sec_par;

    // A lock to avoid computing avg_sec_par several times, initialized as false by default.
    bool compute_ave_sec_par = false;

    /// Compute the 12x12 sectional inertia matrix in lumped format, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix)
    virtual void ComputeLumpedInertiaMatrix(ChMatrixNM<double, 12, 12>& M  ///< 12x12 sectional mass matrix values here
    );

    // A quick algorithm to derive the mass matrix via a simple transformation in case of mass axis orientation and mass
    // cetner offset. But, it has been validated to be WRONG. It is kept here only to remind you not to try this simple
    // method again.
    virtual void ComputeSimpleConsistentInertiaMatrix(
        ChMatrixNM<double, 12, 12>& M  ///< 12x12 sectional mass matrix values here
    );

    /// A very generic implementation for consistent mass matrix, considering mass center offset and axis orientation.
    /// The transformation of mass center offset and mass principal axis orientation is hard-coded.
    virtual void ComputeConsistentInertiaMatrix(
        ChMatrixNM<double, 12, 12>& M  ///< 12x12 sectional mass matrix values here
    );

  public:
    // If fixed-size matrix of EIGEN3 library is used, we need to add this macro
    // to reduce the possibility of alignment problems
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
