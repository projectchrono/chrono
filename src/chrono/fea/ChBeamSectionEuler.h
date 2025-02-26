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

#ifndef CHBEAMSECTIONEULER_H
#define CHBEAMSECTIONEULER_H

#include "chrono/fea/ChBeamSection.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Base class for all constitutive models of sections of Euler beams.
/// To be used with ChElementBeamEuler.
/// For practical purposes, either you use the concrete inherited classes like ChBeamSectionEulerSimple,
/// ChBeamSectionEulerAdvanced etc., or you inherit your class from this.

class ChApi ChBeamSectionEuler : public ChBeamSection {
  public:
    ChBeamSectionEuler()
        : rdamping_beta(0.01),      // default Rayleigh beta damping.
          rdamping_alpha(0),        // default Rayleigh alpha damping.
          JzzJyy_factor(1. / 500.)  // default tiny rotational inertia of section on Y and Z to avoid singular mass
    {}
    virtual ~ChBeamSectionEuler() {}

    // STIFFNESS INTERFACE

    /// Gets the axial rigidity, usually A*E.
    virtual double GetAxialRigidity() const = 0;

    /// Gets the torsion rigidity, for torsion about X axis at elastic center, usually J*G.
    virtual double GetTorsionRigidityX() const = 0;

    /// Gets the bending rigidity, for bending about Y axis at elastic center, usually Iyy*E.
    virtual double GetBendingRigidityY() const = 0;

    /// Gets the bending rigidity, for bending about Z axis at elastic center, usually Izz*E.
    virtual double GetBendingRigidityZ() const = 0;

    /// Set the rotation of the Y Z section axes for which the YbendingRigidity and ZbendingRigidity are defined.
    virtual double GetSectionRotation() const = 0;

    /// Gets the Y position of the elastic center respect to centerline.
    virtual double GetCentroidY() const = 0;
    /// Gets the Z position of the elastic center respect to centerline.
    virtual double GetCentroidZ() const = 0;

    /// Gets the Y position of the shear center respect to centerline.
    virtual double GetShearCenterY() const = 0;
    /// Gets the Z position of the shear center respect to centerline.
    virtual double GetShearCenterZ() const = 0;

    // MASS INTERFACE

    /// Get mass per unit length, ex.SI units [kg/m]
    virtual double GetMassPerUnitLength() const = 0;

    /// Get the Jxx component of the inertia per unit length (polar inertia) in the Y Z unrotated reference
    /// frame of the section at centerline. Note: it automatically follows Jxx=Jyy+Jzz for the polar theorem. Also,
    /// Jxx=density*Ixx if constant density.
    virtual double GetInertiaJxxPerUnitLength() const = 0;

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix)
    virtual void ComputeInertiaMatrix(ChMatrix66d& M  ///< 6x6 sectional mass matrix values here
                                      ) = 0;

    /// Compute the 6x6 sectional inertia damping matrix [Ri] (gyroscopic matrix damping), as in linearization
    ///  dFi=[Mi]*d{xacc,wacc}+[Ri]*d{xvel,wvel}+[Ki]*d{pos,rot}
    /// The matrix is computed in the material reference, i.e. both linear and rotational coords assumed in the basis of
    /// the centerline reference. Default implementation: falls back to numerical differentiation of
    /// ComputeInertialForce to compute Ri, please override this if analytical formula of Ri is known!
    virtual void ComputeInertiaDampingMatrix(
        ChMatrix66d& Ri,      ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
        const ChVector3d& mW  ///< current angular velocity of section, in material frame
    );

    /// Compute the 6x6 sectional inertia stiffness matrix [Ki^], as in linearization
    ///  dFi=[Mi]*d{xacc,wacc}+[Ri]*d{xvel,wvel}+[Ki]*d{pos,rot}
    /// The matrix is computed in the material reference.
    /// NOTE the matrix already contains the 'geometric' stiffness, so it transforms to absolute transl/local rot just
    /// like [Mi] and [Ri]:
    ///  [Ki]_al =[R,0;0,I]*[Ki^]*[R',0;0,I']  , with [Ki^]=([Ki]+[0,f~';0,0])  for f=current force part of inertial
    ///  forces.
    /// Default implementation: falls back to numerical differentiation of ComputeInertialForce to compute Ki^,
    /// please override this if analytical formula of Ki^ is known!
    virtual void ComputeInertiaStiffnessMatrix(
        ChMatrix66d& Ki,          ///< 6x6 sectional inertial-stiffness matrix [Ki^] values here
        const ChVector3d& mWvel,  ///< current angular velocity of section, in material frame
        const ChVector3d& mWacc,  ///< current angular acceleration of section, in material frame
        const ChVector3d& mXacc   ///< current acceleration of section, in material frame (not absolute!)
    );

    /// Compute the values of inertial force & torque depending on quadratic velocity terms,
    /// that is the gyroscopic torque (null for Euler beam as point-like mass section, might be nonzero if adding
    /// Rayleigh beam theory) and the centrifugal term (if any). All terms expressed in the material reference, ie. the
    /// reference in the centerline of the section.
    virtual void ComputeQuadraticTerms(ChVector3d& mF,       ///< centrifugal term (if any) returned here
                                       ChVector3d& mT,       ///< gyroscopic term returned here
                                       const ChVector3d& mW  ///< current angular velocity of section, in material frame
                                       ) = 0;

    /// Compute the total inertial forces (per unit length). This default implementation falls back to  Fi =
    /// [Mi]*{xacc,wacc}+{mF,mT} where [Mi] is given by ComputeInertiaMatrix() and {F_quad,T_quad} are given by
    /// ComputeQuadraticTerms(), i.e. gyro and centrif.terms. Note: both force and torque are returned in the basis of
    /// the material frame (not the absolute frame!), ex. to apply it to a Chrono body, the force must be rotated to
    /// absolute basis. For faster implementations one can override this, ex. avoid doing the [Mi] matrix product.
    virtual void ComputeInertialForce(
        ChVector3d& mFi,          ///< total inertial force returned here, in basis of material frame
        ChVector3d& mTi,          ///< total inertial torque returned here, in basis of material frame
        const ChVector3d& mWvel,  ///< current angular velocity of section, in material frame
        const ChVector3d& mWacc,  ///< current angular acceleration of section, in material frame
        const ChVector3d& mXacc   ///< current acceleration of section, in material frame (not absolute!)
    );

    /// The Euler beam model has no rotational inertia per each section, assuming mass is concentrated on
    /// the centerline. However this creates a singular mass matrix, that might end in problems when doing modal
    /// analysis etc. A solution is to force Jyy and Jzz inertials per unit lengths to be a percent of the mass per unit
    /// length. By default it is 1/500. Use this function to set such factor. You can also turn it to zero. Note that
    /// the effect becomes negligible anyway for finer meshing.
    void SetArtificialJyyJzzFactor(double mf) { JzzJyy_factor = mf; }
    double GetArtificialJyyJzzFactor() { return JzzJyy_factor; }

    // DAMPING INTERFACE

    /// Set the "alpha" Rayleigh damping ratio,
    /// the mass-proportional structural damping in: R = alpha*M + beta*K
    virtual void SetRayleighDampingAlpha(double malpha) { this->rdamping_alpha = malpha; }
    double GetRayleighDampingAlpha() { return this->rdamping_alpha; }

    /// Set the "beta" Rayleigh damping ratio,
    /// the stiffness-proportional structural damping in: R = alpha*M + beta*K
    virtual void SetRayleighDampingBeta(double mbeta) { this->rdamping_beta = mbeta; }
    double GetRayleighDampingBeta() { return this->rdamping_beta; }

    /// Set both beta and alpha coefficients in Rayleigh damping model:  R = alpha*M + beta*K.
    /// For backward compatibility, if one provides only the first parameter, this would be the "beta"
    /// stiffness-proportional term, and the "alpha" mass proportional term would be left to default zero.
    virtual void SetRayleighDamping(double mbeta, double malpha = 0) {
        this->rdamping_beta = mbeta;
        this->rdamping_alpha = malpha;
    }

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
    double rdamping_beta;
    double rdamping_alpha;
    double JzzJyy_factor;
};

/// Basic section of an Euler-Bernoulli beam in 3D, for a homogeneous density
/// and homogeneous elasticity, given basic material properties (Izz and Iyy moments of inertia,
/// area, Young modulus, etc.).
/// This is a simple section model that assumes the elastic center, the shear center and the mass
/// center to be all in the centerline of the beam (section origin); this is the case of symmetric sections for example.
/// To be used with ChElementBeamEuler.
/// This material can be shared between multiple beams.
///
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratSimple.png"
///
class ChApi ChBeamSectionEulerSimple : public ChBeamSectionEuler {
  public:
    double Area;
    double Iyy;
    double Izz;
    double J;
    double G;
    double E;
    double density;
    double Ks_y;
    double Ks_z;

    ChBeamSectionEulerSimple()
        : E(0.01e9),     // default E stiffness: (almost rubber)
          density(1000)  // default density: water
    {
        SetShearModulusFromPoisson(0.3);      // default G (low poisson ratio)
        SetAsRectangularSection(0.01, 0.01);  // defaults Area, Ixx, Iyy, Ks_y, Ks_z, J
    }

    virtual ~ChBeamSectionEulerSimple() {}

    /// Set the cross sectional area A of the beam (m^2)
    void SetArea(const double ma) { this->Area = ma; }
    double GetArea() const { return this->Area; }

    /// Set the Iyy moment of inertia of the beam (for flexion about y axis)
    /// Note: some textbook calls this Iyy as Iz
    void SetIyy(double ma) { this->Iyy = ma; }
    double GetIyy() const { return this->Iyy; }

    /// Set the Izz moment of inertia of the beam (for flexion about z axis)
    /// Note: some textbook calls this Izz as Iy
    void SetIzz(double ma) { this->Izz = ma; }
    double GetIzz() const { return this->Izz; }

    /// Set the J torsion constant of the beam (for torsion about x axis)
    void SetJ(double ma) { this->J = ma; }
    double GetJ() const { return this->J; }

    /// Set the Timoshenko shear coefficient Ks for y shear, usually about 0.8,
    /// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
    void SetKsy(double ma) { this->Ks_y = ma; }
    double GetKsy() const { return this->Ks_y; }

    /// Set the Timoshenko shear coefficient Ks for z shear, usually about 0.8,
    /// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
    void SetKsz(double ma) { this->Ks_z = ma; }
    double GetKsz() const { return this->Ks_z; }

    /// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
    /// at once, given the y and z widths of the beam assumed
    /// with rectangular shape.
    void SetAsRectangularSection(double width_y, double width_z) {
        this->Area = width_y * width_z;
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

        this->SetDrawThickness(width_y, width_z);
    }

    /// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
    /// at once, given the diameter of the beam assumed
    /// with circular shape.
    void SetAsCircularSection(double diameter) {
        this->Area = CH_PI * std::pow((0.5 * diameter), 2);
        this->Izz = (CH_PI / 4.0) * std::pow((0.5 * diameter), 4);
        this->Iyy = Izz;

        // exact expression for circular beam J = Ixx ,
        // where for polar theorem Ixx = Izz+Iyy
        this->J = Izz + Iyy;

        // set Ks using Timoshenko-Gere formula for solid circular shape
        double poisson = this->E / (2.0 * this->G) - 1.0;
        this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
        this->Ks_z = this->Ks_y;

        this->SetDrawCircularRadius(diameter / 2);
    }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set the shear modulus, used for computing the torsion rigidity = J*G
    void SetShearModulus(double mG) { this->G = mG; }
    double GetShearModulus() const { return this->G; }

    /// Set the shear modulus, given current Young modulus and the specified Poisson ratio.
    void SetShearModulusFromPoisson(double mpoisson) { this->G = this->E / (2.0 * (1.0 + mpoisson)); }

    // INTERFACES

    /// Gets the axial rigidity, usually A*E.
    virtual double GetAxialRigidity() const override { return this->Area * this->E; }

    /// Gets the torsion rigidity, for torsion about X axis at elastic center, usually J*G.
    virtual double GetTorsionRigidityX() const override { return this->J * this->G; }

    /// Gets the bending rigidity, for bending about Y axis at elastic center, usually Iyy*E.
    virtual double GetBendingRigidityY() const override { return this->Iyy * this->E; }

    /// Gets the bending rigidity, for bending about Z axis at elastic center, usually Izz*E.
    virtual double GetBendingRigidityZ() const override { return this->Izz * this->E; }

    /// Set the rotation of the Y Z section axes for which the YbendingRigidity and ZbendingRigidity are defined.
    virtual double GetSectionRotation() const override { return 0; }

    /// Gets the Y position of the elastic center respect to centerline.
    virtual double GetCentroidY() const override { return 0; }
    /// Gets the Z position of the elastic center respect to centerline.
    virtual double GetCentroidZ() const override { return 0; }

    /// Gets the Y position of the shear center respect to centerline.
    virtual double GetShearCenterY() const override { return 0; }
    /// Gets the Z position of the shear center respect to centerline.
    virtual double GetShearCenterZ() const override { return 0; }

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    virtual void ComputeInertiaMatrix(ChMatrix66d& M) override;

    /// Compute the 6x6 sectional inertia damping matrix [Ri] (gyroscopic matrix damping)
    virtual void ComputeInertiaDampingMatrix(
        ChMatrix66d& Ri,      ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
        const ChVector3d& mW  ///< current angular velocity of section, in material frame
        ) override;

    /// Compute the 6x6 sectional inertia stiffness matrix [Ki^]
    virtual void ComputeInertiaStiffnessMatrix(
        ChMatrix66d& Ki,          ///< 6x6 sectional inertial-stiffness matrix [Ki^] values here
        const ChVector3d& mWvel,  ///< current angular velocity of section, in material frame
        const ChVector3d& mWacc,  ///< current angular acceleration of section, in material frame
        const ChVector3d& mXacc   ///< current acceleration of section, in material frame (not absolute!)
        ) override;

    /// Compute the centrifugal term and gyroscopic term
    virtual void ComputeQuadraticTerms(ChVector3d& mF, ChVector3d& mT, const ChVector3d& mW) override;

    /// Get mass per unit length, ex.SI units [kg/m]
    virtual double GetMassPerUnitLength() const override { return this->Area * this->density; }

    /// Get the Jxx component of the inertia per unit length (polar inertia) in the Y Z unrotated reference
    /// frame of the section at centerline. Note: it automatically follows Jxx=Jyy+Jzz for the polar theorem. Also,
    /// Jxx=density*Ixx if constant density.
    virtual double GetInertiaJxxPerUnitLength() const override { return (this->Iyy + this->Izz) * this->density; }
};

// for backward compatibility - note it WILL BE DEPRECATED
using ChBeamSectionBasic = ChBeamSectionEulerSimple;

/// Advanced section of an Euler-Bernoulli beam in 3D, for a homogeneous density
/// and homogeneous elasticity, given basic material properties (Izz and Iyy moments of inertia,
/// area, Young modulus, etc.), but also supporting the advanced case of
/// Iyy and Izz axes rotated respect reference, elastic center with offset
/// from centerline reference, and shear center with offset from centerline reference.
/// To be used with ChElementBeamEuler.
/// This material can be shared between multiple beams.
///
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElementBeamEuler_section.png"
///
class ChApi ChBeamSectionEulerAdvanced : public ChBeamSectionEulerSimple {
  public:
    double alpha;  // Rotation of Izz Iyy respect to reference line x
    double Cy;     // Centroid (elastic center, tension center)
    double Cz;
    double Sy;  // Shear center
    double Sz;

    ChBeamSectionEulerAdvanced() : alpha(0), Cy(0), Cz(0), Sy(0), Sz(0) {}

    virtual ~ChBeamSectionEulerAdvanced() {}

    /// Set the rotation in [rad], about elastic center, of the Y Z axes for which the
    /// Iyy and Izz are computed.
    void SetSectionRotation(double ma) { this->alpha = ma; }

    /// Set the displacement of the centroid C (i.e. the elastic center,
    /// or tension center) with respect to the reference beam line.
    void SetCentroid(double my, double mz) {
        this->Cy = my;
        this->Cz = mz;
    }

    /// Set the displacement of the shear center S with respect to the reference beam line.
    /// For shapes like rectangles, rotated rectangles, etc., it corresponds to the centroid C,
    /// but for "L" shaped or "U" shaped beams this is not always true, and the shear center
    /// accounts for torsion effects when a shear force is applied.
    void SetShearCenter(double my, double mz) {
        this->Sy = my;
        this->Sz = mz;
    }

    // INTERFACES

    virtual double GetSectionRotation() const override { return this->alpha; }

    virtual double GetCentroidY() const override { return this->Cy; }
    virtual double GetCentroidZ() const override { return this->Cz; }

    virtual double GetShearCenterY() const override { return this->Sy; }
    virtual double GetShearCenterZ() const override { return this->Sz; }
};

// for backward compatibility - note it WILL BE DEPRECATED
using ChBeamSectionAdvanced = ChBeamSectionEulerAdvanced;

/// General purpose section of an Euler-Bernoulli beam in 3D, not assuming homogeneous density
/// or homogeneous elasticity, given basic material properties. This is the case where
/// one uses a FEA preprocessor to compute the rigidity of a complex beam made with multi-layered
/// reinforcements with different elasticity and different density - in such a case you could not
/// use ChBeamSectionEulerAdvanced because you do not have a single E or single density, but you rather
/// have collective values of bending rigidities, and collective mass per unit length. This class
/// allows using these values directly, bypassing any knowledge of area, density, Izz Iyy, E young modulus, etc.
/// To be used with ChElementBeamEuler.
/// The center of mass of the section can have an offset respect to the centerline.
/// This material can be shared between multiple beams.
///
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElementBeamEuler_section.png"
///

class ChApi ChBeamSectionEulerAdvancedGeneric : public ChBeamSectionEuler {
  protected:
    double Ax;     // axial rigidity
    double Txx;    // torsion rigidity
    double Byy;    // bending about yy rigidity
    double Bzz;    // bending about zz rigidity
    double alpha;  // section rotation about elastic center
    double Cy;     // Centroid (elastic center, tension center)
    double Cz;
    double Sy;  // Shear center
    double Sz;
    double mu;   // mass per unit length
    double Jxx;  // inertia per unit length
    double My;   // Mass center
    double Mz;

  public:
    ChBeamSectionEulerAdvancedGeneric()
        : Ax(1), Txx(1), Byy(1), Bzz(1), alpha(0), Cy(0), Cz(0), Sy(0), Sz(0), mu(1000), Jxx(1), My(0), Mz(0) {}

    ChBeamSectionEulerAdvancedGeneric(
        const double mAx,      ///< axial rigidity
        const double mTxx,     ///< torsion rigidity
        const double mByy,     ///< bending regidity about yy
        const double mBzz,     ///< bending rigidity about zz
        const double malpha,   ///< section rotation about elastic center [rad]
        const double mCy,      ///< elastic center y displacement respect to centerline
        const double mCz,      ///< elastic center z displacement respect to centerline
        const double mSy,      ///< shear center y displacement respect to centerline
        const double mSz,      ///< shear center z displacement respect to centerline
        const double mmu,      ///< mass per unit length
        const double mJxx,     ///< polar inertia Jxx per unit lenght, measured respect to centerline
        const double mMy = 0,  ///< mass center y displacement respect to centerline
        const double mMz = 0   ///< mass center z displacement respect to centerline
        )
        : Ax(mAx),
          Txx(mTxx),
          Byy(mByy),
          Bzz(mBzz),
          alpha(malpha),
          Cy(mCy),
          Cz(mCz),
          Sy(mSy),
          Sz(mSz),
          mu(mmu),
          Jxx(mJxx),
          My(mMy),
          Mz(mMz) {}

    virtual ~ChBeamSectionEulerAdvancedGeneric() {}

    /// Sets the axial rigidity, usually A*E for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetAxialRigidity(const double mv) { Ax = mv; }

    /// Sets the torsion rigidity, for torsion about X axis, at elastic center,
    /// usually J*G for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetTorsionRigidityX(const double mv) { Txx = mv; }

    /// Sets the bending rigidity, for bending about Y axis, at elastic center,
    /// usually Iyy*E for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetBendingRigidityY(const double mv) { Byy = mv; }

    /// Sets the bending rigidity, for bending about Z axis, at elastic center,
    /// usually Izz*E for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetBendingRigidityZ(const double mv) { Bzz = mv; }

    /// Set the rotation in [rad], abour elastic center, of the Y Z axes for which the
    /// YbendingRigidity and ZbendingRigidity values are defined.
    virtual void SetSectionRotation(const double mv) { alpha = mv; }

    /// Sets the Y position of the elastic center respect to centerline.
    virtual void SetCentroidY(const double mv) { Cy = mv; }
    /// Sets the Z position of the elastic center respect to centerline.
    virtual void SetCentroidZ(const double mv) { Cz = mv; }

    /// Sets the Y position of the shear center respect to centerline.
    virtual void SetShearCenterY(const double mv) { Sy = mv; }
    /// Sets the Z position of the shear center respect to centerline.
    virtual void SetShearCenterZ(const double mv) { Sz = mv; }

    /// Set mass per unit length, ex.SI units [kg/m]
    /// For uniform density it would be A*density, but for nonuniform density
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetMassPerUnitLength(const double mv) { mu = mv; }

    /// Set the Jxx component of the inertia per unit length (polar inertia), computed at centerline.
    /// For uniform density it would be Ixx*density or, by polar theorem, (Izz+Iyy)*density, but for
    /// nonuniform density here you can put a value ad-hoc from a preprocessor
    virtual void SetInertiaJxxPerUnitLength(const double mv) { Jxx = mv; }

    /// Set inertia moment per unit length Jxx_massref, as assumed computed in the "mass reference"
    /// frame, ie. centered at the center of mass. Call this after you set SetCenterOfMass() and SetMassPerUnitLength()
    virtual void SetInertiaJxxPerUnitLengthInMassReference(const double mv) {
        Jxx = mv + this->mu * this->Mz * this->Mz + this->mu * this->My * this->My;
    }

    /// Get inertia moment per unit length Jxx_massref, as assumed computed in the "mass reference"
    /// frame, ie. centered at the center of mass
    virtual double GetInertiaJxxPerUnitLengthInMassReference() {
        return this->Jxx - this->mu * this->Mz * this->Mz - this->mu * this->My * this->My;
    }

    /// "mass reference": set the displacement of the center of mass respect to
    /// the section centerline reference.
    void SetCenterOfMass(double my, double mz) {
        this->My = my;
        this->Mz = mz;
    }
    double GetCenterOfMassY() { return this->My; }
    double GetCenterOfMassZ() { return this->Mz; }

    // INTERFACES

    /// Gets the axial rigidity, usually A*E.
    virtual double GetAxialRigidity() const override { return this->Ax; }

    /// Gets the torsion rigidity, for torsion about X axis at elastic center, usually J*G.
    virtual double GetTorsionRigidityX() const override { return this->Txx; }

    /// Gets the bending rigidity, for bending about Y axis at elastic center, usually Iyy*E.
    virtual double GetBendingRigidityY() const override { return this->Byy; }

    /// Gets the bending rigidity, for bending about Z axis at elastic center, usually Izz*E.
    virtual double GetBendingRigidityZ() const override { return this->Bzz; }

    /// Set the rotation of the Y Z section axes for which the YbendingRigidity and ZbendingRigidity are defined.
    virtual double GetSectionRotation() const override { return this->alpha; }

    /// Gets the Y position of the elastic center respect to centerline.
    virtual double GetCentroidY() const override { return this->Cy; }
    /// Gets the Z position of the elastic center respect to centerline.
    virtual double GetCentroidZ() const override { return this->Cz; }

    /// Gets the Y position of the shear center respect to centerline.
    virtual double GetShearCenterY() const override { return this->Sy; }
    /// Gets the Z position of the shear center respect to centerline.
    virtual double GetShearCenterZ() const override { return this->Sz; }

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    virtual void ComputeInertiaMatrix(ChMatrix66d& M) override;

    /// Compute the 6x6 sectional inertia damping matrix [Ri] (gyroscopic matrix damping)
    virtual void ComputeInertiaDampingMatrix(
        ChMatrix66d& Ri,      ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
        const ChVector3d& mW  ///< current angular velocity of section, in material frame
        ) override;

    /// Compute the 6x6 sectional inertia stiffness matrix [Ki^]
    virtual void ComputeInertiaStiffnessMatrix(
        ChMatrix66d& Ki,          ///< 6x6 sectional inertial-stiffness matrix [Ki^] values here
        const ChVector3d& mWvel,  ///< current angular velocity of section, in material frame
        const ChVector3d& mWacc,  ///< current angular acceleration of section, in material frame
        const ChVector3d& mXacc   ///< current acceleration of section, in material frame (not absolute!)
        ) override;

    /// Compute the centrifugal term and gyroscopic term
    virtual void ComputeQuadraticTerms(ChVector3d& mF, ChVector3d& mT, const ChVector3d& mW) override;

    /// Get mass per unit length, ex.SI units [kg/m]
    virtual double GetMassPerUnitLength() const override { return this->mu; }

    /// Get the Jxx component of the inertia per unit length (polar inertia), at centerline.
    virtual double GetInertiaJxxPerUnitLength() const override { return this->Jxx; }
};

/// A simple specialization of ChBeamSectionEuler if you just need the simplest model
/// for a rectangular centered beam, with uniform elasticity and uniform density.
/// This section automatically itializes at construction:
/// - elasticity  as rectangular section
/// - inertia     as rectangular section
/// - damping:    none   - you can set it later
class ChApi ChBeamSectionEulerEasyRectangular : public ChBeamSectionEulerSimple {
  public:
    ChBeamSectionEulerEasyRectangular(double width_y,  ///< width of section in y direction
                                      double width_z,  ///< width of section in z direction
                                      double E,        ///< Young modulus
                                      double G,        ///< Shear modulus (only needed for the torsion)
                                      double density   ///< volumetric density (ex. in SI units: [kg/m^3])
    );
};

/// A simple specialization of ChBeamSectionEuler if you just need the simplest model
/// for a beam with circular centered section, with uniform elasticity and uniform density.
/// This section automatically itializes at construction:
/// - elasticity  as circular section
/// - inertia     as circular section
/// - damping:    none   - you can set it later
class ChApi ChBeamSectionEulerEasyCircular : public ChBeamSectionEulerSimple {
  public:
    ChBeamSectionEulerEasyCircular(double diameter,  ///< diameter of circular section
                                   double E,         ///< Young modulus
                                   double G,         ///< Shear modulus (only needed for the torsion)
                                   double density    ///< volumetric density (ex. in SI units: [kg/m^3])
    );
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Rayleigh beam sections, like Euler sections but adding the effect of Jyy Jzz rotational inertias

/// This works exactly as ChBeamSectionEulerSimple, but adds the effect of Jyy Jzz rotational sectional inertias,
/// whereas the conventional Euler theory would assume the mass to be concentrated in the center of mass, hence Jyy Jzz
/// =0. For wide sections, Jyy Jzz can become not negligible. In this simple symmetric case with homogeneous density,
/// the values of Jyy Jzz are computed automatically as
///  \f$ J_{yy} = \rho I_{yy} \f$,  \f$ J_{zz} = \rho I_{zz} \f$
/// This is a simple section model that assumes the elastic center, the shear center and the mass
/// center to be all in the centerline of the beam (section origin); this is the case of symmetric sections for example.
///
/// To be used with ChElementBeamEuler.
/// This material can be shared between multiple beams.
///
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratSimple.png"
///
class ChApi ChBeamSectionRayleighSimple : public ChBeamSectionEulerSimple {
  public:
    ChBeamSectionRayleighSimple() {}

    virtual ~ChBeamSectionRayleighSimple() {}

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    virtual void ComputeInertiaMatrix(ChMatrix66d& M) override;

    /// Compute the 6x6 sectional inertia damping matrix [Ri] (gyroscopic matrix damping)
    virtual void ComputeInertiaDampingMatrix(
        ChMatrix66d& Ri,      ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
        const ChVector3d& mW  ///< current angular velocity of section, in material frame
        ) override;

    /// Compute the 6x6 sectional inertia stiffness matrix [Ki^]
    virtual void ComputeInertiaStiffnessMatrix(
        ChMatrix66d& Ki,          ///< 6x6 sectional inertial-stiffness matrix [Ki^] values here
        const ChVector3d& mWvel,  ///< current angular velocity of section, in material frame
        const ChVector3d& mWacc,  ///< current angular acceleration of section, in material frame
        const ChVector3d& mXacc   ///< current acceleration of section, in material frame (not absolute!)
        ) override;

    /// Compute the centrifugal term and gyroscopic term
    virtual void ComputeQuadraticTerms(ChVector3d& mF, ChVector3d& mT, const ChVector3d& mW) override;
};

/// This works exactly as ChBeamSectionEulerEasyRectangular,
/// but adds the effect of Jyy Jzz rotational sectional inertias.
class ChApi ChBeamSectionRayleighEasyRectangular : public ChBeamSectionRayleighSimple {
  public:
    ChBeamSectionRayleighEasyRectangular(double width_y,  ///< width of section in y direction
                                         double width_z,  ///< width of section in z direction
                                         double E,        ///< Young modulus
                                         double G,        ///< Shear modulus (only needed for the torsion)
                                         double density   ///< volumetric density (ex. in SI units: [kg/m^3])
    );
};

/// This works exactly as ChBeamSectionEulerEasyCircular,
/// but adds the effect of Jyy Jzz rotational sectional inertias.
class ChApi ChBeamSectionRayleighEasyCircular : public ChBeamSectionRayleighSimple {
  public:
    ChBeamSectionRayleighEasyCircular(double diameter,  ///< diameter of circular section
                                      double E,         ///< Young modulus
                                      double G,         ///< Shear modulus (only needed for the torsion)
                                      double density    ///< volumetric density (ex. in SI units: [kg/m^3])
    );
};

/// This works exactly as ChBeamSectionEulerAdvancedGeneric,
/// but adds the effect of Jyy Jzz rotational sectional inertias.
/// The Jxx inertia of the Euler base class is automatically computed from Jyy Jzz by the polar theorem.
class ChApi ChBeamSectionRayleighAdvancedGeneric : public ChBeamSectionEulerAdvancedGeneric {
  protected:
    double Jzz;  // sectional inertia per unit length, in centerline reference, measured along centerline main axes
    double Jyy;  // sectional inertia per unit length, in centerline reference, measured along centerline main axes
    double Jyz;  // sectional inertia per unit length, in centerline reference, measured along centerline main axes
  public:
    ChBeamSectionRayleighAdvancedGeneric() : Jzz(0.5), Jyy(0.5), Jyz(0) {}

    ChBeamSectionRayleighAdvancedGeneric(
        const double mAx,      ///< axial rigidity
        const double mTxx,     ///< torsion rigidity
        const double mByy,     ///< bending regidity about yy
        const double mBzz,     ///< bending rigidity about zz
        const double malpha,   ///< section rotation about elastic center [rad]
        const double mCy,      ///< elastic center y displacement respect to centerline
        const double mCz,      ///< elastic center z displacement respect to centerline
        const double mSy,      ///< shear center y displacement respect to centerline
        const double mSz,      ///< shear center z displacement respect to centerline
        const double mmu,      ///< mass per unit length
        const double mJyy,     ///< inertia Jyy per unit lenght, in centerline reference, along centerline main axes
        const double mJzz,     ///< inertia Jzz per unit lenght, in centerline reference, along centerline main axes
        const double mJyz,     ///< inertia Jyz per unit lenght, in centerline reference, along centerline main axes
        const double mMy = 0,  ///< mass center y displacement respect to centerline
        const double mMz = 0   ///< mass center z displacement respect to centerline
        )
        : ChBeamSectionEulerAdvancedGeneric(mAx,
                                            mTxx,
                                            mByy,
                                            mBzz,
                                            malpha,
                                            mCy,
                                            mCz,
                                            mSy,
                                            mSz,
                                            mmu,
                                            (mJyy + mJzz),
                                            mMy,
                                            mMz),
          Jzz(mJzz),
          Jyy(mJyy),
          Jyz(mJyz) {}

    virtual ~ChBeamSectionRayleighAdvancedGeneric() {}

    /// Set the Jyy Jzz Jyz components of the sectional inertia per unit length,
    /// in centerline reference, measured along centerline main axes.
    /// These are defined as:
    /// \f$ J_{yy} =  \int_\Omega \rho z^2 d\Omega \f$, also Jyy = Mm(4,4)
    /// \f$ J_{zz} =  \int_\Omega \rho y^2 d\Omega \f$, also Jzz = Mm(5,5)
    /// \f$ J_{yz} =  \int_\Omega \rho y z  d\Omega \f$, also Jyz = -Mm(4,5) = -Mm(5,4)
    /// It is not needed to enter also Jxx because Jxx=(Jzz+Jyy) by the polar theorem.
    virtual void SetInertiasPerUnitLength(const double mJyy, const double mJzz, const double mJyz);

    /// Set inertia moments, per unit length, as assumed computed in the Ym Zm "mass reference"
    /// frame, ie. centered at the center of mass and rotated by phi angle to match the main axes of inertia:
    /// \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$,
    /// \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega \f$.
    /// Assuming the center of mass is already set.
    virtual void SetMainInertiasInMassReference(double Jmyy, double Jmzz, double phi);

    /// Get inertia moments, per unit length, as assumed computed in the Ym Zm "mass reference" frame, and the rotation
    /// phi of that frame, ie. inertias centered at the center of mass and rotated by phi angle to match the main axes
    /// of inertia: \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$, \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega
    /// \f$. Assuming the center of mass is already set.
    virtual void GetMainInertiasInMassReference(double& Jmyy, double& Jmzz, double& phi);

    // INTERFACES

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel}
    virtual void ComputeInertiaMatrix(ChMatrix66d& M) override;

    /// Compute the 6x6 sectional inertia damping matrix [Ri] (gyroscopic matrix damping)
    virtual void ComputeInertiaDampingMatrix(
        ChMatrix66d& Ri,      ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
        const ChVector3d& mW  ///< current angular velocity of section, in material frame
        ) override;

    /// Compute the 6x6 sectional inertia stiffness matrix [Ki^]
    virtual void ComputeInertiaStiffnessMatrix(
        ChMatrix66d& Ki,          ///< 6x6 sectional inertial-stiffness matrix [Ki^] values here
        const ChVector3d& mWvel,  ///< current angular velocity of section, in material frame
        const ChVector3d& mWacc,  ///< current angular acceleration of section, in material frame
        const ChVector3d& mXacc   ///< current acceleration of section, in material frame (not absolute!)
        ) override;

    /// Compute the centrifugal term and gyroscopic term
    virtual void ComputeQuadraticTerms(ChVector3d& mF, ChVector3d& mT, const ChVector3d& mW) override;
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
