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

#ifndef CHBEAMSECTIONCOSSERAT_H
#define CHBEAMSECTIONCOSSERAT_H

#include "chrono/fea/ChBeamSection.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

//  forward
class ChBeamSectionCosserat;

/// Base interface for elasticity of beam sections of Cosserat type,
/// where xyz force "n" and xyz torque "m" are a 6-dimensional function of
/// generalized strains, "e" traction/shear and "k" curvatures, as:
///   {n,m}=f({e,k})
/// There are various children classes that implement this function in different ways.
/// Note that the Timoshenko beam theory can be a sub-case of this.
class ChApi ChElasticityCosserat {
  public:
    ChElasticityCosserat() : section(nullptr) {}

    virtual ~ChElasticityCosserat() {}

    /// Compute the generalized cut force and cut torque, given actual deformation and curvature.
    virtual void ComputeStress(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) = 0;

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;,
    /// given actual deformation and curvature (if needed).
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Km] by numerical differentiation calling ComputeStress() multiple times.
    virtual void ComputeStiffnessMatrix(
        ChMatrixNM<double, 6, 6>& K, ///< 6x6 material stiffness matrix values here
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
    );

    ChBeamSectionCosserat* section;
};

/// Simple linear elasticity model for a Cosserat beam, using basic material
/// properties (zz and yy moments of inertia, area, Young modulus, etc.).
/// Uniform stiffness properties E,G are hence assumed through the section.
/// The classical Timoshenko beam theory is encompassed in this model, that
/// can be interpreted as a 3D extension of the Timoshenko beam theory.
/// This can be shared between multiple beams.
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratSimple.png"
/// 
class ChApi ChElasticityCosseratSimple : public ChElasticityCosserat {
  public:
    double Iyy;
    double Izz;
    double J;
    double G;
    double E;
	double A;

    double Ks_y;
    double Ks_z;

    ChElasticityCosseratSimple();

    ChElasticityCosseratSimple(
        const double mIyy,   ///< Iyy second moment of area of the beam \f$ I_y =  \int_\Omega z^2 dA \f$
        const double mIzz,   ///< Izz second moment of area of the beam \f$ I_z =  \int_\Omega y^2 dA \f$
        const double mJ,     ///< torsion constant (torsion rigidity will be G*J, torsional stiffness = G*J*length)
        const double mG,     ///< G shear modulus
        const double mE,     ///< E young modulus 
        const double mA,     ///< A area
        const double mKs_y,  ///< Timoshenko shear coefficient Ks for y shear
        const double mKs_z   ///< Timoshenko shear coefficient Ks for z shear
    ) :
        Iyy(mIyy), Izz(mIzz), J(mJ), G(mG), E(mE), A(mA), Ks_y(mKs_y), Ks_z(mKs_z)
    {}

    virtual ~ChElasticityCosseratSimple() {}

	/// Set the A area of the beam.
    void SetArea(double ma) { this->A = ma; }
    double GetArea() const { return this->A; }

    /// Set the Iyy second moment of area of the beam (for bending about y in xz plane),
	/// defined as \f$ I_y =  \int_\Omega z^2 dA \f$.
    /// Note: some textbook calls this Iyy as Iy
	/// Ex SI units: [m^4]
    void SetIyy(double ma) { this->Iyy = ma; }
    double GetIyy() const { return this->Iyy; }

    /// Set the Izz second moment of area of the beam (for bending about z in xy plane). 
	/// defined as \f$ I_z =  \int_\Omega y^2 dA \f$.
    /// Note: some textbook calls this Izz as Iz
	/// Ex SI units: [m^4]
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
    /// with rectangular shape. You must set E and G anyway.
    virtual void SetAsRectangularSection(double width_y, double width_z);

    /// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
    /// at once, given the diameter of the beam assumed
    /// with circular shape. You must set E and G anyway.
    virtual void SetAsCircularSection(double diameter);

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set G, the shear modulus
    void SetGshearModulus(double mG) { this->G = mG; }
    double GetGshearModulus() const { return this->G; }

    /// Set G, the shear modulus, given current E and the specified Poisson ratio
    void SetGwithPoissonRatio(double mpoisson) { this->G = this->E / (2.0 * (1.0 + mpoisson)); }



    // Interface to base:

    /// Compute the generalized cut force and cut torque.
    virtual void ComputeStress(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    virtual void ComputeStiffnessMatrix(
        ChMatrixNM<double, 6, 6>& K, ///< 6x6 stiffness matrix
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;
};

/// Generic linear elasticity for a Cosserat beam using directly a 6x6 matrix [E] 
/// as user-input data. 
/// The [E] matrix values ("rigidity" values) can be computed from a preprocessing
/// stage using a FEA analysis over a detailed 3D model of a chunk of beam,
/// hence recovering the 6x6 matrix that connects yxz displacements "e" and
/// xyz rotations "k" to the xyz cut-force "n" and xyz cut-torque "m" as in
///   \f$ (m,n)=[E](e,k) \f$ 
/// where \f$ e, k, m, n \f$  are expressed in the centerline reference.
/// Using a matrix of rigidity values, the model does not assume homogeneous elasticity
/// and bypasses the need of entering E or G values.
/// This can be shared between multiple beams.
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratGeneric.png"
/// 
class ChApi ChElasticityCosseratGeneric : public ChElasticityCosserat {
  public:
    ChElasticityCosseratGeneric();

    virtual ~ChElasticityCosseratGeneric() {}

    /// Access the E matrix, for getting/setting its values.
    /// This is the matrix that defines the linear elastic constitutive model
    /// as it maps  yxz displacements "e" and xyz rotations "k"
    /// to the xyz cut-force "n" and xyz cut-torque "m" as in
    ///    \f$ (m,n)=[E](e,k) \f$ 
    ChMatrixNM<double, 6, 6>& Ematrix() { return this->mE; }


    // Interface to base:

    /// Compute the generalized cut force and cut torque.
    virtual void ComputeStress(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    virtual void ComputeStiffnessMatrix(
        ChMatrixNM<double, 6, 6>& K, ///< 6x6 stiffness matrix
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;

  private:
    ChMatrixNM<double, 6, 6> mE;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Advanced linear elasticity for a Cosserat beam.
/// Uniform stiffness properties E,G are assumed through the section as in
/// ChElasticityCosseratSimple, but here it also supports the advanced case of
/// Iyy and Izz axes rotated respect reference, elastic center with offset
/// from reference, and shear center with offset from reference.
/// This material can be shared between multiple beams.
/// The linear elasticity is uncoupled between shear terms S and axial terms A
/// as to have this stiffness matrix pattern:
/// <pre>
///  n_x   [A       A A ]   e_x
///  n_y   [  S S S     ]   e_y
///  n_z = [  S S S     ] * e_z
///  m_x   [  S S S     ]   k_x
///  m_y   [A       A A ]   k_y
///  m_z   [A       A A ]   k_z
///  </pre>
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratAdvanced.png"
/// 
class ChApi ChElasticityCosseratAdvanced : public ChElasticityCosseratSimple {
  public:
    double alpha;  ///< Rotation of Izz Iyy respect to reference section, centered on line x
    double Cy;     ///< Elastic center, respect to reference section (elastic center, tension center)
    double Cz;     ///<
    double beta;   ///< Rotation of shear reference section, centered on line x
    double Sy;     ///< Shear center, respect to reference section
    double Sz;     ///<

    ChElasticityCosseratAdvanced();

    ChElasticityCosseratAdvanced(
        const double mIyy,   ///< Iyy second moment of area of the beam \f$ I_y =  \int_\Omega z^2 dA \f$
        const double mIzz,   ///< Izz second moment of area of the beam \f$ I_z =  \int_\Omega y^2 dA \f$
        const double mJ,     ///< torsion constant (torsion rigidity will be G*J, torsional stiffness = G*J*length)
        const double mG,     ///< G shear modulus
        const double mE,     ///< E young modulus 
        const double mA,     ///< A area
        const double mKs_y,  ///< Timoshenko shear coefficient Ks for y shear
        const double mKs_z,  ///< Timoshenko shear coefficient Ks for z shear
        const double malpha, ///< section rotation for which Iyy Izz are computed
        const double mCy,    ///< Cy offset of elastic center about which Iyy Izz are computed
        const double mCz,    ///< Cz offset of elastic center about which Iyy Izz are computed
        const double mbeta, ///< section rotation for which Ks_y Ks_z are computed
        const double mSy,    ///< Sy offset of shear center
        const double mSz     ///< Sz offset of shear center
    ) :
        ChElasticityCosseratSimple(mIyy, mIzz, mJ, mG, mE, mA, mKs_y, mKs_z), alpha(malpha), Cy(mCy), Cz(mCz), beta(mbeta), Sy(mSy), Sz(mSz)
    {}


    virtual ~ChElasticityCosseratAdvanced() {}

    /// "Elastic reference": set alpha, the rotation of the section for which the Iyy Izz are
    /// defined, respect to the reference section coordinate system placed at centerline.
    void SetSectionRotation(double ma) { this->alpha = ma; }
    double GetSectionRotation() { return this->alpha; }

    /// "Elastic reference": set the displacement of the elastic center 
    /// (or tension center) respect to the reference section coordinate system placed at centerline.
    void SetCentroid(double my, double mz) {
        this->Cy = my;
        this->Cz = mz;
    }
    double GetCentroidY() { return this->Cy; }
    double GetCentroidZ() { return this->Cz; }

    /// "Shear reference": set beta, the rotation of the section for shear decoupling, respect to
    /// the reference section coordinate system placed at centerline. 
    void SetShearRotation(double mb) { this->beta = mb; }
    double GetShearRotation() { return this->beta; }

    /// "Shear reference": set the displacement of the shear center S
    /// respect to the reference beam line placed at centerline. For shapes like rectangles,
    /// rotated rectangles, etc., it corresponds to the elastic center C, but
    /// for "L" shaped or "U" shaped beams this is not always true, and
    /// the shear center accounts for torsion effects when a shear force is applied.
    void SetShearCenter(double my, double mz) {
        this->Sy = my;
        this->Sz = mz;
    }
    double GetShearCenterY() { return this->Sy; }
    double GetShearCenterZ() { return this->Sz; }

    // Interface to base:

    /// Compute the generalized cut force and cut torque.
    virtual void ComputeStress(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    virtual void ComputeStiffnessMatrix(
        ChMatrixNM<double, 6, 6>& K, ///< 6x6 stiffness matrix
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;
};

/// Advanced linear elasticity for a Cosserat section, not assuming homogeneous elasticity. 
/// This is the case where one uses a FEA preprocessor to compute the rigidity of a complex beam 
/// made with multi-layered reinforcements with different elasticity - in such a case you could not
/// use ChElasticityCosseratAdvanced because you do not have a single E or G, but you rather
/// have collective values of bending/shear/axial rigidities. This class allows using these values directly,
/// bypassing any knowledge of area, Izz Iyy, E young modulus, etc.
/// This material can be shared between multiple beams.
/// The linear elasticity is uncoupled between shear terms S and axial terms A
/// as to have this stiffness matrix pattern:
/// <pre>
///  n_x   [A       A A ]   e_x
///  n_y   [  S S S     ]   e_y
///  n_z = [  S S S     ] * e_z
///  m_x   [  S S S     ]   k_x
///  m_y   [A       A A ]   k_y
///  m_z   [A       A A ]   k_z
///  </pre>
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratAdvanced.png"
/// 

class ChApi ChElasticityCosseratAdvancedGeneric : public ChElasticityCosserat {
private:
    double Ax;      // axial rigidity
    double Txx;     // torsion rigidity
    double Byy;     // bending rigidity
    double Bzz;     // bending rigidity
    double Hyy;     // shear rigidity
    double Hzz;     // shear rigidity
    double alpha;   // rotation of reference at elastic center, for bending effects [rad]
    double Cy;      // Centroid (elastic center, tension center)
    double Cz;
    double beta;    // rotation of reference at shear center, for shear effects [rad]
    double Sy;      // Shear center
    double Sz;
public:

    ChElasticityCosseratAdvancedGeneric() : Ax(1), Txx(1), Byy(1), Bzz(1), Hyy(1), Hzz(1), alpha(0),Cy(0),Cz(0), beta(0), Sy(0),Sz(0) {}

    ChElasticityCosseratAdvancedGeneric(    const double mAx,      ///< axial rigidity
                                            const double mTxx,     ///< torsion rigidity
                                            const double mByy,     ///< bending regidity on Y of reference at elastic center
                                            const double mBzz,     ///< bending rigidity on Z of reference at elastic center
                                            const double mHyy,     ///< shear rigidity on Y of reference at shear center
                                            const double mHzz,     ///< shear rigidity on Y of reference at shear center
                                            const double malpha,   ///< rotation of reference at elastic center, for bending effects [rad]
                                            const double mCy,      ///< elastic center y displacement respect to centerline
                                            const double mCz,      ///< elastic center z displacement respect to centerline
                                            const double mbeta,    ///< rotation of reference at shear center, for shear effects [rad]
                                            const double mSy,      ///< shear center y displacement respect to centerline
                                            const double mSz       ///< shear center z displacement respect to centerline
    ) :
        Ax(mAx), Txx(mTxx), Byy(mByy), Bzz(mBzz), Hyy(mHyy), Hzz(mHzz), alpha(malpha), Cy(mCy), Cz(mCz), beta(mbeta), Sy(mSy), Sz(mSz) {}


    virtual ~ChElasticityCosseratAdvancedGeneric() {}


    /// Sets the axial rigidity, usually A*E for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetAxialRigidity(const double mv) {
        Ax = mv;
    }

    /// Sets the torsion rigidity, for torsion about X axis, at elastic center, 
    /// usually J*G for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor 
    virtual void SetXtorsionRigidity(const double mv) {
        Txx = mv;
    }

    /// Sets the bending rigidity, for bending about Y axis, at elastic center, 
    /// usually Iyy*E for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetYbendingRigidity(const double mv) {
        Byy = mv;
    }

    /// Sets the bending rigidity, for bending about Z axis, at elastic center, 
    /// usually Izz*E for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetZbendingRigidity(const double mv) {
        Bzz = mv;
    }

    /// Sets the shear rigidity, for shear about Y axis, at shear center, 
    /// usually A*G*(Timoshenko correction factor) for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetYshearRigidity(const double mv) {
        Hyy = mv;
    }

    /// Sets the shear rigidity, for shear about Z axis, at shear center, 
    /// usually A*G*(Timoshenko correction factor) for uniform elasticity, but for nonuniform elasticity
    /// here you can put a value ad-hoc from a preprocessor
    virtual void SetZshearRigidity(const double mv) {
        Hzz = mv;
    }

    /// Set the rotation in [rad]  of the Y Z axes for which the 
    /// YbendingRigidity and ZbendingRigidity values are defined. 
    void SetSectionRotation(double ma) { this->alpha = ma; }
    double GetSectionRotation() { return this->alpha; }

    /// "Elastic reference": set the displacement of the elastic center 
    /// (or tension center) respect to the reference section coordinate system placed at centerline.
    void SetCentroid(double my, double mz) {
        this->Cy = my;
        this->Cz = mz;
    }
    double GetCentroidY() { return this->Cy; }
    double GetCentroidZ() { return this->Cz; }

    /// Set the rotation in [rad] of the Y Z axes for which the 
    /// YshearRigidity and ZshearRigidity values are defined. 
    void SetShearRotation(double mb) { this->beta = mb; }
    double GetShearRotation() { return this->beta; }

    /// "Shear reference": set the displacement of the shear center S
    /// respect to the reference beam line placed at centerline. For shapes like rectangles,
    /// rotated rectangles, etc., it corresponds to the elastic center C, but
    /// for "L" shaped or "U" shaped beams this is not always true, and
    /// the shear center accounts for torsion effects when a shear force is applied.
    void SetShearCenter(double my, double mz) {
        this->Sy = my;
        this->Sz = mz;
    }
    double GetShearCenterY() { return this->Sy; }
    double GetShearCenterZ() { return this->Sz; }

    // Interface to base:

    /// Compute the generalized cut force and cut torque.
    virtual void ComputeStress(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    virtual void ComputeStiffnessMatrix(
        ChMatrixNM<double, 6, 6>& K, ///< 6x6 stiffness matrix
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;

};


/// Elasticity for a beam section in 3D, where the section is
/// defined by a mesh of triangles.
/// This model saves you from the need of knowing I_z, I_y, A, etc.,
/// because the generalized n and m are automatically computed by integrating stresses
/// on the triangulated section. Note that stresses are linearly interpolated
/// between the vertexes of the triangle sections.
/// Triangles can share vertexes.
/// Each vertex has its own material.
/// Section is assumed always flat, even if the section mesh is not connected, ex.
/// if one models a section like a "8" shape where the two "o" are not connected.
///
/// Benefits:
/// - no need to provide I_y, I_z, A, etc.
/// - possibility of getting values of stresses in different points of the section
/// - possibility of using some 1D plasticity to discover plasticizing zones in the section
///
/// Limitations (TO BE REMOVED IN FUTURE using Vlasov / Prandtl theories):
/// - section torsional warping not included,
/// - torsion stresses are correct only in tube-like shapes, or similar; other models such as
///   ChElasticityCosseratAdvanced contain torsional effects via the macroscopic J constant; here there is
///   a torsion correction factor just for correcting the m_x result, but at this point, shear in material points would
///   have less meaning.
/// - shear stresses (ex. cantilever with transverse load) should be almost parabolic in the section in reality,
///   here would be constant. Other models such as ChElasticityCosseratAdvanced correct this effect at the macroscopic
///   level using the Timoshenko correction factors Ks_y and Ks_z, here they are used as well, but if so, shear in
///   material points would have less meaning.
///
/// This material can be shared between multiple beams.
///
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratMesh.png"
///
class ChApi ChElasticityCosseratMesh : public ChElasticityCosserat {
  public:
    class ChSectionMaterial {
      public:
        ChSectionMaterial(double mE = 1.0, double mG = 1.0) : E(mE), G(mG) {}
        double E;
        double G;
    };

    ChElasticityCosseratMesh() {}

    virtual ~ChElasticityCosseratMesh() {}

    /// Access the list of vertexes, to get/change/add mesh section vertexes.
    virtual std::vector<ChVector2<>>& Vertexes() { return vertexes; }

    /// Access the list of material(s), to get/change/add mesh section materials.
    /// Each material correspond to an equivalent vertex.
    /// If there is only one material, it will be used for all vertexes.
    std::vector<std::shared_ptr<ChSectionMaterial>>& Materials() { return materials; }

    /// Access the list of triangles, to get/change/add mesh section triangles.
    /// Each triangle has three integer indexes pointing to the three connected vertexes
    /// in the Vertexes() array, where 0 is the 1st vertex etc.
    std::vector<ChVector<int>>& Triangles() { return triangles; }

    /// Set rectangular centered section, using two triangles. 
	/// Note: for testing only, use ChElasticityCosseratSimple instead. 
	/// No material defined: you still must set E and G.
    virtual void SetAsRectangularSection(double width_y, double width_z);

    /// Set circular centered, using n triangles. 
	/// Note: for testing only, use ChElasticityCosseratSimple instead. 
	/// No material defined: you still must set E and G.
    virtual void SetAsCircularSection(double diameter);

    // Interface to base:

    /// Compute the generalized cut force and cut torque.
    virtual void ComputeStress(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& strain_e,  ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k   ///< local strain (curvature part), x= torsion, y and z are line curvatures
        ) override;

    /*
    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    /// * for the moment, defaults to numerical differentiation *
    virtual void ComputeStiffnessMatrix(
        ChMatrixNM<double, 6, 6>& K,///< 6x6 stiffness matrix
        const ChVector<>& strain_e, ///< local strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k  ///< local strain (curvature part), x= torsion, y and z are line curvatures
    ) override;
    */

  protected:
    std::vector<ChVector2<>> vertexes;
    std::vector<std::shared_ptr<ChSectionMaterial>> materials;
    std::vector<ChVector<int>> triangles;
};


//----------------------------------------------------------------------------------------


/// Base class for plasticity of beam sections of Cosserat type.
/// This can be shared between multiple beams.
class ChApi ChPlasticityCosserat {
  public:
    ChPlasticityCosserat();

    virtual ~ChPlasticityCosserat() {}

    // Given a trial strain, it computes the effective stress and strain by
    // clamping against the yeld surface. An implicit return mapping integration
    // step is computed automatically per each call of this function.
    // Note: for the elastic part, it must use the elasticity model in this->section->elasticity.
    // If not beyond yeld, simply:
    //      elastic strain = tot strain - plastic strain
    // If it is beyond yeld:
    //      elastic strain is computed by fully implicit strain integration with return mapping,
    //      and plastic strains in "data_new" are updated.
    // Returns true if it had to do return mapping, false if it was in elastic regime
    virtual bool ComputeStressWithReturnMapping(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        ChVector<>& e_strain_e_new,  ///< updated elastic strain (deformation part)
        ChVector<>& e_strain_k_new,  ///< updated elastic strain (curvature part)
        ChBeamMaterialInternalData& data_new,  ///< updated material internal variables, at this point, including
                                               ///< {p_strain_e, p_strain_k, p_strain_acc}
        const ChVector<>& tot_strain_e,  ///< trial tot strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& tot_strain_k,  ///< trial tot strain (curvature part), x= torsion, y and z are line curvatures
        const ChBeamMaterialInternalData& data  ///< trial material internal variables, at this point, including
                                                ///< {p_strain_e, p_strain_k, p_strain_acc}
        ) = 0;

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;,
    /// given actual internal data and deformation and curvature (if needed). If in
    /// plastic regime, uses elastoplastic matrix, otherwise uses elastic.
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Km] by numerical differentiation calling ComputeStressWithReturnMapping() multiple times.
    virtual void ComputeStiffnessMatrixElastoplastic(
        ChMatrixNM<double, 6, 6>& K, ///< 6x6 material stiffness matrix values here
        const ChVector<>& strain_e,  ///< tot strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k,  ///< tot strain (curvature part), x= torsion, y and z are line curvatures
        const ChBeamMaterialInternalData& data  ///< updated material internal variables, at this point,
                                                ///< including {p_strain_e, p_strain_k, p_strain_acc}
    );

    // Populate a vector with the appropriate ChBeamSectionPlasticity data structures.
    // Children classes may override this. By default uses ChBeamMaterialInternalData for basic plasticity.
    // Thanks to unique_ptr there is no need to call delete for the pointed objects.
    virtual void CreatePlasticityData(int numpoints,
                                      std::vector<std::unique_ptr<ChBeamMaterialInternalData>>& plastic_data);


    ChBeamSectionCosserat* section;
    double nr_yeld_tolerance;
    int nr_yeld_maxiters;
};

/// Internal variables for basic lumped plasticity in Cosserat beams.
class ChApi ChInternalDataLumpedCosserat : public ChBeamMaterialInternalData {
  public:
    ChInternalDataLumpedCosserat() {}

    virtual ~ChInternalDataLumpedCosserat() {}

    virtual void Copy(const ChBeamMaterialInternalData& other) override {
        ChBeamMaterialInternalData::Copy(other);

        if (auto mother = dynamic_cast<const ChInternalDataLumpedCosserat*>(&other)) {
            p_strain_e = mother->p_strain_e;
            p_strain_k = mother->p_strain_k;
            p_strain_acc_e = mother->p_strain_acc_e;
            p_strain_acc_k = mother->p_strain_acc_k;
        }
    }

    ChVector<> p_strain_acc_e;  // separate strain accumulator for xyz
    ChVector<> p_strain_acc_k;  // separate strain accumulator for xyz
    ChVector<> p_strain_e;
    ChVector<> p_strain_k;
};

/// Lumped plasticity of Cosserat-type beams.
/// This defines 6 independent yelds for the six generalized forces/moments in the beam.
/// Note that this is a rough approximation of plasticity in beams for at least two
/// main reasons: it cannot capture how plastic zones are made inside a section (which
/// is mostly important when cycling with back and forth bending), and it does not
/// capture coupled My+Mz effects, or Nx+My or Nx+Mz as often happens. Briefly: use
/// it if plasticization happens in a scenario of pure bending on a single xy or xz plane,
/// or pure compression/extension, or pure torsion.
/// This can be shared between multiple beams.
class ChApi ChPlasticityCosseratLumped : public ChPlasticityCosserat {
  public:
    /// Default constructor: linear isotropic constant hardening
    ChPlasticityCosseratLumped();

    virtual ~ChPlasticityCosseratLumped() {}

    /// Given a trial strain, it computes the effective stress and strain by
    /// clamping against the yeld surface. An implicit return mapping integration
    /// step is computed automatically per each call of this function.
    /// Returns true if it had to do return mapping, false if it was in elastic regime
    virtual bool ComputeStressWithReturnMapping(
        ChVector<>& stress_n,        ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,        ///< local stress (generalized torque), x component = torsion torque along beam
        ChVector<>& e_strain_e_new,  ///< updated elastic strain (deformation part)
        ChVector<>& e_strain_k_new,  ///< updated elastic strain (curvature part)
        ChBeamMaterialInternalData& data_new,  ///< updated material internal variables, at this point, including
                                               ///< {p_strain_e, p_strain_k, p_strain_acc}
        const ChVector<>& tot_strain_e,  ///< trial tot strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& tot_strain_k,  ///< trial tot strain (curvature part), x= torsion, y and z are line curvatures
        const ChBeamMaterialInternalData& data  ///< current material internal variables, at this point, including
                                                ///< {p_strain_e, p_strain_k, p_strain_acc}
        ) override;

    /*
    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;,
    /// given actual internal data and deformation and curvature (if needed). If in
    /// plastic regime, uses elastoplastic matrix, otherwise uses elastic.
    virtual void ComputeStiffnessMatrixElastoplastic(
        ChMatrixNM<double, 6, 6>& K,///< 6x6 material stiffness matrix values here
        const ChVector<>& strain_e, ///< tot strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k, ///< tot strain (curvature part), x= torsion, y and z are line curvatures
        ChBeamMaterialInternalData& data ///< updated material internal variables, at this point, including
    {p_strain_e, p_strain_k, p_strain_acc} ) override {
        ...
    }
    */

    // Populate a vector with the appropriate ChBeamSectionPlasticity data structures.
    // Children classes may override this. By default uses ChBeamMaterialInternalData for basic plasticity.
    // Thanks to unique_ptr there is no need to call delete for the pointed objects.
    virtual void CreatePlasticityData(int numpoints,
                                      std::vector<std::unique_ptr<ChBeamMaterialInternalData>>& plastic_data) override;


    std::shared_ptr<ChFunction> n_yeld_x;   ///< sigma_y(p_strain_acc)
    std::shared_ptr<ChFunction> n_beta_x;   ///< beta(p_strain_acc)
    std::shared_ptr<ChFunction> n_yeld_y;   ///< sigma_y(p_strain_acc)
    std::shared_ptr<ChFunction> n_beta_y;   ///< beta(p_strain_acc)
    std::shared_ptr<ChFunction> n_yeld_z;   ///< sigma_y(p_strain_acc)
    std::shared_ptr<ChFunction> n_beta_z;   ///< beta(p_strain_acc)
    std::shared_ptr<ChFunction> n_yeld_Mx;  ///< sigma_y(p_strain_acc)
    std::shared_ptr<ChFunction> n_beta_Mx;  ///< beta(p_strain_acc)
    std::shared_ptr<ChFunction> n_yeld_My;  ///< sigma_y(p_strain_acc)
    std::shared_ptr<ChFunction> n_beta_My;  ///< beta(p_strain_acc)
    std::shared_ptr<ChFunction> n_yeld_Mz;  ///< sigma_y(p_strain_acc)
    std::shared_ptr<ChFunction> n_beta_Mz;  ///< beta(p_strain_acc)
};


//-----------------------------------------------------------------------------------------------



/// Base interface for structural damping of beam sections of Cosserat type,
/// where xyz force "n" and xyz torque "m" are a 6-dimensional function of
/// generalized strain speeds, "e'" traction/shear speed and "k'" curvature speed, as:
///   {n,m}=f({e',k'})
/// Children classes implement this function in different ways.
class ChApi ChDampingCosserat {
  public:
    ChDampingCosserat() : section(nullptr) {}

    virtual ~ChDampingCosserat() {}

    /// Compute the generalized cut force and cut torque, caused by structural damping,
    /// given actual deformation speed and curvature speed.
    /// This MUST be implemented by subclasses.
    virtual void ComputeStress(
        ChVector<>& stress_n,         ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,         ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& dstrain_e,  ///< local strain speed (deformation); x elongation speed; y,z shear speeds
        const ChVector<>& dstrain_k   ///< local strain speed (curvature); x torsion speed; y, z line curvature speeds
        ) = 0;

    /// Compute the 6x6 tangent material damping matrix, ie the jacobian [Rm]=dstress/dstrainspeed.
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Rm] by numerical differentiation calling ComputeStress() multiple times.
    virtual void ComputeDampingMatrix(ChMatrixNM<double, 6, 6>& R,  ///< 6x6 material stiffness matrix values here
                                      const ChVector<>& dstrain_e,  ///< current strain speed (deformation part)
                                      const ChVector<>& dstrain_k   ///< current strain speed (curvature part)
    );


    ChBeamSectionCosserat* section;
};

/// Simple linear lumped damping of beam sections of Cosserat type,
///   {n,m}=f({e',k'})
/// where damping is proportional to speed of deformation/curvature via
/// linear constants:
///   <pre>
///   n = R_e * e'
///   m = R_k * k'
///   </pre>
class ChApi ChDampingCosseratLinear : public ChDampingCosserat {
  public:
    ChDampingCosseratLinear() {}

    virtual ~ChDampingCosseratLinear() {}

    /// Compute the generalized cut force and cut torque, caused by structural damping,
    /// given actual deformation speed and curvature speed.
    virtual void ComputeStress(
        ChVector<>& stress_n,         ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,         ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& dstrain_e,  ///< local strain speed (deformation); x elongation speed; y,z shear speeds
        const ChVector<>& dstrain_k   ///< local strain speed (curvature); x torsion speed; y,z line curvature speeds
        ) override;

    /// Compute the 6x6 tangent material damping matrix, ie the jacobian [Rm]=dstress/dstrainspeed.
    /// By the way, in this model, it is simply a diagonal matrix with R_e and R_k values on the diagonal.
    virtual void ComputeDampingMatrix(ChMatrixNM<double, 6, 6>& R,  ///< 6x6 material stiffness matrix values here
                                      const ChVector<>& dstrain_e,  ///< current strain speed (deformation part)
                                      const ChVector<>& dstrain_k   ///< current strain speed (curvature part)
                                      ) override;

    ChVector<> GetDampingCoefficientsRe() { return R_e; }
    void SetDampingCoefficientsRe(const ChVector<> mR_e) { R_e = mR_e; }

    ChVector<> GetDampingCoefficientsRk() { return R_k; }
    void SetDampingCoefficientsRk(const ChVector<> mR_k) { R_k = mR_k; }


  private:
    ChVector<> R_e;
    ChVector<> R_k;
};


/// Simple Rayleigh damping of beam sections of Cosserat type,
/// where damping is proportional to stiffness via a beta coefficient.
/// In order to generalize it also in case of nonlinearity, the full
/// element tangent stiffness matrix cannot be used (it may contain negative eigenvalues)
/// and it can't be used to recover instant nodal caused by damping as F=beta*K*q_dt
/// so it is generalized to the following implementation at the material stress level
///   <pre>
///   {n,m}=beta*[E]*{e',k'}
///   </pre>
/// where 
/// - beta is the 2nd Rayleigh damping parameter
/// - [E] is the 6x6 material stiffness matrix at the undeformed unstressed case (hence assumed constant)
/// - {e',k'} is the speed of deformation/curvature
/// Note that the alpha mass-proportional parameter (the first of the alpha,beta parameters of the original
/// Rayleigh model) is not supported.

class ChApi ChDampingCosseratRayleigh : public ChDampingCosserat {
  public:
		/// Construct the Rayleigh damping model from the stiffness model used by the section.
		/// This is important because the Rayleigh damping is proportional to the stiffness,
		/// so the model must know which is the stiffness matrix of the material.
	    /// Note: melasticity must be alreay set with proper values: its [E] stiffness matrix will be
		/// fetched just once for all.
	ChDampingCosseratRayleigh(std::shared_ptr<ChElasticityCosserat> melasticity, const double& mbeta = 0);

	virtual ~ChDampingCosseratRayleigh() {}

    /// Compute the generalized cut force and cut torque, caused by structural damping,
    /// given actual deformation speed and curvature speed.
    virtual void ComputeStress(
        ChVector<>& stress_n,         ///< local stress (generalized force), x component = traction along beam
        ChVector<>& stress_m,         ///< local stress (generalized torque), x component = torsion torque along beam
        const ChVector<>& dstrain_e,  ///< local strain speed (deformation); x elongation speed; y,z shear speeds
        const ChVector<>& dstrain_k   ///< local strain speed (curvature); x torsion speed; y,z line curvature speeds
        ) override;

    /// Compute the 6x6 tangent material damping matrix, ie the jacobian [Rm]=dstress/dstrainspeed.
    /// In this model, it is beta*[E] where [E] is the 6x6 stiffness matrix at material level, assumed constant
    virtual void ComputeDampingMatrix(ChMatrixNM<double, 6, 6>& R,  ///< 6x6 material stiffness matrix values here
                                      const ChVector<>& dstrain_e,  ///< current strain speed (deformation part)
                                      const ChVector<>& dstrain_k   ///< current strain speed (curvature part)
                                      ) override;

	/// Get the beta Rayleigh parameter (stiffness proportional damping)
    double GetBeta() { return beta; }
    /// Set the beta Rayleigh parameter (stiffness proportional damping)
	void SetBeta(const double mbeta) { beta = mbeta; }

	/// After you added this damping to a ChBeamSectionCosserat, in case you have changed some parameters in the stiffness model after creating 
	/// this Rayleigh damping, you must call this UpdateStiffnessModel() method so that here we
	/// update the [E] 6x6 material stiffness matrix, which is stored here as private and constant data for performance
	void UpdateStiffnessModel();


  private:
	std::shared_ptr<ChElasticityCosserat> section_elasticity;
    ChMatrixNM<double, 6, 6> E_const; // to store the precomputed stiffness matrix at undeformed unstressed initial state
	double beta;
	bool updated;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



////////////////////////////////////////////////////////////////////////////////////


/// Base class for ineri tal properties (mass, moment of inertia) of beam sections of Cosserat type.
/// This can be shared between multiple beams.
class ChApi ChInertiaCosserat {
  public:
	ChInertiaCosserat() : section(nullptr) {};

    virtual ~ChInertiaCosserat() {}

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel} 
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix)
    virtual void ComputeInertiaMatrix(ChMatrixNM<double, 6, 6>& M  ///< 6x6 sectional mass matrix values here
                                      ) = 0;

    /// Compute the values of inertial force & torque depending on quadratic velocity terms,
    /// that is the gyroscopic torque and the centrifugal term (if any). All terms expressed 
    /// in the material reference, ie. the reference in the centerline of the section.
    virtual void ComputeQuadraticTerms(ChVector<>& mF,   ///< centrifugal term (if any) returned here
                                       ChVector<>& mT,   ///< gyroscopic term  returned here
                                       const ChVector<>& mW    ///< current angular velocity of section, in material frame
                                      ) = 0;

    /// Compute mass per unit length, ex.SI units [kg/m]. 
    /// This is also the(0, 0) element in the sectional inertia matrix.
    virtual double GetMassPerUnitLength() = 0;

	ChBeamSectionCosserat* section;
};



/// Inertia properties of a beam of Cosserat type, defined from an uniform density [kg/m^3], 
/// and the following geometric information:
///  - a section area 
///  - Iyy Izz second moments of area
/// The polar moment of area is automatically inferred via perpendicular axis theorem, Ip=Iyy+Izz.
/// The section is assumed aligned to principal axis of the moment of area tensor, ie. Iyz=0,
/// The section is assumed to be centered in the center of mass,
/// The density is constant.
/// 
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChInertiaCosseratSimple.png"
///
class ChApi ChInertiaCosseratSimple : public ChInertiaCosserat {
  public:

	ChInertiaCosseratSimple() 
						: rho(1000), A(1), Izz(1), Iyy(1) {};

	ChInertiaCosseratSimple(double density,			///< the density fo the material [kg/m^3], assumed constant
						    double Area,			///< area of the section, [m^2]
							double Iyy_area_moment,	///< second moment of area [m^4] about Y 
							double Izz_area_moment	///< second moment of area [m^4] about Z 
							) 
						: rho(density), A(Area), Izz(Izz_area_moment), Iyy(Iyy_area_moment) {};

    virtual ~ChInertiaCosseratSimple() {}

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel} 
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix).
    /// In this case it is simply a constant diagonal mass matrix with diagonal 
    /// {rho*A,rho*A,rho*A, rho*Iyy+Izz, rho*Iyy, rho*Izz}
    virtual void ComputeInertiaMatrix(ChMatrixNM<double, 6, 6>& M  ///< 6x6 sectional mass matrix values here
                                      ) override;

    /// Compute the values of inertial torque depending on quadratic velocity terms, per unit length,
    /// that is the gyroscopic torque w x [J]w . Quadratic force is null as mass is centered. All terms expressed 
    /// in the material reference, ie. the reference in the centerline of the section.
    virtual void ComputeQuadraticTerms(ChVector<>& mF,   ///< centrifugal term (if any) returned here
                                       ChVector<>& mT,   ///< gyroscopic term  returned here
                                       const ChVector<>& mW    ///< current angular velocity of section, in material frame
                                      ) override;


	/// Compute mass per unit length, ex.SI units [kg/m]
	/// In this case is simply  \f$ \mu = \rho A \f$, given area in [m^2] and with \f$ \rho \f$ density in [kg/m^3].
	virtual double GetMassPerUnitLength() override { return this->rho * this->A; }


	/// Compute the Jxx component of the inertia tensor per unit length,
	/// i.e. the part associated with rotation about the beam direction.
	/// In this case it is \f$ J_{xx} = \rho I_p \f$, where \f$ I_p = I_{zz} + I_{yy} \f$ is the polar moment of area. 
	virtual double GetInertiaJxxPerUnitLength()  { return this->rho * (this->Iyy + this->Izz); }

	/// Compute the Jyy component of the inertia tensor per unit length,
	/// i.e. the part associated with rotation of the section on its Y axis.
	/// Defined as: \f$ J_{yy} = \int_\Omega \rho z^2 d\Omega \f$, with \f$ \rho \f$ density in [kg/m^3].
	/// For uniform density it is  \f$ J_{yy} = \rho I_{yy} \f$, where \f$ I_{yy} =  \int_\Omega z^2 d\Omega \f$ is the second moment of area. 
	virtual double GetInertiaJyyPerUnitLength()  { return this->rho * this->Iyy; }

	/// Compute the Jzz component of the inertia tensor per unit length,
	/// i.e. the part associated with rotation of the section on its Z axis.
	/// Defined as: \f$ J_{zz} = \int_\Omega \rho y^2 d\Omega \f$, with \f$ \rho \f$ density in [kg/m^3].
	/// For uniform density it is  \f$ J_{zz} = \rho I_{zz} \f$, where \f$ I_{zz} =  \int_\Omega y^2 d\Omega \f$ is the second moment of area. 
	virtual double GetInertiaJzzPerUnitLength()  { return this->rho * this->Izz; }

	/// Set the volumetric density, assumed constant in the section. Ex. SI units: [kg/m^3].
	void SetDensity(const double md) {	rho = md; }
	double GetDensity() const { return rho; }

	/// Set the area of section for computing mass properties. Ex. SI units: [m^2]
	void SetArea(const double ma) {	A = ma; }
	double GetArea() const { return A; }

	/// Set the Iyy second moment of area of the beam (for bending about y in xz plane),
	/// defined as \f$ I_{yy} =  \int_\Omega z^2 d\Omega \f$. 
    /// Note: some textbook calls this Iyy as Iy.
	/// Note: it can correspond to the same Iyy that you used for the elasticity, ex. in ChElasticityCosseratSimple.
	/// Ex. SI units: [m^4]
	void SetIyy(double mi) { this->Iyy = mi; }
    double GetIyy() const { return this->Iyy; }

	/// Set the Izz second moment of area of the beam (for bending about z in xy plane),
	/// defined as \f$ I_{zz} =  \int_\Omega y^2 d\Omega \f$. 
    /// Note: some textbook calls this Izz as Iz.
	/// Note: it can correspond to the same Izz that you used for the elasticity, ex. in ChElasticityCosseratSimple.
	/// Ex. SI units: [m^4]
	void SetIzz(double mi) { this->Izz = mi; }
    double GetIzz() const { return this->Izz; }


	/// Shortcut: set Izz, Iyy, Area and density at once, given the y and z widths of the beam assumed
    /// with rectangular shape, and volumetric density. Assuming centered section.
    virtual void SetAsRectangularSection(double width_y, double width_z, double density);

    /// Shortcut: set Izz, Iyy, Area and density at once, given the diameter the beam assumed
    /// with circular shape, and volumetric density. Assuming centered section.
    virtual void SetAsCircularSection(double diameter, double density);

private:
	double rho; // density
	double A;   // Area
	double Izz; // moment of area: m^4
	double Iyy; // moment of area: m^4
};


// for backward compatibility - note it WILL BE DEPRECATED
using ChInertiaCosseratUniformDensity = ChInertiaCosseratSimple;





/// Inertia properties of a beam of Cosserat type, not necessarily of uniform density, 
/// from the following information that allows the center of mass to be
/// offset respect to the beam centerline:
///  - a mass per unit length 
///  - offset of the center of mass Cm along Y Z section axes,
///  - Jyy Jzz Jzy moments of inertia computed in section reference Y Z, not rotated and origin in centerline
/// The polar moment of area is automatically inferred via perpendicular axis theorem.
/// 
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChInertiaCosseratAdvanced.png"
///

class ChApi ChInertiaCosseratAdvanced : public ChInertiaCosserat {
  public:

	ChInertiaCosseratAdvanced() 
						: mu(1), cm_y(0), cm_z(0), Jzz(1), Jyy(1), Jyz(0) {};

	ChInertiaCosseratAdvanced(double mu_density,    ///< mass per unit length [kg/m] 
		                    double c_y,             ///< displacement of center of mass along Y
                            double c_z,             ///< displacement of center of mass along Z					
                            double Jyy_moment,	    ///< moment of inertia per unit length, about Y. Also Jyy= Mm(4,4)
							double Jzz_moment,	    ///< moment of inertia per unit length, about Z. Also Jzz= Mm(5,5)
                            double Jyz_moment       ///< moment of inertia per unit length, about YZ (off diagonal term). Also Jyz= -Mm(4,5) = -Mm(5,4)
							) 
						: mu(mu_density), cm_y(0), cm_z(0), Jzz(Jzz_moment), Jyy(Jyy_moment), Jyz(Jyz_moment) {};

    virtual ~ChInertiaCosseratAdvanced() {}

    /// Compute the 6x6 sectional inertia matrix, as in  {x_momentum,w_momentum}=[Mm]{xvel,wvel} 
    /// The matrix is computed in the material reference (i.e. it is the sectional mass matrix).
    virtual void ComputeInertiaMatrix(ChMatrixNM<double, 6, 6>& M  ///< 6x6 sectional mass matrix values here
                                      ) override;

    /// Compute the values of inertial force & torque depending on quadratic velocity terms,
    /// that is the gyroscopic torque w x [J]w and the centrifugal term (if center of mass is offset). All terms expressed 
    /// in the material reference, ie. the reference in the centerline of the section.
    virtual void ComputeQuadraticTerms(ChVector<>& mF,   ///< centrifugal term (if any) returned here
                                       ChVector<>& mT,   ///< gyroscopic term  returned here
                                       const ChVector<>& mW    ///< current angular velocity of section, in material frame
                                      ) override;

	/// Get mass per unit length, ex.SI units [kg/m]
	virtual double GetMassPerUnitLength() override { return this->mu; }


    /// Set mass c, ex.SI units [kg/m].
    /// Note that for uniform volumetric density \f$ \rho \f$, and area \f$ A \f$, this is also \f$ \mu = \rho A \f$.
    virtual void SetMassPerUnitLength(double mmu) { mu = mmu; }

    /// "mass reference": set the displacement of the center of mass respect to 
    /// the section centerline reference.
    void SetCenterOfMass(double my, double mz) {
        this->cm_y = my;
        this->cm_z = mz;
    }
    double GetCenterOfMassY() {
        return this->cm_y;
    }
    double GetCenterOfMassZ() {
        return this->cm_z;
    }

    /// Set inertia moments, assumed computed in the Y Z unrotated reference
    /// frame of the section at centerline, and defined as: 
    /// \f$ J_{yy} =  \int_\Omega \rho z^2 d\Omega \f$, also Jyy = Mm(4,4) 
    /// \f$ J_{zz} =  \int_\Omega \rho y^2 d\Omega \f$, also Jzz = Mm(5,5) 
    /// \f$ J_{yz} =  \int_\Omega \rho y z  d\Omega \f$, also Jyz = -Mm(4,5) = -Mm(5,4)
    /// Note that for an uniform density, these are also related to second moments of area
    /// as \f$ J_{yy} = \rho I_{yy} \f$,  \f$ J_{zz} = \rho I_{zz} \f$.
    /// Note also that \f$ J_{xy} = J_{xz} = J_{yx} = J_{zx} = 0 \f$ anyway. 
    /// Note also that \f$ J_{xy} \f$ does not need to be input, as automatically computed 
    /// via \f$ J_{xx} = J_{yy} +J_{zz} \f$ for the polar theorem.
    virtual void SetInertiasPerUnitLength(double Jyy_moment, double Jzz_moment, double Jyz_moment);

    /// Get the Jxx component of the inertia per unit length (polar inertia), in the Y Z unrotated reference
    /// frame of the section at centerline. Note: it automatically follows Jxx=Jyy+Jzz for the polar theorem.
    virtual double GetInertiaJxxPerUnitLength()  { return  this->Jyy + this->Jzz; }

    /// Get the Jyy component of the inertia per unit length, in the Y Z unrotated reference
    /// frame of the section at centerline, also Jyy = Mm(4,4)
    virtual double GetInertiaJyyPerUnitLength()  { return  this->Jyy; }

    /// Get the Jzz component of the inertia per unit length, in the Y Z unrotated reference
    /// frame of the section at centerline, also Jzz = Mm(5,5) 
    virtual double GetInertiaJzzPerUnitLength()  { return  this->Jzz; }

    /// Get the Jyz off-diagonal component of the inertia per unit length, in the Y Z unrotated reference
    /// frame of the section at centerline. Also Jyz = -Mm(4,5) = -Mm(5,4)
    virtual double GetInertiaJyzPerUnitLength()  { return  this->Jyz; }


    /// Set inertia moments, per unit length, as assumed computed in the Ym Zm "mass reference"
    /// frame, ie. centered at the center of mass and rotated by phi angle to match the main axes of inertia:
    /// \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$, 
    /// \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega \f$.
    /// Assuming the center of mass is already set.
    virtual void SetMainInertiasInMassReference(double Jmyy, double Jmzz, double phi);

    /// Get inertia moments, per unit length, as assumed computed in the Ym Zm "mass reference" frame, and the rotation phi of that frame,
    /// ie. inertias centered at the center of mass and rotated by phi angle to match the main axes of inertia:
    /// \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$, 
    /// \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega \f$.
    /// Assuming the center of mass is already set.
    virtual void GetMainInertiasInMassReference(double& Jmyy, double& Jmzz, double& phi);

private:
	double mu;   // mass per unit length
    double cm_y; // center of mass offset along Y of section
    double cm_z; // center of mass offset along Z of section
	double Jzz;  
	double Jyy;  
    double Jyz;  
};




/// Inertia properties of a beam of Cosserat type, not necessarily of uniform density, 
/// from the following information that allows the center of mass to be
/// offset respect to the beam centerline:
///  - a mass per unit length 
///  - offset of the center of mass Cm along Y Z section axes,
///  - rotation of axes Y_m Z_m (used for computed Jzz_m Jyy_m) respect to Y Z section axes.
///  - Jyy_m Jzz_m principal moments of inertia computed in reference Y_m Z_m, rotated and with origin in center of mass Cm
/// The polar moment of area is automatically inferred via perpendicular axis theorem.
/// 
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChInertiaCosseratMassref.png"
///

class ChApi ChInertiaCosseratMassref : public ChInertiaCosseratAdvanced {
  public:

	ChInertiaCosseratMassref() 
						: Jzz_m(1), Jyy_m(1), phi(0) {};

	ChInertiaCosseratMassref(double mu_density,    ///< mass per unit length [kg/m] 
		                    double c_y,             ///< displacement of center of mass Cm along Y
                            double c_z,             ///< displacement of center of mass Cm along Z		
                            double phi_massref,     ///< rotation of auxiliary mass reference Ym Zm respect to Y Z reference 
                            double Jyy_massref,	    ///< moment of inertia per unit length, about Ym, with origin in Cm 
							double Jzz_massref 	    ///< moment of inertia per unit length, about Zm, with origin in Cm  
							) 
    {
        this->SetMassPerUnitLength(mu_density);
        this->SetCenterOfMass(c_y, c_z);
        this->SetMainInertiasInMassReference(Jyy_massref, Jzz_massref, phi_massref); 
    };

    virtual ~ChInertiaCosseratMassref() {}

    /// Set inertia moments, assumed computed in the Y Z unrotated reference
    /// frame of the section at centerline, and defined as: 
    /// \f$ J_{yy} =  \int_\Omega \rho z^2 d\Omega \f$, also Jyy = Mm(4,4) 
    /// \f$ J_{zz} =  \int_\Omega \rho y^2 d\Omega \f$, also Jzz = Mm(5,5) 
    /// \f$ J_{yz} =  \int_\Omega \rho y z  d\Omega \f$, also Jyz = -Mm(4,5) = -Mm(5,4)
    /// Note that for an uniform density, these are also related to second moments of area
    /// as \f$ J_{yy} = \rho I_{yy} \f$,  \f$ J_{zz} = \rho I_{zz} \f$.
    /// Note also that \f$ J_{xy} = J_{xz} = J_{yx} = J_{zx} = 0 \f$ anyway. 
    /// Note also that \f$ J_{xy} \f$ does not need to be input, as automatically computed 
    /// via \f$ J_{xx} = J_{yy} +J_{zz} \f$ for the polar theorem.
    virtual void SetInertiasPerUnitLength(double Jyy_moment, double Jzz_moment, double Jyz_moment) override {
        ChInertiaCosseratAdvanced::SetInertiasPerUnitLength(Jyy_moment,Jzz_moment,Jyz_moment);
        this->GetMainInertiasInMassReference(this->Jyy_m, this->Jzz_m, this->phi);
    };

    /// Set inertia moments as assumed computed in the Ym Zm "mass reference"
    /// frame, ie. centered at the center of mass and rotated by phi angle to match the main axes of inertia:
    /// \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$, 
    /// \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega \f$.
    /// Assuming the center of mass is already set.
    virtual void SetMainInertiasInMassReference(double Jyy_massref, double Jzz_massref, double phi_massref) override {
        this->Jyy_m = Jyy_massref;
        this->Jzz_m = Jzz_massref;
        this->phi = phi_massref;
        ChInertiaCosseratAdvanced::SetMainInertiasInMassReference(this->Jyy_m, this->Jzz_m, this->phi);
    };

    /// Get inertia moments as assumed computed in the Ym Zm "mass reference" frame, and the rotation phi of that frame,
    /// ie. inertias centered at the center of mass and rotated by phi angle to match the main axes of inertia:
    /// \f$ Jm_{yy} =  \int_\Omega \rho z_{m}^2 d\Omega \f$, 
    /// \f$ Jm_{zz} =  \int_\Omega \rho y_{m}^2 d\Omega \f$.
    /// Assuming the center of mass is already set.
    virtual void GetMainInertiasInMassReference(double& Jyy_massref, double& Jzz_massref, double& phi_massref) override {
        Jyy_massref = this->Jyy_m;
        Jzz_massref = this->Jzz_m;
        phi_massref = this->phi;
    };

private:
    double phi; // rotation of reference, also main inertia axes ie. Iyz=0
	double Jzz_m;  
	double Jyy_m;  
};







////////////////////////////////////////////////////////////////////////////////////////////////



/// Base class for properties of beam sections of Cosserat type (with shear too)
/// such as ChElementBeamIGA.
/// A beam section can be shared between multiple beams.
/// A beam section contains the models for elasticity, inertia, plasticity, damping, etc.
/// This base model expect that you provide at least the elasticity and inertia models,
/// and optionally you can also add a damping model and a plasticity model.
/// This accomodates most of the constitutive models because there are many
/// combinations of the different types of damping models, elasticity models, etc.,
/// but if you need some extreme customization, you might also inherit your C++ class from this.
/// On the other side, if you need a more immediate way to create sections, look at
/// the special cases called ChBeamSectionCosseratEasyRectangular and ChBeamSectionCosseratEasyCircular.

class ChApi ChBeamSectionCosserat : public ChBeamSection {
  protected:
	ChBeamSectionCosserat() {};

  public:

    ChBeamSectionCosserat(
		std::shared_ptr<ChInertiaCosserat>    minertia,			    ///< inertia model for this section (density, etc)
        std::shared_ptr<ChElasticityCosserat> melasticity,		    ///< elasticity model for this section
		std::shared_ptr<ChPlasticityCosserat> mplasticity = {},		///< plasticity model for this section, if any
		std::shared_ptr<ChDampingCosserat>    mdamping = {}			///< damping model for this section, if any
    );

    virtual ~ChBeamSectionCosserat() {}

    /// Compute the generalized cut force and cut torque, given the actual generalized section strain
    /// expressed as deformation vector e and curvature k, that is: {F,M}=f({e,k}), and
    /// given the actual material state required for plasticity if any (but if mdata=nullptr,
    /// computes only the elastic force).
    /// If there is plasticity, the stress is clamped by automatically performing an implicit return mapping.
    /// In sake of generality, if possible this is the function that should be used by beam finite elements
    /// to compute internal forces, ex.by some Gauss quadrature.
    virtual void ComputeStress(
        ChVector<>& stress_n,        ///< stress (generalized force F), x component = traction along beam
        ChVector<>& stress_m,        ///< stress (generalized torque M), x component = torsion torque along beam
        const ChVector<>& strain_e,  ///< strain (deformation part e): x= elongation, y and z are shear
        const ChVector<>& strain_k,  ///< strain (curvature part k), x= torsion, y and z are line curvatures
        ChBeamMaterialInternalData* mdata_new = nullptr,   ///< updated material internal variables, at this
                                                           ///< point, including {p_strain_e, p_strain_k, p_strain_acc}
        const ChBeamMaterialInternalData* mdata = nullptr  ///< current material internal variables, at this point,
                                                           ///< including {p_strain_e, p_strain_k, p_strain_acc}
    );

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    /// at a given strain state, and at given internal data state (if mdata=nullptr,
    /// computes only the elastic tangent stiffenss, regardless of plasticity).
    virtual void ComputeStiffnessMatrix(
        ChMatrixNM<double, 6, 6>& K, ///< 6x6 stiffness matrix
        const ChVector<>& strain_e,  ///< strain (deformation part): x= elongation, y and z are shear
        const ChVector<>& strain_k,  ///< strain (curvature part), x= torsion, y and z are line curvatures
        const ChBeamMaterialInternalData* mdata = nullptr  ///< material internal variables, at this point, if any,
                                                           ///< including {p_strain_e, p_strain_k, p_strain_acc}
    );

    /// Set the elasticity model for this section.
    /// By default it uses a simple centered linear elastic model, but you can set more complex models.
    void SetElasticity(std::shared_ptr<ChElasticityCosserat> melasticity);

    /// Get the elasticity model for this section.
    /// Use this function to access parameters such as stiffness, Young modulus, etc.
    /// By default it uses a simple centered linear elastic model.
    std::shared_ptr<ChElasticityCosserat> GetElasticity() { return this->elasticity; }

    /// Set the plasticity model for this section.
    /// This is independent from the elasticity model.
    /// Note that by default there is no plasticity model,
    /// so by default plasticity never happens.
    void SetPlasticity(std::shared_ptr<ChPlasticityCosserat> mplasticity);

    /// Get the elasticity model for this section, if any.
    /// Use this function to access parameters such as yeld limit, etc.
    std::shared_ptr<ChPlasticityCosserat> GetPlasticity() { return this->plasticity; }

	/// Set the inertial model for this section, that defines the 
    /// mass per unit length and the inertia tensor of the section.
    void SetInertia(std::shared_ptr<ChInertiaCosserat> minertia);

    /// Get the inertial model for this section, if any.
    /// Use this function to access parameters such as mass per unit length, etc.
    std::shared_ptr<ChInertiaCosserat> GetInertia() { return this->inertia; }


    /// Set the damping model for this section.
    /// By default no damping.
    void SetDamping(std::shared_ptr<ChDampingCosserat> mdamping);

    /// Get the damping model for this section.
    /// By default no damping.
    std::shared_ptr<ChDampingCosserat> GetDamping() { return this->damping; }


  private:
    std::shared_ptr<ChElasticityCosserat> elasticity;
    std::shared_ptr<ChPlasticityCosserat> plasticity;
    std::shared_ptr<ChDampingCosserat> damping;
	std::shared_ptr<ChInertiaCosserat> inertia;
};



/// A simple specialization of ChBeamSectionCosserat if you do not need to define
/// its separate models for elasticity, plasticity, damping and inertia. 
/// Good if you just need the simplest model for a rectangular centered beam. This section automatically
/// creates, initializes and embeds, at construction, these models:
/// - elasticity: ChElasticityCosseratSimple  
/// - inertia:    ChInertiaCosseratSimple
/// - damping:    none   - you can add it later
/// - plasticity: none 
class ChApi ChBeamSectionCosseratEasyRectangular : public ChBeamSectionCosserat {
public:
	ChBeamSectionCosseratEasyRectangular(
		double width_y,			///< width of section in y direction
		double width_z,			///< width of section in z direction
		double E,				///< Young modulus
		double G,				///< shear modulus
		double density			///< volumetric density (ex. in SI units: [kg/m^3])
	);
};


/// A simple specialization of ChBeamSectionCosserat if you do not need to define
/// its separate models for elasticity, plasticity, damping and inertia. 
/// Good if you just need the simplest model for a circular centered beam. This section automatically
/// creates, initializes and embeds, at construction, these models:
/// - elasticity: ChElasticityCosseratSimple  
/// - inertia:    ChInertiaCosseratSimple
/// - damping:    none   - you can add it later
/// - plasticity: none 
class ChApi ChBeamSectionCosseratEasyCircular : public ChBeamSectionCosserat {
public:
	ChBeamSectionCosseratEasyCircular(
		double diameter,		///< diameter of section 
		double E,				///< Young modulus
		double G,				///< shear modulus
		double density			///< volumetric density (ex. in SI units: [kg/m^3])
	);
};




/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
