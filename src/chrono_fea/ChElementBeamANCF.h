// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Antonio Recuero
// =============================================================================
// ANCF beam element with 3 nodes. Description of this element and its internal
// forces may be found in Nachbagauer, Gruber, Gerstmayr, "Structural and Continuum
// Mechanics Approaches for a 3D Shear Deformable ANCF Beam Finite Element:
// Application to Static and Linearized Dynamic Examples", Journal of Computational
//  and Nonlinear Dynamics, 2013, April 2013, Vol. 8, 021004.
// =============================================================================

#ifndef CHELEMENTBEAMANCF_H
#define CHELEMENTBEAMANCF_H

#include <vector>

#include "chrono/core/ChQuadrature.h"
#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementBeam.h"
#include "chrono_fea/ChNodeFEAxyzDD.h"
#include "chrono_fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{
// ----------------------------------------------------------------------------
/// Material definition.
/// This class implements material properties for a layer.
class ChApiFea ChMaterialBeamANCF {
  public:
    /// Construct an isotropic material.
    ChMaterialBeamANCF(double rho,        ///< material density
                       double E,          ///< Young's modulus
                       double nu,         ///< Poisson ratio
                       const double& k1,  // Shear correction factor along beam local y axis
                       const double& k2   // Shear correction factor along beam local z axis
                       );

    /// Construct a (possibly) orthotropic material.
    ChMaterialBeamANCF(double rho,            ///< material density
                       const ChVector<>& E,   ///< elasticity moduli (E_x, E_y, E_z)
                       const ChVector<>& nu,  ///< Poisson ratios (nu_xy, nu_xz, nu_yz)
                       const ChVector<>& G,   ///< shear moduli (G_xy, G_xz, G_yz)
                       const double& k1,      // Shear correction factor along beam local y axis
                       const double& k2       // Shear correction factor along beam local z axis
                       );

    /// Return the material density.
    double Get_rho() const { return m_rho; }

    /// Return the matrix of elastic coefficients: Diagonal terms.
    const ChMatrixNM<double, 6, 6>& Get_E_eps() const { return m_E_eps; }

    /// Return the matrix of elastic coefficients: Coupling terms.
    const ChMatrixNM<double, 6, 6>& Get_E_eps_Nu() const { return m_E_eps_Nu; }

  private:
    /// Calculate the matrix of elastic coefficients: k1 and k2 are Timoshenko shear correction factors.
    void Calc_E_eps(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G, double k1, double k2);

    /// Calculate the matrix of elastic coefficients.
    void Calc_E_eps_Nu(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G);

    double m_rho;                         ///< density
    ChMatrixNM<double, 6, 6> m_E_eps;     ///< matrix of elastic coefficients
    ChMatrixNM<double, 6, 6> m_E_eps_Nu;  ///< matrix of elastic coefficients
};

// ----------------------------------------------------------------------------
/// ANCF beam element with 3 nodes.
/// This class implements a continuum-based elastic force formulation.
///
/// The node numbering, as follows:
///               v
///               ^
///               |
/// A o-----+-----o-----+-----o B -> u
///              /C
///             w
/// where C is the third and central node.

class ChApiFea ChElementBeamANCF : public ChElementBeam, public ChLoadableU, public ChLoadableUVW {
  public:
    ChElementBeamANCF();
    ~ChElementBeamANCF() {}

    /// Get the number of nodes used by this element.
    virtual int GetNnodes() override { return 3; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 3 * 9; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return 9; }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyzDD> nodeA,   //
                  std::shared_ptr<ChNodeFEAxyzDD> nodeB,   //
                  std::shared_ptr<ChNodeFEAxyzDD> nodeC);  //

    /// Specify the element dimensions.
    void SetDimensions(double lenX, double beam_h, double beam_w) {
        m_lenX = lenX;
        m_thicknessY = beam_h;
        m_thicknessZ = beam_w;
    }

    /// Specify the element material.
    void SetMaterial(std::shared_ptr<ChMaterialBeamANCF> beam_mat) { m_material = beam_mat; }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeA() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeB() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeC() const { return m_nodes[2]; }

    /// Return the material.
    std::shared_ptr<ChMaterialBeamANCF> GetMaterial() const { return m_material; }

    /// Turn gravity on/off.
    void SetGravityOn(bool val) { m_gravity_on = val; }

    /// Set the structural damping.
    void SetAlphaDamp(double a) { m_Alpha = a; }

    /// Get the element length in the X direction.
    double GetLengthX() const { return m_lenX; }

    /// Get the total thickness of the shell element.
    double GetThicknessY() { return m_thicknessY; }

    /// Get the total thickness of the shell element.
    double GetThicknessZ() { return m_thicknessZ; }
    // Shape functions
    // ---------------

    /// Fills the N shape function matrix.
    /// NOTE! actually N should be a 3row, 27 column sparse matrix,
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the s1 through s9 values in a 1 row, 9 columns matrix.
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);

    /// Fills the Nx shape function derivative matrix with respect to X.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 9 columns matrix.
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);

    /// Fills the Ny shape function derivative matrix with respect to Y.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 9 columns matrix.
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);

    /// Fills the Nz shape function derivative matrix with respect to Z.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 9 columns matrix.
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);

    /// Return a vector with three strain components.
    ChVector<> EvaluateBeamSectionStrains();

    /// Poisson effect selection.
    enum StrainFormulation {
        CMPoisson,   ///< Continuum-Mechanics formulation, including Poisson effects
        CMNoPoisson  ///< Continuum-Mechanics formulation, disregarding Poisson effects
    };

  private:
    std::vector<std::shared_ptr<ChNodeFEAxyzDD> > m_nodes;  ///< element nodes
    double m_lenX;                                          ///< total element length
    double m_thicknessY;                                    ///< total element thickness along Y
    double m_thicknessZ;                                    ///< total element thickness along Z
    double m_GaussScaling;                                  ///< Gauss scaling for beam
    double m_Alpha;                                         ///< structural damping
    bool m_gravity_on;                                      ///< enable/disable gravity calculation
    ChMatrixNM<double, 27, 1> m_GravForce;                  ///< Gravity Force
    ChMatrixNM<double, 27, 27> m_MassMatrix;                ///< mass matrix
    ChMatrixNM<double, 27, 27> m_JacobianMatrix;            ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])
    ChMatrixNM<double, 9, 3> m_d0;                          ///< initial nodal coordinates
    ChMatrixNM<double, 9, 9> m_d0d0T;                       ///< matrix m_d0 * m_d0^T
    ChMatrixNM<double, 9, 3> m_d;                           ///< current nodal coordinates
    ChMatrixNM<double, 9, 9> m_ddT;                         ///< matrix m_d * m_d^T
    ChMatrixNM<double, 27, 1> m_d_dt;                       ///< current nodal velocities
    std::shared_ptr<ChMaterialBeamANCF> m_material;         ///< beam material
    StrainFormulation m_strain_form;                        ///< Strain formulation

  public:
    // Interface to ChElementBase base class
    // -------------------------------------

    // Fill the D vector (column matrix) with the current field values at the
    // nodes of the element, with proper ordering.
    // If the D vector has not the size of this->GetNdofs(), it will be resized.
    //  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    // Set H as a linear combination of M, K, and R.
    //   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R],
    // where [M] is the mass matrix, [K] is the stiffness matrix, and [R] is the damping matrix.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    // Set M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrix<>& M) override;

    /// Add contribution of element inertia to total nodal masses
    virtual void ComputeNodalMass() override;

    /// Computes the internal forces.
    /// (E.g. the actual position of nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    /// Initial setup.
    /// This is used mostly to precompute matrices that do not change during the simulation,
    /// such as the local stiffness of each element (if any), the mass, etc.
    virtual void SetupInitial(ChSystem* system) override;

    /// Update the state of this element.
    virtual void Update() override;

    // Interface to ChElementBeam base class
    // --------------------------------------

    // void EvaluateSectionPoint(const double u, const ChMatrix<>& displ, ChVector<>& point); // Not needed?

    // Dummy method definitions.
    virtual void EvaluateSectionStrain(const double,
                                       const chrono::ChMatrix<double>&,
                                       chrono::ChVector<double>&) override {}

    virtual void EvaluateSectionForceTorque(const double,
                                            const chrono::ChMatrix<double>&,
                                            chrono::ChVector<double>&,
                                            chrono::ChVector<double>&) override {}

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta,
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override {}

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are corotated (expressed in world reference)
    virtual void EvaluateSectionFrame(const double eta,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override {}
    // Internal computations
    // ---------------------

    /// Compute Jacobians of the internal forces.
    /// This function calculates a linear combination of the stiffness (K) and damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coeficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobians(double Kfactor, double Rfactor);

    /// Compute the mass matrix of the element.
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();

    /// Compute the gravitational forces.
    void ComputeGravityForce(const ChVector<>& g_acc);

    // Calculate the determinant of the initial configuration position vector gradient matrix
    // at the specified point.
    double Calc_detJ0(double x, double y, double z);

    // Same as above, but also return the dense shape function vector derivatives.
    double Calc_detJ0(double x,
                      double y,
                      double z,
                      ChMatrixNM<double, 1, 9>& Nx,
                      ChMatrixNM<double, 1, 9>& Ny,
                      ChMatrixNM<double, 1, 9>& Nz,
                      ChMatrixNM<double, 1, 3>& Nx_d0,
                      ChMatrixNM<double, 1, 3>& Ny_d0,
                      ChMatrixNM<double, 1, 3>& Nz_d0);

    // Calculate the current 9x3 matrix of nodal coordinates.
    void CalcCoordMatrix(ChMatrixNM<double, 9, 3>& d);

    // Calculate the current 27x1 matrix of nodal coordinate derivatives.
    void CalcCoordDerivMatrix(ChMatrixNM<double, 27, 1>& dt);

    /// Set the strain formulation.
    void SetStrainFormulation(StrainFormulation model) { m_strain_form = model; }

    /// Get the strain formulation.
    StrainFormulation GetStrainFormulation() const { return m_strain_form; }

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 3 * 9; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 3 * 9; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 9; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 3; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 9; }

    void EvaluateSectionVelNorm(double U, ChVector<>& Result);

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate
                           ) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity.
    /// Density is mass per unit surface.
    virtual double GetDensity() override;

    /// Gets the tangent to the centerline at the parametric coordinate U.
    /// Each coordinate ranging in -1..+1.
    ChVector<> ComputeTangent(const double U);

    friend class MyMassBeam;
    friend class MyGravityBeam;
    friend class MyForceBeam;
    friend class MyForceBeam_Nu;
    friend class MyJacobianBeam;
    friend class MyJacobianBeam_Nu;
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
