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
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero, Radu Serban
// =============================================================================
// ANCF laminated shell element with four nodes.
// =============================================================================

#ifndef CHELEMENTSHELLANCF_H
#define CHELEMENTSHELLANCF_H

#include <vector>

#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChMaterialShellANCF.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// ANCF laminated shell element with four nodes.
/// This class implements composite material elastic force formulations.
///
/// The node numbering is in ccw fashion as in the following scheme:
///         v
///         ^
/// D o-----+-----o C
///   |     |     |
/// --+-----+-----+-> u
///   |     |     |
/// A o-----+-----o B
///
class ChApi ChElementShellANCF : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
  public:
    ChElementShellANCF();
    ~ChElementShellANCF() {}

    /// Definition of a layer
    class Layer {
      public:
        /// Return the layer thickness.
        double Get_thickness() const { return m_thickness; }

        /// Return the fiber angle.
        double Get_theta() const { return m_theta; }

        /// Return the layer material.
        std::shared_ptr<ChMaterialShellANCF> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(ChElementShellANCF* element,                   ///< containing element
              double thickness,                              ///< layer thickness
              double theta,                                  ///< fiber angle
              std::shared_ptr<ChMaterialShellANCF> material  ///< layer material
        );

        double Get_detJ0C() const { return m_detJ0C; }
        const ChMatrixNM<double, 6, 6>& Get_T0() const { return m_T0; }

        /// Initial setup for this layer: calculate T0 and detJ0 at the element center.
        void SetupInitial();

        ChElementShellANCF* m_element;                    ///< containing ANCF shell element
        std::shared_ptr<ChMaterialShellANCF> m_material;  ///< layer material
        double m_thickness;                               ///< layer thickness
        double m_theta;                                   ///< fiber angle

        double m_detJ0C;
        ChMatrixNM<double, 6, 6> m_T0;

        friend class ChElementShellANCF;
        friend class MyForce;
        friend class MyJacobian;
    };

    /// Get the number of nodes used by this element.
    virtual int GetNnodes() override { return 4; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 4 * 6; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return 6; }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyzD> nodeA,
                  std::shared_ptr<ChNodeFEAxyzD> nodeB,
                  std::shared_ptr<ChNodeFEAxyzD> nodeC,
                  std::shared_ptr<ChNodeFEAxyzD> nodeD);

    /// Specify the element dimensions.
    void SetDimensions(double lenX, double lenY) {
        m_lenX = lenX;
        m_lenY = lenY;
    }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNodeA() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNodeB() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNodeC() const { return m_nodes[2]; }

    /// Get a handle to the fourth node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNodeD() const { return m_nodes[3]; }

    /// Add a layer.
    void AddLayer(double thickness,                              ///< layer thickness
                  double theta,                                  ///< fiber angle (radians)
                  std::shared_ptr<ChMaterialShellANCF> material  ///< layer material
    );

    /// Get the number of layers.
    size_t GetNumLayers() const { return m_numLayers; }

    /// Get a handle to the specified layer.
    const Layer& GetLayer(size_t i) const { return m_layers[i]; }

    /// Turn gravity on/off.
    void SetGravityOn(bool val) { m_gravity_on = val; }

    /// Set the structural damping.
    void SetAlphaDamp(double a) { m_Alpha = a; }

    /// Get the element length in the X direction.
    double GetLengthX() const { return m_lenX; }
    /// Get the element length in the Y direction.
    double GetLengthY() const { return m_lenY; }
    /// Get the total thickness of the shell element.
    double GetThickness() { return m_thickness; }

    // Shape functions
    // ---------------

    /// Fills the N shape function matrix.
    /// NOTE! actually N should be a 3row, 24 column sparse matrix,
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the s1 through s8 values in a 1 row, 8 columns matrix!
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);

    /// Fills the Nx shape function derivative matrix with respect to X.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 8 columns matrix!
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);

    /// Fills the Ny shape function derivative matrix with respect to Y.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 8 columns matrix!
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);

    /// Fills the Nz shape function derivative matrix with respect to Z.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 8 columns matrix!
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);
    /// Return a vector with three strain components
    ChVector<> EvaluateSectionStrains();
    void EvaluateDeflection(double& defVec);

  private:
    std::vector<std::shared_ptr<ChNodeFEAxyzD> > m_nodes;  ///< element nodes
    std::vector<Layer> m_layers;                           ///< element layers
    size_t m_numLayers;                                    ///< number of layers for this element
    double m_lenX;                                         ///< element length in X direction
    double m_lenY;                                         ///< element length in Y direction
    double m_thickness;                                    ///< total element thickness
    std::vector<double> m_GaussZ;                          ///< layer separation z values (scaled to [-1,1])
    double m_GaussScaling;                                 ///< scaling factor due to change of integration intervals
    double m_Alpha;                                        ///< structural damping
    bool m_gravity_on;                                     ///< enable/disable gravity calculation
    ChMatrixNM<double, 24, 1> m_GravForce;                 ///< Gravity Force
    ChMatrixNM<double, 24, 24> m_MassMatrix;               ///< mass matrix
    ChMatrixNM<double, 24, 24> m_JacobianMatrix;           ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])
    ChMatrixNM<double, 8, 3> m_d0;                         ///< initial nodal coordinates
    ChMatrixNM<double, 8, 8> m_d0d0T;                      ///< matrix m_d0 * m_d0^T
    ChMatrixNM<double, 8, 3> m_d;                          ///< current nodal coordinates
    ChMatrixNM<double, 8, 8> m_ddT;                        ///< matrix m_d * m_d^T
    ChMatrixNM<double, 24, 1> m_d_dt;                      ///< current nodal velocities
    ChMatrixNM<double, 8, 1> m_strainANS;                  ///< ANS strain
    ChMatrixNM<double, 8, 24> m_strainANS_D;               ///< ANS strain derivatives
    std::vector<ChMatrixNM<double, 5, 1> > m_alphaEAS;     ///< EAS parameters (5 per layer)
    std::vector<ChMatrixNM<double, 5, 5> > m_KalphaEAS;    ///< EAS Jacobians (a 5x5 matrix per layer)
    static const double m_toleranceEAS;                    ///< tolerance for nonlinear EAS solver (on residual)
    static const int m_maxIterationsEAS;                   ///< maximum number of nonlinear EAS iterations

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

    // Interface to ChElementShell base class
    // --------------------------------------

    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override;

    virtual void EvaluateSectionFrame(const double u, const double v, ChVector<>& point, ChQuaternion<>& rot) override;

    virtual void EvaluateSectionPoint(const double u, const double v, ChVector<>& point) override;

    // Internal computations
    // ---------------------

    /// Compute Jacobians of the internal forces.
    /// This function calculates a linear combination of the stiffness (K) and damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coefficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobians(double Kfactor, double Rfactor);

    /// Compute the mass matrix of the element.
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();

    /// Compute the gravitational forces.
    void ComputeGravityForce(const ChVector<>& g_acc);

    // [ANS] Shape function for Assumed Naturals Strain (Interpolation of strain and strainD in a thickness direction)
    void ShapeFunctionANSbilinearShell(ChMatrixNM<double, 1, 4>& S_ANS, double x, double y);

    // [ANS] Calculate the ANS strain and strain derivatives.
    void CalcStrainANSbilinearShell();

    // [EAS] Basis function of M for Enhanced Assumed Strain.
    void Basis_M(ChMatrixNM<double, 6, 5>& M, double x, double y, double z);

    // Calculate the determinant of the initial configuration position vector gradient matrix
    // at the specified point.
    double Calc_detJ0(double x, double y, double z);

    // Same as above, but also return the dense shape function vector derivatives.
    double Calc_detJ0(double x,
                      double y,
                      double z,
                      ChMatrixNM<double, 1, 8>& Nx,
                      ChMatrixNM<double, 1, 8>& Ny,
                      ChMatrixNM<double, 1, 8>& Nz,
                      ChMatrixNM<double, 1, 3>& Nx_d0,
                      ChMatrixNM<double, 1, 3>& Ny_d0,
                      ChMatrixNM<double, 1, 3>& Nz_d0);

    // Calculate the current 8x3 matrix of nodal coordinates.
    void CalcCoordMatrix(ChMatrixNM<double, 8, 3>& d);

    // Calculate the current 24x1 matrix of nodal coordinate derivatives.
    void CalcCoordDerivMatrix(ChMatrixNM<double, 24, 1>& dt);

    // Helper functions
    // ----------------

    /// Numerical inverse for a 5x5 matrix.
    static void Inverse55_Numerical(ChMatrixNM<double, 5, 5>& a, int n);

    /// Analytical inverse for a 5x5 matrix.
    static void Inverse55_Analytical(ChMatrixNM<double, 5, 5>& A, ChMatrixNM<double, 5, 5>& B);

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 4 * 6; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 4 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    virtual void EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) override;

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
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

    /// Gets the normal to the surface at the parametric coordinate U,V.
    /// Each coordinate ranging in -1..+1.
    virtual ChVector<> ComputeNormal(const double U, const double V) override;

    friend class MyMass;
    friend class MyGravity;
    friend class MyForce;
    friend class MyJacobian;
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
