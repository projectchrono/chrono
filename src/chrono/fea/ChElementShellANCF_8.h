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
// ANCF laminated shell element with eight nodes: High-Order.
// Element 3833 of paper: 'Analysis of higher-order quadrilateral plate elements
// based on the absolute nodal coordinate formulation for three-dimensional
// elasticity'
// H.C.J. Ebel, M.K.Matikainen, V.V.T. Hurskainen, A.M.Mikkola, Multibody System
// Dynamics, To be published, 2017
//
// Note: this element uses the ChMaterailShellANCF class for the material
// properties of a layer.
// =============================================================================

#ifndef CHELEMENTSHELLANCF8_H
#define CHELEMENTSHELLANCF8_H

#include <vector>

#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChMaterialShellANCF.h"
#include "chrono/fea/ChNodeFEAxyzDD.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// ANCF laminated shell element with eight nodes.
/// This class implements composite material elastic force formulations.
///
/// The node numbering is in ccw fashion as in the following scheme:
///         v
///         ^
/// D o-----G-----o C
///   |     |     |
/// --H-----+-----F-> u
///   |     |     |
/// A o-----E-----o B
///
class ChApi ChElementShellANCF_8 : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
  public:
    ChElementShellANCF_8();
    ~ChElementShellANCF_8() {}

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
        Layer(ChElementShellANCF_8* element,                 ///< containing element
              double thickness,                              ///< layer thickness
              double theta,                                  ///< fiber angle
              std::shared_ptr<ChMaterialShellANCF> material  ///< layer material
        );

        double Get_detJ0C() const { return m_detJ0C; }
        const ChMatrixNM<double, 6, 6>& Get_T0() const { return m_T0; }

        /// Initial setup for this layer: calculate T0 and detJ0 at the element center.
        void SetupInitial();

        ChElementShellANCF_8* m_element;                  ///< containing ANCF shell element
        std::shared_ptr<ChMaterialShellANCF> m_material;  ///< layer material
        double m_thickness;                               ///< layer thickness
        double m_theta;                                   ///< fiber angle

        double m_detJ0C;
        ChMatrixNM<double, 6, 6> m_T0;

        friend class ChElementShellANCF_8;
        friend class MyForce_8;
        friend class MyJacobian_8;
    };

    /// Get the number of nodes used by this element.
    virtual int GetNnodes() override { return 8; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 8 * 9; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return 9; }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyzDD> nodeA,
                  std::shared_ptr<ChNodeFEAxyzDD> nodeB,
                  std::shared_ptr<ChNodeFEAxyzDD> nodeC,
                  std::shared_ptr<ChNodeFEAxyzDD> nodeD,
                  std::shared_ptr<ChNodeFEAxyzDD> nodeE,
                  std::shared_ptr<ChNodeFEAxyzDD> nodeF,
                  std::shared_ptr<ChNodeFEAxyzDD> nodeG,
                  std::shared_ptr<ChNodeFEAxyzDD> nodeH);

    /// Specify the element dimensions.
    void SetDimensions(double lenX, double lenY) {
        m_lenX = lenX;
        m_lenY = lenY;
    }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeA() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeB() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeC() const { return m_nodes[2]; }

    /// Get a handle to the fourth node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeD() const { return m_nodes[3]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeE() const { return m_nodes[4]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeF() const { return m_nodes[5]; }

    /// Get a handle to the third node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeG() const { return m_nodes[6]; }

    /// Get a handle to the fourth node of this element.
    std::shared_ptr<ChNodeFEAxyzDD> GetNodeH() const { return m_nodes[7]; }
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
    /// NOTE! actually N should be a 3row, 72 column sparse matrix,
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the s1 through s24 values in a 1 row, 24 columns matrix!
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);

    /// Fills the Nx shape function derivative matrix with respect to X.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 24 columns matrix!
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);

    /// Fills the Ny shape function derivative matrix with respect to Y.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 24 columns matrix!
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);

    /// Fills the Nz shape function derivative matrix with respect to Z.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 24 columns matrix!
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);
    /// Return a vector with three strain components
    ChVector<> EvaluateSectionStrains();

  private:
    std::vector<std::shared_ptr<ChNodeFEAxyzDD> > m_nodes;  ///< element nodes
    std::vector<Layer> m_layers;                            ///< element layers
    size_t m_numLayers;                                     ///< number of layers for this element
    double m_lenX;                                          ///< element length in X direction
    double m_lenY;                                          ///< element length in Y direction
    double m_thickness;                                     ///< total element thickness
    std::vector<double> m_GaussZ;                           ///< layer separation z values (scaled to [-1,1])
    double m_GaussScaling;                                  ///< scaling factor due to change of integration intervals
    double m_Alpha;                                         ///< structural damping
    bool m_gravity_on;                                      ///< enable/disable gravity calculation
    ChMatrixNM<double, 72, 1> m_GravForce;                  ///< Gravity Force
    ChMatrixNM<double, 72, 72> m_MassMatrix;                ///< mass matrix
    ChMatrixNM<double, 72, 72> m_JacobianMatrix;            ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])
    ChMatrixNM<double, 24, 3> m_d0;                         ///< initial nodal coordinates
    ChMatrixNM<double, 24, 24> m_d0d0T;                     ///< matrix m_d0 * m_d0^T
    ChMatrixNM<double, 24, 3> m_d;                          ///< current nodal coordinates
    ChMatrixNM<double, 24, 24> m_ddT;                       ///< matrix m_d * m_d^T
    ChMatrixNM<double, 72, 1> m_d_dt;                       ///< current nodal velocities

  public:
    // Interface to ChElementBase base class
    // -------------------------------------

    // Fill the DD vector (column matrix) with the current field values at the
    // nodes of the element, with proper ordering.
    // If the D vector has not the size of this->GetNdofs(), it will be resized.
    //  {x_a y_a z_a Dx_a Dy_a Dz_a DDx_a DDy_a DDz_a
    //  {x_b y_b z_b Dx_b Dy_b Dz_b DDx_b DDy_b DDz_b
    //  {x_c y_c z_c Dx_c Dy_c Dz_c DDx_c DDy_c DDz_c
    //  {x_d y_d z_d Dx_d Dy_d Dz_d DDx_d DDy_d DDz_d
    //  {x_e y_e z_e Dx_e Dy_e Dz_e DDx_e DDy_e DDz_e
    //  {x_f y_f z_f Dx_f Dy_f Dz_f DDx_f DDy_f DDz_f
    //  {x_g y_g z_g Dx_g Dy_g Dz_g DDx_g DDy_g DDz_g
    //  {x_h y_h z_h Dx_h Dy_h Dz_h DDx_h DDy_h DDz_h}

    virtual void GetStateBlock(ChMatrixDynamic<>& mDD) override;

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

    virtual void EvaluateSectionFrame(const double u,
                                      const double v,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override;

    virtual void EvaluateSectionPoint(const double u,
                                      const double v,
                                      ChVector<>& point) override;

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
                      ChMatrixNM<double, 1, 24>& Nx,
                      ChMatrixNM<double, 1, 24>& Ny,
                      ChMatrixNM<double, 1, 24>& Nz,
                      ChMatrixNM<double, 1, 3>& Nx_d0,
                      ChMatrixNM<double, 1, 3>& Ny_d0,
                      ChMatrixNM<double, 1, 3>& Nz_d0);

    // Calculate the current 24x3 matrix of nodal coordinates.
    void CalcCoordMatrix(ChMatrixNM<double, 24, 3>& d);

    // Calculate the current 72x1 matrix of nodal coordinate derivatives.
    void CalcCoordDerivMatrix(ChMatrixNM<double, 72, 1>& dt);

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 8 * 9; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 8 * 9; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mDD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mDD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 9; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 8; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 9; }

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

    friend class MyMass_8;
    friend class MyGravity_8;
    friend class MyForce_8;
    friend class MyJacobian_8;
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
