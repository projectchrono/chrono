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
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero, Radu Serban
// =============================================================================
// ANCF laminated shell element with four nodes.
// =============================================================================

#ifndef CHELEMENTSHELLANCF_H
#define CHELEMENTSHELLANCF_H

#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementShell.h"
#include "chrono_fea/ChNodeFEAxyzD.h"
#include "chrono_fea/ChUtilsFEA.h"
#include "core/ChQuadrature.h"
#include "core/ChShared.h"

namespace chrono {
namespace fea {

// ----------------------------------------------------------------------------
/// Material definition.
/// This class implements material properties for a layer.
class ChApiFea ChMaterialShellANCF : public ChShared {
  public:
    /// Construct an isotropic material.
    ChMaterialShellANCF(double rho,  ///< material density
                        double E,    ///< Young's modulus
                        double nu    ///< Poisson ratio
                        );

    /// Construct a (possibly) orthotropic material.
    ChMaterialShellANCF(double rho,            ///< material density
                        const ChVector<>& E,   ///< elasticity moduli (E_x, E_y, E_z)
                        const ChVector<>& nu,  ///< Poisson ratios (nu_xy, nu_xz, nu_yz)
                        const ChVector<>& G    ///< shear moduli (G_xy, G_xz, G_yz)
                        );

    /// Return the material density.
    double Get_rho() const { return m_rho; }

    /// Return the matrix of elastic coefficients.
    const ChMatrixNM<double, 6, 6>& Get_E_eps() const { return m_E_eps; }

  private:
    /// Calculate the matrix of elastic coefficients.
    void Calc_E_eps(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G);

    double m_rho;                      ///< density
    ChMatrixNM<double, 6, 6> m_E_eps;  ///< matrix of elastic coefficients
};

// ----------------------------------------------------------------------------
/// ANCF laminated shell element with four nodes.
/// This class implements composite material elastic force formulations.
class ChApiFea ChElementShellANCF : public ChElementShell, public ChLoadableUV, public ChLoadableUVW {
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
        ChSharedPtr<ChMaterialShellANCF> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(ChElementShellANCF* element,               ///< containing element
              double thickness,                          ///< layer thickness
              double theta,                              ///< fiber angle
              ChSharedPtr<ChMaterialShellANCF> material  ///< layer material
              );

        double Get_detJ0C() const { return m_detJ0C; }
        const ChMatrixNM<double, 6, 6>& Get_T0() const { return m_T0; }

        /// Initial setup for this layer: calculate T0 and detJ0 at the element center.
        void SetupInitial();

        ChElementShellANCF* m_element;                ///< containing ANCF shell element
        ChSharedPtr<ChMaterialShellANCF> m_material;  ///< layer material
        double m_thickness;                           ///< layer thickness
        double m_theta;                               ///< fiber angle

        double m_detJ0C;
        ChMatrixNM<double, 6, 6> m_T0;

        friend class ChElementShellANCF;
        friend class MyForce;
    };

    /// Get the number of nodes used by this element.
    virtual int GetNnodes() override { return 4; }

    /// Get the number of coordinates of the node positions in space.
    /// Note this is not the coordinates of the field, use GetNdofs() instead.
    virtual int GetNcoords() override { return 4 * 6; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 4 * 6; }

    /// Specify the nodes of this element.
    void SetNodes(ChSharedPtr<ChNodeFEAxyzD> nodeA,
                  ChSharedPtr<ChNodeFEAxyzD> nodeB,
                  ChSharedPtr<ChNodeFEAxyzD> nodeC,
                  ChSharedPtr<ChNodeFEAxyzD> nodeD);

    /// Specify the element dimensions.
    void SetDimensions(double lenX, double lenY) {
        m_lenX = lenX;
        m_lenY = lenY;
    }

    /// Access the n-th node of this element.
    virtual ChSharedPtr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeA() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeB() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeC() const { return m_nodes[2]; }

    /// Get a handle to the fourth node of this element.
    ChSharedPtr<ChNodeFEAxyzD> GetNodeD() const { return m_nodes[3]; }

    /// Add a layer.
    void AddLayer(double thickness,                          ///< layer thickness
                  double theta,                              ///< fiber angle (radians)
                  ChSharedPtr<ChMaterialShellANCF> material  ///< layer material
                  );

    /// Get the number of layers.
    size_t GetNumLayers() const { return m_numLayers; }

    /// Get a handle to the specified layer.
    const Layer& GetLayer(size_t i) const { return m_layers[i]; }

    /// Set the storage of the five alpha parameters for EAS (max no. of layers 7)
    void SetStockAlpha(const ChMatrixNM<double, 35, 1>& a) { m_StockAlpha_EAS = a; }
    /// Set all the alpha parameters for EAS
    const ChMatrixNM<double, 35, 1>& GetStockAlpha() const { return m_StockAlpha_EAS; }
    /// Set Jacobian of EAS
    void SetStockJac(const ChMatrixNM<double, 24, 24>& a) { m_stock_jac_EAS = a; }
    /// Set Jacobian
    void SetStockKTE(const ChMatrixNM<double, 24, 24>& a) { m_stock_KTE = a; }
    /// Set the step size used in calculating the structural damping coefficient.
    void Setdt(double a) { m_dt = a; }
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

  private:
    enum JacobianType { ANALYTICAL, NUMERICAL };

    std::vector<ChSharedPtr<ChNodeFEAxyzD> > m_nodes;  ///< element nodes
    std::vector<Layer> m_layers;                       ///< element layers
    size_t m_numLayers;                                ///< number of layers for this element
    double m_lenX;                                     ///< element length in X direction
    double m_lenY;                                     ///< element length in Y direction
    double m_thickness;                                ///< total element thickness
    std::vector<double> m_GaussZ;                      ///< layer separation z values (scaled to [-1,1])
    double m_GaussScaling;                             ///< scaling factor due to change of integration intervals
    double m_Alpha;                                    ///< structural damping
    ChMatrixNM<double, 24, 24> m_StiffnessMatrix;      ///< stiffness matrix
    ChMatrixNM<double, 24, 24> m_MassMatrix;           ///< mass matrix
    ChMatrixNM<double, 24, 24> m_stock_jac_EAS;        ///< EAS per element
    ChMatrixNM<double, 24, 24> m_stock_KTE;            ///< Analytical Jacobian
    ChMatrixNM<double, 8, 3> m_d0;                     ///< initial nodal coordinates
    ChMatrixNM<double, 24, 1> m_GravForce;             ///< Gravity Force
    ChMatrixNM<double, 35, 1> m_StockAlpha_EAS;        ///< StockAlpha(5*7,1): Max #Layer is 7
    JacobianType m_flag_HE;                            ///< Jacobian evaluation type (analytical or numerical)
    double m_dt;                                       ///< time step used in calculating structural damping coefficient
    bool m_gravity_on;                                 ///< flag indicating whether or not gravity is included

    // Interface to ChElementBase base class
    // -------------------------------------

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor.
    /// Optionally, also superimposes global damping matrix R, scaled by Rfactor, and global
    /// mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

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
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override;

    virtual void EvaluateSectionFrame(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override;

    virtual void EvaluateSectionPoint(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point) override;

    // Internal computations
    // ---------------------

    /// Compute the STIFFNESS MATRIX of the element.
    /// K = integral( .... ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeStiffnessMatrix();

    /// Compute the MASS MATRIX of the element.
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();

    /// Compute the gravitational forces.
    void ComputeGravityForce(const ChVector<>& g_acc);

    // [ANS] Shape function for Assumed Naturals Strain (Interpolation of strain and strainD in a thickness direction)
    void shapefunction_ANS_BilinearShell(ChMatrixNM<double, 1, 4>& S_ANS, double x, double y);

    // [ANS] Calculation of ANS strain and strainD.
    void AssumedNaturalStrain_BilinearShell(const ChMatrixNM<double, 8, 3>& d,
                                            ChMatrixNM<double, 8, 1>& strain_ans,
                                            ChMatrixNM<double, 8, 24>& strainD_ans);

    // [EAS] Basis function of M for Enhanced Assumed Strain.
    void Basis_M(ChMatrixNM<double, 6, 5>& M, double x, double y, double z);

    // [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    void T0DetJElementCenterForEAS(ChMatrixNM<double, 6, 6>& T0, double& detJ0C, double& theta);

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

    // Helper functions
    // ----------------

    /// Numerial inverse for a 5x5 matrix.
    static void Inverse55_Numerical(ChMatrixNM<double, 5, 5>& a, int n);

    /// Analytical inverse for a 5x5 matrix.
    static void Inverse55_Analytical(ChMatrixNM<double, 5, 5>& A, ChMatrixNM<double, 5, 5>& B);

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() { return 4 * 6; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() { return 4 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) { return m_nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) { return 6; }

    virtual void EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) override;

    /// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) override;

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
};

}  // end of namespace fea
}  // end of namespace chrono

#endif
