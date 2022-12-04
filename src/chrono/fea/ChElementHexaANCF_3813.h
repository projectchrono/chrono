// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
// =============================================================================
// Authors: Bryan Peterson, Antonio Recuero, Radu Serban
// =============================================================================
// Hexahedronal element with 8 nodes (with EAS)
// =============================================================================

#ifndef CH_ELEMENT_HEXA_ANCF_3813_H
#define CH_ELEMENT_HEXA_ANCF_3813_H

#include "chrono/fea/ChElementANCF.h"
#include "chrono/fea/ChElementHexahedron.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono/physics/ChLoadable.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Hexahedronal solid element with 8 nodes (with EAS).
/// While technically not an ANCF element, the name is justified because the implementation can use the same ANCF
/// machinery.
class ChApi ChElementHexaANCF_3813 : public ChElementANCF,
                                     public ChElementHexahedron,
                                     public ChElementGeneric,
                                     public ChLoadableUVW {
  public:
    using ShapeVector = ChMatrixNM<double, 1, 8>;

    ChElementHexaANCF_3813();
    ~ChElementHexaANCF_3813() {}

    /// Get number of nodes of this element
    virtual int GetNnodes() override { return 8; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual int GetNdofs() override { return 8 * 3; }

    /// Get the number of active coordinates in the field used by the referenced nodes.
    virtual int GetNdofs_active() override { return m_element_dof; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override { return m_nodes[n]->GetNdofX(); }

    /// Get the number of active coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs_active(int n) override { return m_nodes[n]->GetNdofX_active(); }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return m_nodes[n]; }

    /// Return the specified hexahedron node (0 <= n <= 7).
    virtual std::shared_ptr<ChNodeFEAxyz> GetHexahedronNode(int n) override { return m_nodes[n]; }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                  std::shared_ptr<ChNodeFEAxyz> nodeB,
                  std::shared_ptr<ChNodeFEAxyz> nodeC,
                  std::shared_ptr<ChNodeFEAxyz> nodeD,
                  std::shared_ptr<ChNodeFEAxyz> nodeE,
                  std::shared_ptr<ChNodeFEAxyz> nodeF,
                  std::shared_ptr<ChNodeFEAxyz> nodeG,
                  std::shared_ptr<ChNodeFEAxyz> nodeH);

    /// Set the element number.
    void SetElemNum(int kb) { m_elementnumber = kb; }

    /// Set EAS internal parameters (stored values).
    void
    SetStockAlpha(double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);

    /// Set some element parameters (dimensions).
    void SetInertFlexVec(const ChVector<double>& a) { m_InertFlexVec = a; }

    int GetElemNum() const { return m_elementnumber; }

    /// Get initial position of the element in matrix form
    const ChMatrixNM<double, 8, 3>& GetInitialPos() const { return m_d0; }

    std::shared_ptr<ChNodeFEAxyz> GetNodeA() const { return m_nodes[0]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeB() const { return m_nodes[1]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeC() const { return m_nodes[2]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeD() const { return m_nodes[3]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeE() const { return m_nodes[4]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeF() const { return m_nodes[5]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeG() const { return m_nodes[6]; }
    std::shared_ptr<ChNodeFEAxyz> GetNodeH() const { return m_nodes[7]; }

    double GetLengthX() const { return m_InertFlexVec.x(); }
    double GetLengthY() const { return m_InertFlexVec.y(); }
    double GetLengthZ() const { return m_InertFlexVec.z(); }

    void SetMaterial(std::shared_ptr<ChContinuumElastic> my_material) { m_Material = my_material; }
    std::shared_ptr<ChContinuumElastic> GetMaterial() const { return m_Material; }
    /// Set whether material is Mooney-Rivlin (Otherwise linear elastic isotropic)
    void SetMooneyRivlin(bool val) { m_isMooney = val; }
    /// Set Mooney-Rivlin coefficients
    void SetMRCoefficients(double C1, double C2) {
        CCOM1 = C1;
        CCOM2 = C2;
    }
    /// Fills the N shape function matrix
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    void ShapeFunctions(ShapeVector& N, double x, double y, double z);
    void ShapeFunctionsDerivativeX(ShapeVector& Nx, double x, double y, double z);
    void ShapeFunctionsDerivativeY(ShapeVector& Ny, double x, double y, double z);
    void ShapeFunctionsDerivativeZ(ShapeVector& Nz, double x, double y, double z);
    // Functions for ChLoadable interface
    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 8 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 8 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return 8; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return m_nodes[nblock]->NodeGetOffsetW(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return !m_nodes[nblock]->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

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

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return this->m_Material->Get_density(); }

  private:
    enum JacobianType { ANALYTICAL, NUMERICAL };

    // Private Data
    std::vector<std::shared_ptr<ChNodeFEAxyz> > m_nodes;  ///< Element nodes

    std::shared_ptr<ChContinuumElastic> m_Material;  ///< Elastic Material

    ChMatrixNM<double, 24, 24> m_StiffnessMatrix;  ///< Stiffness matrix
    ChMatrixNM<double, 24, 24> m_MassMatrix;       ///< Mass matrix
    ChVectorN<double, 8> m_GravForceScale;  ///< Gravity scaling matrix used to get the generalized force due to gravity
    ChVector<double> m_InertFlexVec;        ///< for element size (EL,EW,EH)
    // EAS
    int m_elementnumber;                         ///< Element number, for EAS
    ChMatrixNM<double, 24, 24> m_stock_jac_EAS;  ///< EAS Jacobian matrix
    ChVectorN<double, 9> m_stock_alpha_EAS;      ///< EAS previous step internal parameters
    ChMatrixNM<double, 24, 24> m_stock_KTE;      ///< Analytical Jacobian
    ChMatrixNM<double, 8, 3> m_d0;               ///< Initial Coordinate per element
    JacobianType m_flag_HE;
    bool m_isMooney;  ///< Flag indicating whether the material is Mooney Rivlin
    double CCOM1;     ///< First coefficient for Mooney-Rivlin
    double CCOM2;     ///< Second coefficient for Mooney-Rivlin
                      // Private Methods

    virtual void Update() override;

    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    /// Computes the STIFFNESS MATRIX of the element:
    /// K = integral( .... ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeStiffnessMatrix();
    /// Computes the MASS MATRIX of the element
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();
    /// Compute the matrix to scale gravity by to get the generalized gravitational force.
    void ComputeGravityForceScale();
    /// Initial setup. Precompute mass and matrices that do not change during the simulation.
    virtual void SetupInitial(ChSystem* system) override;
    /// Sets M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrixRef M) override { M = m_MassMatrix; }

    /// Compute the generalized force vector due to gravity using the efficient element specific method
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    // [EAS] matrix T0 (inverse and transposed) and detJ0 at center are used for Enhanced Assumed Strains alpha
    void T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0, ChMatrixNM<double, 6, 6>& T0, double& detJ0C);
    // [EAS] Basis function of M for Enhanced Assumed Strain
    void Basis_M(ChMatrixNM<double, 6, 9>& M, double x, double y, double z);

    friend class Brick_ForceAnalytical;
    friend class Brick_ForceNumerical;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
