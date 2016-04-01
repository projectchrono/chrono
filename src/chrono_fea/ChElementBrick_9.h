// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Brick element with 9 nodes (central node for curvature)
// =============================================================================

#ifndef CHELEMENTBRICK9_H
#define CHELEMENTBRICK9_H

#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementGeneric.h"
#include "chrono_fea/ChNodeFEAcurv.h"
#include "chrono_fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Brick element with 9 nodes.
class ChApiFea ChElementBrick_9 : public ChElementGeneric, public ChLoadableUVW {
  public:
    ChElementBrick_9();
    ~ChElementBrick_9() {}

    /// Get number of nodes of this element
    virtual int GetNnodes() override { return 9; }

    /// Get number of degrees of freedom of this element
    virtual int GetNdofs() override { return 8 * 3 + 9; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual int GetNodeNdofs(int n) override {
        if (n < 8)
            return 3;

        return 9;
    }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override {
        if (n < 8)
            return m_nodes[n];

        return m_central_node;
    }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyz> node1,
                  std::shared_ptr<ChNodeFEAxyz> node2,
                  std::shared_ptr<ChNodeFEAxyz> node3,
                  std::shared_ptr<ChNodeFEAxyz> node4,
                  std::shared_ptr<ChNodeFEAxyz> node5,
                  std::shared_ptr<ChNodeFEAxyz> node6,
                  std::shared_ptr<ChNodeFEAxyz> node7,
                  std::shared_ptr<ChNodeFEAxyz> node8,
                  std::shared_ptr<ChNodeFEAcurv> nodeC);

    std::shared_ptr<ChNodeFEAxyz> GetNode1() const { return m_nodes[0]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode2() const { return m_nodes[1]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode3() const { return m_nodes[2]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode4() const { return m_nodes[3]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode5() const { return m_nodes[4]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode6() const { return m_nodes[5]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode7() const { return m_nodes[6]; }
    std::shared_ptr<ChNodeFEAxyz> GetNode8() const { return m_nodes[7]; }
    std::shared_ptr<ChNodeFEAcurv> GetCentralNode() const { return m_central_node; }

    double GetLengthX() const {
        //// TODO
        return 0;
    }
    double GetLengthY() const {
        //// TODO
        return 0;
    }

    double GetLengthZ() const {
        //// TODO
        return 0;
    }

    void SetMaterial(std::shared_ptr<ChContinuumElastic> material) { m_material = material; }
    std::shared_ptr<ChContinuumElastic> GetMaterial() const { return m_material; }

    /// Fills the N shape function matrix.
    ///   N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...];
    void ShapeFunctions(ChMatrix<>& N, double x, double y, double z);
    void ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z);
    void ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z);
    void ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z);

    /// Get the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 8 * 3 + 9; }

    /// Get the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 8 * 3 + 9; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) override {
        //// TODO
    }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) override {
        //// TODO
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() override {
        //// TODO -- problem here???
        return 3;
    }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() { return 9; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override {
        if (nblock < 8)
            return m_nodes[nblock]->NodeGetOffset_w();
    
        return m_central_node->NodeGetOffset_w();
    }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override {
        if (nblock < 8)
            return 3;

        return 9;
    }

    /// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) override;

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
    virtual double GetDensity() override { return this->m_material->Get_density(); }

  private:
    std::vector<std::shared_ptr<ChNodeFEAxyz>> m_nodes;  ///< corner element nodes
    std::shared_ptr<ChNodeFEAcurv> m_central_node;       ///< central node

    std::shared_ptr<ChContinuumElastic> m_material;  ///< elastic naterial

    ChMatrixNM<double, 33, 1> m_GravForce;        ///< gravitational force
    ChMatrixNM<double, 33, 33> m_MassMatrix;      ///< mass matrix
    ChMatrixNM<double, 33, 33> m_JacobianMatrix;  ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])

    /// Update this element.
    virtual void Update() override;

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD);

    /// Computes the MASS MATRIX of the element.
    void ComputeMassMatrix();
    /// Compute the gravitational forces.
    void ComputeGravityForce(const ChVector<>& g_acc);

    /// Initial element setup.
    virtual void SetupInitial(ChSystem* system) override;
    /// Sets M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrix<>& M) override;
    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override;

    /// Compute internal forces and load them in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    /// Compute Jacobians of the internal forces.
    /// This function calculates a linear combination of the stiffness (K) and damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coeficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobians(double Kfactor, double Rfactor);

    friend class MyMassBrick9;
    friend class MyGravityBrick9;
    friend class MyForceBrick9;
    friend class MyJacobianBrick9;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
