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
// Authors: Andrea Favali
// =============================================================================

#ifndef CHELEMENTHEXA8_H
#define CHELEMENTHEXA8_H

#include "chrono/fea/ChElementHexahedron.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Class for FEA elements of hexahedron type (isoparametric 3D bricks) with 8 nodes.
/// This element has a linear displacement field.
class ChApi ChElementHexa_8 : public ChElementHexahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    std::shared_ptr<ChContinuumElastic> Material;
    // std::vector< ChMatrixDynamic<> > MatrB;	// matrices of shape function's partial derivatives (one for each
    // integration point)
    // we use a vector to keep in memory all the 8 matrices (-> 8 integr. point)
    // NO! each matrix is stored in the respective gauss point

    ChMatrixDynamic<> StiffnessMatrix;

  public:
    ChElementHexa_8();
    virtual ~ChElementHexa_8();

    virtual int GetNnodes() override { return 8; }
    virtual int GetNdofs() override { return 8 * 3; }
    virtual int GetNodeNdofs(int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                          std::shared_ptr<ChNodeFEAxyz> nodeB,
                          std::shared_ptr<ChNodeFEAxyz> nodeC,
                          std::shared_ptr<ChNodeFEAxyz> nodeD,
                          std::shared_ptr<ChNodeFEAxyz> nodeE,
                          std::shared_ptr<ChNodeFEAxyz> nodeF,
                          std::shared_ptr<ChNodeFEAxyz> nodeG,
                          std::shared_ptr<ChNodeFEAxyz> nodeH);

    //
    // QUADRATURE functions
    //

    virtual void SetDefaultIntegrationRule() { this->ir->SetIntOnCube(8, &this->GpVector); }

    virtual void SetReducedIntegrationRule() { this->ir->SetIntOnCube(1, &this->GpVector); }

    virtual void SetIntegrationRule(int nPoints) { this->ir->SetIntOnCube(nPoints, &this->GpVector); }

    //
    // FEA functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at z0,z1,z2 parametric coordinates, where
    /// each zi is in [-1...+1] range.
    /// It stores the Ni(z0,z1,z2) values in a 1 row, 8 columns matrix.
    virtual void ShapeFunctions(ChMatrix<>& N, double z0, double z1, double z2);

    /// Fills the D vector (displacement) column matrix with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Puts inside 'Jacobian' and 'J1' the Jacobian matrix and the shape functions derivatives matrix of the element.
    /// The vector "coord" contains the natural coordinates of the integration point.
    /// in case of hexahedral elements natural coords vary in the classical range -1 ... +1.
    virtual void ComputeJacobian(ChMatrixDynamic<>& Jacobian, ChMatrixDynamic<>& J1, ChVector<> coord);

    /// Computes the matrix of partial derivatives and puts data in "MatrB"
    ///	evaluated at natural coordinates zeta1,...,zeta4 . Also computes determinant of jacobian.
    /// note: in case of hexahedral elements natural coord. vary in the range -1 ... +1
    virtual void ComputeMatrB(ChMatrixDynamic<>& MatrB, double zeta1, double zeta2, double zeta3, double& JacobianDet);

    /// Computes the matrix of partial derivatives and puts data in "GaussPt".
    ///	Stores the determinant of the jacobian in "JacobianDet".
    virtual void ComputeMatrB(ChGaussPoint* GaussPt, double& JacobianDet);

    /// Computes the global STIFFNESS MATRIX of the element:
    /// K = sum (w_i * [B]' * [D] * [B])
    /// The number of Gauss Point is defined by SetIntegrationRule function (default: 8 Gp).
    virtual void ComputeStiffnessMatrix();

    virtual void SetupInitial(ChSystem* system) override { ComputeStiffnessMatrix(); }

    // compute large rotation of element for corotational approach
    virtual void UpdateRotation() override;

    /// Returns the strain tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStrainTensor<> GetStrain(double z1, double z2, double z3);

    /// Returns the stress tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStressTensor<> GetStress(double z1, double z2, double z3);

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    //
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumElastic> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumElastic> GetMaterial() { return Material; }

    /// Get the StiffnessMatrix
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }
    /// Get the Nth gauss point
    ChGaussPoint* GetGaussPoint(int N) { return GpVector[N]; }

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

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

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 8; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

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
    virtual double GetDensity() override { return this->Material->Get_density(); }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
