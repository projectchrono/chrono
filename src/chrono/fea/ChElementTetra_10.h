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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================

#ifndef CHELEMENTTETRA10_H
#define CHELEMENTTETRA10_H

#include <cmath>

#include "chrono/fea/ChElementTetrahedron.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Tetrahedron FEA element with 10 nodes.
/// This is a quadratic element for displacements; stress and strain
/// are interpolated depending on Gauss points.
class ChApi ChElementTetra_10 : public ChElementTetrahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    std::shared_ptr<ChContinuumElastic> Material;
    std::vector<ChMatrixDynamic<> >
        MatrB;  // matrices of shape function's partial derivatives (one for each integration point)
                // we use a vector to keep in memory all the four matrices (-> 4 integr. point)
    ChMatrixDynamic<> StiffnessMatrix;

    ChMatrixNM<double, 4, 4> mM;  // for speeding up corotational approach

  public:
    ChElementTetra_10();
    virtual ~ChElementTetra_10();

    virtual int GetNnodes() override { return 10; }
    virtual int GetNdofs() override { return 10 * 3; }
    virtual int GetNodeNdofs(int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                          std::shared_ptr<ChNodeFEAxyz> nodeB,
                          std::shared_ptr<ChNodeFEAxyz> nodeC,
                          std::shared_ptr<ChNodeFEAxyz> nodeD,
                          std::shared_ptr<ChNodeFEAxyz> nodeE,
                          std::shared_ptr<ChNodeFEAxyz> nodeF,
                          std::shared_ptr<ChNodeFEAxyz> nodeG,
                          std::shared_ptr<ChNodeFEAxyz> nodeH,
                          std::shared_ptr<ChNodeFEAxyz> nodeI,
                          std::shared_ptr<ChNodeFEAxyz> nodeJ);

    //
    // FEM functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at zi parametric coordinates, where
    /// r=1 at 2nd vertex, s=1 at 3rd, t=1 at 4th. All ranging in [0...1].
    /// The last, u (=1 at 1st vertex) is computed form the first 3.
    /// It stores the Ni(r,s,t) values in a 1 row, 10 columns matrix.
    virtual void ShapeFunctions(ChMatrix<>& N, double r, double s, double t);

    /// Fills the D vector (displacement) column matrix with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Approximation!! not the exact volume
    /// This returns an exact value only in case of Constant Metric Tetrahedron
    double ComputeVolume();

    /// Puts inside 'Jacobian' the Jacobian matrix of the element
    /// zeta1,...,zeta4 are the four natural coordinates of the integration point
    /// note: in case of tetrahedral elements natural coord. vary in the range 0 ... +1
    virtual void ComputeJacobian(ChMatrixDynamic<>& Jacobian, double zeta1, double zeta2, double zeta3, double zeta4);

    /// Computes the matrix of partial derivatives and puts data in "mmatrB"
    ///	evaluated at natural coordinates zeta1,...,zeta4
    /// note: in case of tetrahedral elements natural coord. vary in the range 0 ... +1
    virtual void ComputeMatrB(ChMatrixDynamic<>& mmatrB,
                              double zeta1,
                              double zeta2,
                              double zeta3,
                              double zeta4,
                              double& JacobianDet);

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = sum (w_i * [B]' * [D] * [B])
    ///
    virtual void ComputeStiffnessMatrix();

    /// Given the node ID, gets the 4 parameters of the shape function
    void GetParameterForNodeID(const int nodeID, double& z1, double& z2, double& z3, double& z4);

    /// Returns the strain tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStrainTensor<> GetStrain(double z1, double z2, double z3, double z4);

    /// Returns the stress tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStressTensor<> GetStress(double z1, double z2, double z3, double z4);

    virtual void SetupInitial(ChSystem* system) override;

    // compute large rotation of element for corotational approach
    virtual void UpdateRotation() override;

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

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    ChMatrix<>& GetMatrB(int n) { return MatrB[n]; }
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 10 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 10 * 3; }

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
    virtual int GetSubBlocks() override { return 10; }

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

    /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
    /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return true; }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
