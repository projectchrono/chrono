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

#ifndef CHELEMENTTETRA4_H
#define CHELEMENTTETRA4_H

#include <cmath>

#include "chrono/fea/ChContinuumPoisson3D.h"
#include "chrono/fea/ChElementTetrahedron.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/physics/ChTensors.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Tetrahedron FEA element with 4 nodes.
/// This is a classical element with linear displacement, hence with constant stress
/// and constant strain. It can be easily used for 3D FEA problems.
class ChApi ChElementTetra_4 : public ChElementTetrahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    std::shared_ptr<ChContinuumElastic> Material;
    ChMatrixDynamic<> MatrB;            // matrix of shape function's partial derivatives
    ChMatrixDynamic<> StiffnessMatrix;  // undeformed local stiffness matrix

    ChMatrixNM<double, 4, 4> mM;  // for speeding up corotational approach

  public:
    ChElementTetra_4();
    virtual ~ChElementTetra_4();

    virtual int GetNnodes() override { return 4; }
    virtual int GetNdofs() override { return 4 * 3; }
    virtual int GetNodeNdofs(int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                          std::shared_ptr<ChNodeFEAxyz> nodeB,
                          std::shared_ptr<ChNodeFEAxyz> nodeC,
                          std::shared_ptr<ChNodeFEAxyz> nodeD);

    //
    // FEM functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at r,s,t 'volume' coordinates, where
    /// r=1 at 2nd vertex, s=1 at 3rd, t = 1 at 4th. All ranging in [0...1].
    /// The last (u, u=1 at 1st vertex) is computed form the first 3 as 1.0-r-s-t.
    /// NOTE! actually N should be a 3row, 12 column sparse matrix,
    /// as  N = [n1*eye(3) n2*eye(3) n3*eye(3) n4*eye(3)]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the n1 n2 n3 n4 values in a 1 row, 4 columns matrix.
    virtual void ShapeFunctions(ChMatrix<>& N, double r, double s, double t);

    /// Fills the D vector (displacement) column matrix with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    double ComputeVolume();

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    virtual void ComputeStiffnessMatrix();

    /// set up the element's parameters and matrices
    virtual void SetupInitial(ChSystem* system) override;

    /// compute large rotation of element for corotational approach
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
    ChMatrix<>& GetMatrB() { return MatrB; }
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }

    /// Returns the strain tensor (note that the tetrahedron 4 nodes is a linear
    /// element, thus the strain is constant in the entire volume).
    /// The tensor is in the original undeformed unrotated reference.
    ChStrainTensor<> GetStrain();

    /// Returns the stress tensor (note that the tetrahedron 4 nodes is a linear
    /// element, thus the stress is constant in the entire volume).
    /// The tensor is in the original undeformed unrotated reference.
    ChStressTensor<> GetStress();

    /// This function computes and adds corresponding masses to ElementBase member m_TotalMass
    void ComputeNodalMass() override;

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 4 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 4 * 3; }

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
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in 0..+1
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

// -----------------------------------------------------------------------------

/// Tetrahedron FEM element with 4 nodes for scalar fields (for Poisson-like problems).
/// This is a classical element with linear displacement.
/// ***EXPERIMENTAL***
class ChApi ChElementTetra_4_P : public ChElementTetrahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyzP> > nodes;
    std::shared_ptr<ChContinuumPoisson3D> Material;
    ChMatrixDynamic<> MatrB;            // matrix of shape function's partial derivatives
    ChMatrixDynamic<> StiffnessMatrix;  // local stiffness matrix

    ChMatrixNM<double, 4, 4> mM;  // for speeding up corotational approach

  public:
    ChElementTetra_4_P();

    virtual ~ChElementTetra_4_P() {}

    virtual int GetNnodes() override { return 4; }
    virtual int GetNdofs() override { return 4 * 1; }
    virtual int GetNodeNdofs(int n) override { return 1; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzP> nodeA,
                          std::shared_ptr<ChNodeFEAxyzP> nodeB,
                          std::shared_ptr<ChNodeFEAxyzP> nodeC,
                          std::shared_ptr<ChNodeFEAxyzP> nodeD);

    //
    // FEM functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at zi parametric coordinates, where
    /// z0=1 at 1st vertex, z1=1 at second, z2 = 1 at third (volumetric shape functions).
    /// The 4th is computed form the first 3.  All ranging in [0...1].
    /// NOTE! actually N should be a 3row, 12 column sparse matrix,
    /// as  N = [n1*eye(3) n2*eye(3) n3*eye(3) n4*eye(3)]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the n1 n2 n3 n4 values in a 1 row, 4 columns matrix!
    virtual void ShapeFunctions(ChMatrix<>& N, double z0, double z1, double z2);

    /// Fills the D vector column matrix with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    double ComputeVolume();

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    virtual void ComputeStiffnessMatrix();

    /// set up the element's parameters and matrices
    virtual void SetupInitial(ChSystem* system) override;

    // compute large rotation of element for corotational approach
    // Btw: NOT really needed for Poisson problems
    virtual void UpdateRotation() override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal 'pseudo-forces' and set values
    /// in the Fi vector. The iterative solver uses this to know if the residual went to zero.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    //
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumPoisson3D> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumPoisson3D> GetMaterial() { return Material; }

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    ChMatrix<>& GetMatrB() { return MatrB; }
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }

    /// Returns the gradient of P (note that the tetrahedron 4 nodes is a linear
    /// element, thus the gradient is constant in the entire volume).
    /// It is in the original undeformed unrotated reference.
    ChMatrixNM<double, 3, 1> GetPgradient();

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 4 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 4 * 3; }

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

    /// Number of coordinates in the interpolated field: here the {t} temperature
    virtual int Get_field_ncoords() override { return 1; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 1; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in 0..+1
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

    /// Return 0 if not supportable by ChLoaderVolumeGravity
    virtual double GetDensity() override { return 0; }

    /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
    /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return true; }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
