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

#ifndef CHELEMENTBEAMEULER_H
#define CHELEMENTBEAMEULER_H

#include "chrono/fea/ChBeamSection.h"
#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChElementCorotational.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Simple beam element with two nodes and Euler-Bernoulli formulation.
/// For this 'basic' implementation, constant section and constant
/// material are assumed.
///
/// Further information in the
/// [white paper PDF](http://www.projectchrono.org/assets/white_papers/euler_beams.pdf)
///
/// Note that there are also ChElementCableANCF if no torsional effects
/// are needed, as in cables.

class ChApi ChElementBeamEuler : public ChElementBeam,
                                 public ChLoadableU,
                                 public ChLoadableUVW,
                                 public ChElementCorotational {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes;

    std::shared_ptr<ChBeamSectionAdvanced> section;

    ChMatrixDynamic<> StiffnessMatrix;  ///< undeformed local stiffness matrix

    ChQuaternion<> q_refrotA;
    ChQuaternion<> q_refrotB;

    ChQuaternion<> q_element_abs_rot;
    ChQuaternion<> q_element_ref_rot;

    bool disable_corotate;
    bool disable_projector;
    bool force_symmetric_stiffness;

  public:
    ChElementBeamEuler();

    virtual ~ChElementBeamEuler() {}

    virtual int GetNnodes() override { return 2; }
    virtual int GetNdofs() override { return 2 * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB);

    //
    // FEM functions
    //

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetSection(std::shared_ptr<ChBeamSectionAdvanced> my_material) { section = my_material; }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionAdvanced> GetSection() { return section; }

    /// Get the first node (beginning)
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeA() { return nodes[0]; }

    /// Get the second node (ending)
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeB() { return nodes[1]; }

    /// Set the reference rotation of nodeA respect to the element rotation.
    void SetNodeAreferenceRot(ChQuaternion<> mrot) { q_refrotA = mrot; }
    ChQuaternion<> GetNodeAreferenceRot() { return q_refrotA; }

    /// Set the reference rotation of nodeB respect to the element rotation.
    void SetNodeBreferenceRot(ChQuaternion<> mrot) { q_refrotB = mrot; }
    ChQuaternion<> GetNodeBreferenceRot() { return q_refrotB; }

    /// Get the absolute rotation of element in space
    /// This is not the same of Rotation() , that expresses
    /// the accumulated rotation from starting point.
    ChQuaternion<> GetAbsoluteRotation() { return q_element_abs_rot; }

    /// Get the original reference rotation of element in space
    ChQuaternion<> GetRefRotation() { return q_element_ref_rot; }

    /// Set this as true to have the beam behave like a non-corotated beam
    /// (i.e. only for small deformations).
    void SetDisableCorotate(bool md) { disable_corotate = md; }

    /// Set this as true to disable the projectors for computing the
    /// tangent matrix in the corotational formulation
    /// (see C.A.Felippa, B.Haugen, N.Omid, C.Rankin et al. for details).
    /// Disabling the projectors leads to a faster code, but convergence might be more difficult.
    void SetDisableProjector(bool md) { disable_projector = md; }

    /// Set this as true to force the tangent stiffness matrix to be
    /// inexact, but symmetric. This allows the use of faster solvers. For systems close to
    /// the equilibrium, the tangent stiffness would be symmetric anyway.
    void SetForceSymmetricStiffness(bool md) { force_symmetric_stiffness = md; }

    /// Fills the N matrix (compressed! single row, 12 columns) with the
    /// values of shape functions at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Given  u = 12-d state block {u1,r1,u2,r2}' , d = 6-d field {u,r},
    /// one has   f(eta) = [S(eta)]*u   where one fills the sparse [S] matrix
    /// with the N shape functions in this pattern:
    ///      | 0    .   .   .   .   .   3   .   .   .   .   .   |
    ///      | .    1   .   .   .   2   .   4   .   .   .   5   |
    /// [S] =| .    .   1   .   -2  .   .   .   4   .   -5  .   |
    ///      | .    .   .   0   .   .   .   .   .   3   .   .   |
    ///      | .    .   -6  .   8   .   .   .   -7  .   9   .   |
    ///      | .    6   .   .   .   8   .   7   .   .   .   9   |
    virtual void ShapeFunctions(ChMatrix<>& N, double eta);

    virtual void Update() override;

    /// Compute large rotation of element for corotational approach
    /// The reference frame of this Euler-Bernoulli beam has X aligned to two nodes and Y parallel to Y of 1st node
    virtual void UpdateRotation() override;

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// Give that this element includes rotations at nodes, this gives:
    ///  {x_a y_a z_a Rx_a Ry_a Rz_a x_b y_b z_b Rx_b Ry_b Rz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Fills the Ddt vector (column matrix) with the current
    /// tme derivatives of field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// Give that this element includes rotations at nodes, this gives:
    ///  {v_a v_a v_a wx_a wy_a wz_a v_b v_b v_b wx_b wy_b wz_b}
    virtual void GetField_dt(ChMatrixDynamic<>& mD_dt);

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = integral( [B]' * [D] * [B] ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed, so the explicit result of quadrature is used.
    virtual void ComputeStiffnessMatrix();

    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.
    virtual void SetupInitial(ChSystem* system) override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (e.g. the actual position of nodes is not in relaxed reference position)
    /// and set values in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    //
    // Beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) override;

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are corotated (expressed in world reference)
    virtual void EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) override;

    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector<>& Fforce, ChVector<>& Mtorque) override;

    /* To be completed: Created to be consistent with base class implementation*/
    virtual void EvaluateSectionStrain(const double eta, ChVector<>& StrainV) override {}

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 2 * 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 2 * 6; }

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

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 2; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U coordinates of the line, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in line
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

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
