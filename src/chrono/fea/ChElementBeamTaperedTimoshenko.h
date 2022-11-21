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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHELEMENTBEAMTAPEREDTIMOSHENKO_H
#define CHELEMENTBEAMTAPEREDTIMOSHENKO_H

#include "chrono/fea/ChBeamSectionTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChElementCorotational.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Classical Timoshenko beam element with two nodes, and tapered sections.
/// For this tapered beam element, the averaged section properties are
/// used to formulate the mass, stiffness and damping matrices.
/// Note that there are also ChElementCableANCF if no torsional effects
/// are needed, as in cables.
class ChApi ChElementBeamTaperedTimoshenko : public ChElementBeam,
                                             public ChLoadableU,
                                             public ChLoadableUVW,
                                             public ChElementCorotational {
  private:
    // define a tuple to restore the shape functions and derivatives
    using ShapeFunctionN = ChMatrixNM<double, 6, 12>;
    using SFBlock = ChMatrixNM<double, 1, 4>;
    using ShapeFunction5Blocks = std::tuple<SFBlock, SFBlock, SFBlock, SFBlock, ChMatrixNM<double, 1, 2>>;
    using ShapeFunction2Blocks = std::tuple<SFBlock, SFBlock>;
    using ShapeFunctionGroup = std::tuple<ShapeFunctionN,         // restore shape function
                                          ShapeFunction5Blocks,   // restore blocks of shape function
                                          ShapeFunction5Blocks,   // restore blocks of first derivatives
                                          ShapeFunction2Blocks,   // restore blocks of second derivatives
                                          ShapeFunction2Blocks>;  // restore blocks of thrid derivatives

  public:
    ChElementBeamTaperedTimoshenko();

    ~ChElementBeamTaperedTimoshenko() {}

    virtual int GetNnodes() override { return 2; }
    virtual int GetNdofs() override { return 2 * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB);

    //
    // FEM functions
    //

    /// Set the tapered section & material of beam element .
    void SetTaperedSection(std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric> my_material) {
        tapered_section = my_material;
    }

    /// Get the tapered section & material of the element
    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric> GetTaperedSection() { return tapered_section; }

    /// Get the first node (beginning)
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeA() { return nodes[0]; }

    /// Get the second node (ending)
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeB() { return nodes[1]; }

    /// Set the reference rotation of nodeA respect to the element rotation.
    void SetNodeAreferenceRot(ChQuaternion<> mrot) { q_refrotA = mrot; }
    /// Get the reference rotation of nodeA respect to the element rotation.
    ChQuaternion<> GetNodeAreferenceRot() { return q_refrotA; }

    /// Set the reference rotation of nodeB respect to the element rotation.
    void SetNodeBreferenceRot(ChQuaternion<> mrot) { q_refrotB = mrot; }
    /// Get the reference rotation of nodeB respect to the element rotation.
    ChQuaternion<> GetNodeBreferenceRot() { return q_refrotB; }

    /// Get the absolute rotation of element in space
    /// This is not the same of Rotation() , that expresses
    /// the accumulated rotation from starting point.
    ChQuaternion<> GetAbsoluteRotation() { return q_element_abs_rot; }

    /// Get the original reference rotation of element in space
    ChQuaternion<> GetRefRotation() { return q_element_ref_rot; }

    /// Set this as true to have the beam behave like a non-corotated beam
    /// hence do not update the corotated reference. Just for benchmarks!
    void SetDisableCorotate(bool md) { disable_corotate = md; }

    /// Set this as true to force the tangent stiffness matrix to be
    /// inexact, but symmetric. This allows the use of faster solvers. For systems close to
    /// the equilibrium, the tangent stiffness would be symmetric anyway.
    void SetForceSymmetricStiffness(bool md) { force_symmetric_stiffness = md; }

    /// Set this as false to disable the contribution of geometric stiffness
    /// to the total tangent stiffness. By default it is on.
    void SetUseGeometricStiffness(bool md) { this->use_geometric_stiffness = md; }

    /// Set this as true to include the transformation matrix due to the different elastic
    /// center offsets at two ends of beam element with respect to the centerline reference,
    /// in which case the connection line of two elastic centers is not parallel to the
    /// one of two centerline references at two ends. By default it is on.
    /// Please refer to ANSYS theory document for more information.
    void SetUseRc(bool md) { this->use_Rc = md; }

    /// Set this as true to include the transformation matrix due to the different shear
    /// center offsets at two ends of beam element with respect to the centerline reference,
    /// in which case the connection line of two shear centers is not parallel to the one
    /// of two centerline references at two ends. By default it is on.
    /// This transformation matrix could decrease the equivalent torsional stiffness of beam
    /// element, and hence generate larger torsional rotation.
    /// Please refer to ANSYS theory document for more information.
    void SetUseRs(bool md) { this->use_Rs = md; }

    /// Set this as true to use a simplified correction model for the case of inclined shear axis. 
    /// By default it is false. This option may affect the bending-twist coupling of Timoshenko 
    /// beam element, especially when the inclined angle of shear center axis is obvious with
    /// respect to the centerline.
    void SetUseSimplifiedCorrectionForInclinedShearAxis(bool md) {
        this->use_simplified_correction_for_inclined_shear_axis = md;
    }
    
    /// Shape functions for Timoshenko beam.
    /// Please refer to the textbook:
    /// J. S. Przemieniecki, Theory of Matrix Structural Analysis-Dover Publications (1985).
    void ShapeFunctionsTimoshenko(ShapeFunctionGroup& NN, double eta);

    virtual void Update() override;

    /// Compute large rotation of element for corotational approach
    /// The reference frame of this Euler-Bernoulli beam has X aligned to two nodes and Y parallel to Y of 1st node
    virtual void UpdateRotation() override;

    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// Given that this element includes rotations at nodes, this gives:
    ///  {x_a y_a z_a Rx_a Ry_a Rz_a x_b y_b z_b Rx_b Ry_b Rz_b}
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    /// Fills the Ddt vector with the current time derivatives of field values at the nodes of the element, with proper
    /// ordering. If the D vector has not the size of this->GetNdofs(), it will be resized. For corotational elements,
    /// field is assumed in local reference! Give that this element includes rotations at nodes, this gives:
    ///  {v_a v_a v_a wx_a wy_a wz_a v_b v_b v_b wx_b wy_b wz_b}
    void GetField_dt(ChVectorDynamic<>& mD_dt);

    /// Fills the Ddtdt vector with the current time derivatives of field values at the nodes of the element, with
    /// proper ordering. If the D vector has not the size of this->GetNdofs(), it will be resized. For corotational
    /// elements, field is assumed in local reference! Give that this element includes rotations at nodes, this gives:
    ///  {acc_a acc_a acc_a accx_a accy_a accz_a acc_b acc_b acc_b accx_b accy_b accz_b}
    void GetField_dtdt(ChVectorDynamic<>& mD_dtdt);

    /// Computes the local (material) stiffness matrix of the element:
    /// K = integral( [B]' * [D] * [B] ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed, so the explicit result of quadrature is used.
    /// Also, this local material stiffness matrix is constant, computed only at the beginning
    /// for performance reasons; if you later change some material property, call this or InitialSetup().
    void ComputeStiffnessMatrix();

    /// Computes the local (material) damping matrix of the element:
    /// R = beta * integral( [B]' * [D] * [B] ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed, so the explicit result of quadrature is used.
    /// Also, this local material damping matrix is constant, computed only at the beginning
    /// for performance reasons; if you later change some material property, call this or InitialSetup().
    /// Only the stiffness term(beta) is used for this implemented Rayleigh damping model.
    void ComputeDampingMatrix();

    /// Computes the local geometric stiffness Kg of the element.
    /// Note: this->Kg will be set as the geometric stiffness EXCLUDING the multiplication by the P pull force,
    /// in fact P multiplication happens in all terms, thus this allows making the Kg as a constant matrix that
    /// is computed only at the beginning, and later it is multiplied by P all times the real Kg is needed.
    /// If you later change some material property, call this or InitialSetup().
    void ComputeGeometricStiffnessMatrix();

    /// Compute the inertia stiffness matrix [Ki^] and inertial damping matrix [Ri^]
    /// which are due to the gyroscopic effect.
    void ComputeKiRimatricesLocal(bool inertial_damping, bool inertial_stiffness);

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Gets the material mass, material stiffness, material damping and geometric stiffness matrices in local basis.
    /// This functionality can be used to output these matrices for some other special needs.
    virtual void GetKRMmatricesLocal(ChMatrixRef H, double Kmfactor, double Kgfactor, double Rmfactor, double Mfactor);

    /// Computes the internal forces (e.g. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    /// This functionality can be used to output the forces and torques at two nodes directly.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    /// Computes the inertial forces, damping forces, centrifugal forces and gyroscopic moments, then you could
    /// consider them as applied external forces, if you want to do the static solve when including nodal velocites
    /// and accelarations. Strictly speaking, it is not static solve any more. We can name it as quasi-static
    /// equilibrium solving, just the same as Simpack.
    /// It is recommended to use ChStaticNonLinearRheonomicAnalysis to do this kind of quasi-static equilibrium
    /// solving in case of rotating beams.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi, bool Mfactor, bool Kfactor, bool Rfactor, bool Gfactor);

    /// Compute gravity forces, grouped in the Fg vector, one node after the other
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) override;

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

    /// Gets the strains(traction along x, shear along y, along shear z, torsion about x, bending about y, on bending
    /// about z) at a section along the beam line, at abscissa 'eta'. It's evaluated at the elastic center. Note, eta=-1
    /// at node1, eta=+1 at node2. Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionStrain(const double eta, ChVector<>& StrainV_trans, ChVector<>& StrainV_rot);

    /// Gets the elastic strain energy(traction along x, shear along y, along shear z, torsion about x, bending about
    /// y, on bending about z) in the element.
    virtual void EvaluateElementStrainEnergy(ChVector<>& StrainEnergyV_trans, ChVector<>& StrainEnergyV_rot);

    /// Gets the damping dissipated energy(traction along x, shear along y, along shear z, torsion about x, bending
    /// about y, on bending about z) in the element.
    virtual void EvaluateElementDampingEnergy(ChVector<>& DampingEnergyV_trans, ChVector<>& DampingEnergyV_rot);

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

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return 2; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffsetW(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return !nodes[nblock]->IsFixed(); }

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

  protected:
    /// Initial setup. Precompute mass and matrices that do not change during the simulation, such as the local tangent
    /// stiffness Kl of each element, if needed, etc.
    virtual void SetupInitial(ChSystem* system) override;

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;

    /// Tapered section & material of beam element
    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric> tapered_section;

    ChMatrixDynamic<> Km;  ///< local material  stiffness matrix
    ChMatrixDynamic<> Kg;  ///< local geometric stiffness matrix NORMALIZED by P
    ChMatrixDynamic<> M;   ///< local material mass matrix. It could be lumped or consistent mass matrix, depending on
                           ///< SetLumpedMassMatrix(true/false)
    ChMatrixDynamic<> Rm;  ///< local material damping matrix
    ChMatrixDynamic<> Ri;  ///< local inertial-damping (gyroscopic damping) matrix
    ChMatrixDynamic<> Ki;  ///< local inertial-stiffness matrix

    ChQuaternion<> q_refrotA;
    ChQuaternion<> q_refrotB;

    ChQuaternion<> q_element_abs_rot;
    ChQuaternion<> q_element_ref_rot;

    bool disable_corotate;
    bool force_symmetric_stiffness;

    bool use_geometric_stiffness;  ///< whether include geometric stiffness matrix
    bool use_Rc;                   ///< whether use the transformation matrix for elastic axis orientation
    bool use_Rs;                   ///< whether use the transformation matrix for shear axis orientation
    bool use_simplified_correction_for_inclined_shear_axis = false;///< whether use the simplified correction model for shear axis orientation, it's false as default.

    // Flag that turns on/off the computation of the [Ri] 'gyroscopic' inertial damping matrix.
    // If false, Ri=0. Can be used for cpu speedup, profiling, tests. Default: true.
    // bool compute_inertia_damping_matrix = true;

    // Flag that turns on/off the computation of the [Ki] inertial stiffness matrix.
    // If false, Ki=0. Can be used for cpu speedup, profiling, tests. Default: true.
    // bool compute_inertia_stiffness_matrix = true;

    ChMatrixDynamic<> T;   ///< transformation matrix for entire beam element
    ChMatrixDynamic<> Rc;  ///< transformation matrix for elastic axis orientation
    ChMatrixDynamic<> Rs;  ///< transformation matrix for shear axis orientation

    /// compute the transformation matrix due to offset and rotation of axes.
    void ComputeTransformMatrix();

    /// compute the transformation matrix due to offset and rotation of axes, at dimensionless abscissa eta.
    void ComputeTransformMatrixAtPoint(ChMatrixDynamic<>& mT, const double eta);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
