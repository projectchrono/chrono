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
// Authors: Alessandro Tasora, Radu Serban, Antonio Recuero
// =============================================================================
// ANCF gradient-deficient cable element.
// =============================================================================

#ifndef CHELEMENTCABLEANCF_H
#define CHELEMENTCABLEANCF_H

#include "chrono/core/ChVector.h"
#include "chrono/fea/ChBeamSection.h"
#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Simple beam element with two nodes and ANCF gradient-deficient formulation.
/// For this 'basic' implementation, constant section and constant material are assumed along the beam coordinate.
/// Torsional stiffness is impossible because of the formulation.
/// Based on the formulation in:
///  "Analysis of Thin Beams and Cables Using the Absolute Nodal Co-ordinate Formulation"
///  J.GERSTMAYR, A.SHABANA
///  Nonlinear Dynamics (2006) 45: 109-130
///  DOI: 10.1007/s11071-006-1856-1
/// and in:
/// "On the Validation and Applications of a Parallel Flexible Multi-body Dynamics Implementation"
///  D. MELANZ
class ChApi ChElementCableANCF : public ChElementBeam, public ChLoadableU, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyzD> > nodes;

    std::shared_ptr<ChBeamSectionCable> section;
    ChMatrixNM<double, 12, 1> m_GenForceVec0;
    ChMatrixNM<double, 12, 12> m_JacobianMatrix;  ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])
    ChMatrixNM<double, 12, 12> m_MassMatrix;      ///< mass matrix

  public:
    bool m_use_damping;  ///< Boolean indicating whether internal damping is added
    double m_alpha;      ///< Scaling factor for internal damping

    ChElementCableANCF();
    virtual ~ChElementCableANCF() {}

    virtual int GetNnodes() override { return 2; }
    virtual int GetNdofs() override { return 2 * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzD> nodeA, std::shared_ptr<ChNodeFEAxyzD> nodeB);

    //
    // FEM functions
    //

    /// Set the section & material of beam element.
    /// It is a shared property, so it can be shared between other beams.
    void SetSection(std::shared_ptr<ChBeamSectionCable> my_material) { section = my_material; }

    /// Get the section & material of the element.
    std::shared_ptr<ChBeamSectionCable> GetSection() { return section; }

    /// Get the first node (beginning).
    std::shared_ptr<ChNodeFEAxyzD> GetNodeA() { return nodes[0]; }

    /// Get the second node (ending).
    std::shared_ptr<ChNodeFEAxyzD> GetNodeB() { return nodes[1]; }

    /// Get element length.
    double GetCurrLength() { return (nodes[1]->GetPos() - nodes[0]->GetPos()).Length(); }

    /// Fills the N shape function matrix with the values of shape functions at abscissa 'xi'.
    /// Note, xi=0 at node1, xi=+1 at node2.
    /// N should be a 3x12 parse matrix, N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)],
    /// but is stored here in a compressed form: only the s1 s2 s3 s4 values in a 1x4 column vector.
    virtual void ShapeFunctions(ChMatrix<>& N, double xi);

    /// Fills the N shape function derivative matrix with the values of shape function derivatives at abscissa 'xi'.
    /// Note, xi=0 at node1, xi=+1 at node2.
    /// In a compressed form, only four values are stored in a 1x4 column vector.
    virtual void ShapeFunctionsDerivatives(ChMatrix<>& Nd, double xi);

    /// Fills the N shape function derivative matrix with the values of shape function 2nd derivatives at abscissa 'xi'.
    /// Note, xi=0 at node1, xi=+1 at node2.
    /// In a compressed form, only four values are stored in a 1x4 column vector.
    virtual void ShapeFunctionsDerivatives2(ChMatrix<>& Ndd, double xi);

    virtual void Update() override;

    /// Fills the D vector (column matrix) with the current field values at the nodes of the element, with proper
    /// ordering. If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Computes the stiffness matrix of the element:
    /// K = integral( .... ),
    /// Note: in this 'basic' implementation, constant section and constant material are assumed.
    virtual void ComputeInternalJacobians(double Kfactor, double Rfactor);

    /// Computes the mass matrix of the element.
    /// Note: in this 'basic' implementation, constant section and constant material are assumed.
    virtual void ComputeMassMatrix();

    /// Setup: precompute mass and matrices that do not change during the simulation.
    virtual void SetupInitial(ChSystem* system) override;

    /// Sets M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrix<>& M) override { M = m_MassMatrix; }

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also superimposes global
    /// damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces and set values in the Fi vector.
    /// (e.g. the actual position of nodes is not in relaxed reference position).
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    //
    // Beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line and the rotation RxRyRz of section plane, at abscissa
    /// 'eta'. Note, eta=-1 at node1, eta=+1 at node2. Note that 'displ' is the displ.state of 2 nodes, e.g. get it as
    /// GetStateBlock() Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) override;

    /// Gets the absolute xyz position of a point on the beam line and the absolute rotation of section plane, at
    /// abscissa 'eta'. Note, eta=-1 at node1, eta=+1 at node2. Note that 'displ' is the displ.state of 2 nodes,
    /// e.g. get it as GetStateBlock() Results are corotated (expressed in world reference)
    virtual void EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) override;

    /// Gets the force (traction x, shear y, shear z) and the torque (torsion on x, bending on y, on bending on z)
    /// at a section along the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note that 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock().
    /// Results are not corotated, and are expressed in the reference system of beam.
    /// This is not mandatory for the element to work, but it can be useful for plotting, showing results, etc.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector<>& Fforce, ChVector<>& Mtorque) override;

    /// Gets the axial and bending strain of the ANCF element torque (torsion on x, bending on y, on bending on z)
    /// at a section along the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note that 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock().
    /// Results are not corotated, and are expressed in the reference system of beam.
    /// This is not mandatory for the element to work, but it can be useful for plotting, showing results, etc.
    virtual void EvaluateSectionStrain(const double eta, ChVector<>& StrainV) override;

    /// Set structural damping.
    void SetAlphaDamp(double a);

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 2 * 6; }

    /// Gets the number of DOFs affected by this element (speed part).
    virtual int LoadableGet_ndof_w() override { return 2 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field.
    virtual int Get_field_ncoords() override { return 6; }

    /// Return the number of DOFs blocks.
    virtual int GetSubBlocks() override { return 2; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function evaluated at U,V coordinates of the surface,
    /// each ranging in -1..+1.  F is a load, N'*F is the resulting generalized load.
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in line
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// Evaluate N'*F , where N is some type of shape function evaluated at U,V,W coordinates of the volume,
    /// each ranging in -1..+1.  F is a load, N'*F is the resulting generalized load.
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

    /// Return the material density for this element.
    virtual double GetDensity() override { return this->section->Area * this->section->density; }

  private:
    /// Worker function for computing the internal forces.
    /// This function takes the nodal coordinates as arguments and is therefore thread-safe.
    /// (Typically invoked by ComputeInternalForces. Used explicitly in the FD Jacobian approximation).
    void ComputeInternalForces_Impl(const ChVector<>& pA,
                                    const ChVector<>& dA,
                                    const ChVector<>& pB,
                                    const ChVector<>& dB,
                                    const ChVector<>& pA_dt,
                                    const ChVector<>& dA_dt,
                                    const ChVector<>& pB_dt,
                                    const ChVector<>& dB_dt,
                                    ChMatrixDynamic<>& Fi);
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
