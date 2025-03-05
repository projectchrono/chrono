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
// Authors: Erol Lale, Jibril B. Coulibaly
// =============================================================================

#ifndef CHELEMENT_CBLCON_H
#define CHELEMENT_CBLCON_H

#include "chrono_wood/ChWoodApi.h"
#include "chrono_wood/ChBeamSectionCBLCON.h"
//#include "chrono/fea/ChBeamSectionEuler.h"
#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChElementCorotational.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

using namespace chrono::fea;

namespace chrono {
namespace wood {

/// @addtogroup wood_elements
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

class ChWoodApi ChElementCBLCON : public ChElementBeam,				
                                 public ChLoadableU,
                                 public ChLoadableUVW,
                                 public ChElementCorotational {
  public:
    using ShapeVector = ChMatrixNM<double, 1, 10>;
    using Amatrix = ChMatrixNM<double,3,6> ;
    using StateVarVector = ChVectorN<double, 18>;

    ChElementCBLCON();

    ~ChElementCBLCON() {}

    virtual unsigned int GetNumNodes() override { return 2; }
    virtual unsigned int GetNumCoordsPosLevel() override { return 2 * 6; }
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; }
	
	virtual std::shared_ptr<ChNodeFEAxyzrot> GetConnectorNode(unsigned int n) { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB);
	
	//
	// CBLCON functions
	//
	
	int GetVertNodeVec_Size() { return V_vert_nodes.size(); }
	
	std::vector<std::shared_ptr<ChNodeFEAxyzrot>> GetVertNodeVec(int n) { return this->V_vert_nodes[n]; }
    void SetVertNodeVec(std::vector<std::vector<std::shared_ptr<ChNodeFEAxyzrot>>> mVvert) { V_vert_nodes=mVvert; }
	void AddVertNodeVec(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mvec) {
		//m_node->SetIndex(static_cast<unsigned int>(vnodes.size()) + 1);
		V_vert_nodes.push_back(mvec);
	}

    //
    // FEM functions
    //
	/// Get the state data, in a vector with as many elements as Gauss points.
    //std::vector<std::unique_ptr<ChBeamMaterialInternalData>>& GetStateData() { return statev; }
    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetSection(std::shared_ptr<ChBeamSectionCBLCON> my_section) { section = my_section; }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionCBLCON> GetSection() { return section; }

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
    /// hence do not update the corotated reference. Just for benchmarks!
    void SetDisableCorotate(bool md) { disable_corotate = md; }


    /// Set this as true to force the tangent stiffness matrix to be
    /// inexact, but symmetric. This allows the use of faster solvers. For systems close to
    /// the equilibrium, the tangent stiffness would be symmetric anyway.
    void SetForceSymmetricStiffness(bool md) { force_symmetric_stiffness = md; }

    /// Set this as false to disable the contribution of geometric stiffness 
    /// to the total tangent stiffness. By default it is on.
    void SetUseGeometricStiffness(bool md) { this->use_geometric_stiffness = md; }


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
    void ShapeFunctions(ShapeVector& N, double eta);
	
	void ComputeAmatrix( Amatrix& A, chrono::ChVector3d X , chrono::ChVector3d XI );

    virtual void Update() override;

    /// Compute large rotation of element for corotational approach
    /// The reference frame of this Euler-Bernoulli beam has X aligned to two nodes and Y parallel to Y of 1st node
    virtual void UpdateRotation() override;
    //
    // The default implementation in ChElementBase is ok, but inefficient because it passes
    // through the computation of the M mass matrix via ComputeKRMmatricesGlobal(H,0,0,M).
    virtual void EleIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& error, const double c){};
    
    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// Given that this element includes rotations at nodes, this gives:
    ///  {x_a y_a z_a Rx_a Ry_a Rz_a x_b y_b z_b Rx_b Ry_b Rz_b}
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    /// Fills the Ddt vector with the current time derivatives of field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// Give that this element includes rotations at nodes, this gives:
    ///  {v_a v_a v_a wx_a wy_a wz_a v_b v_b v_b wx_b wy_b wz_b}
    void GetField_dt(ChVectorDynamic<>& mD_dt);
	
	///
	///
	/// Compute strain at a CBLCON facet 
	///
	void ComputeStrainIncrement(ChVectorN<double, 12>& displ_incr, ChVector3d& mStrain, ChVector3d& curvature);
	///
	///
	/// Compute stress at a CBLCON facet . TODO : seems unused
	///
	void ComputeStress(ChVector3d& mstress);
	///
	///	
	virtual void ComputeMmatrixGlobal(ChMatrixRef M) override;
    /// Computes the local (material) stiffness matrix of the element:
    /// K = integral( [B]' * [D] * [B] ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed, so the explicit result of quadrature is used.
    /// Also, this local material stiffness matrix is constant, computed only at the beginning 
    /// for performance reasons; if you later change some material property, call this or InitialSetup().
    void ComputeStiffnessMatrix();

    /// Computes the local geometric stiffness Kg of the element.
    /// Note: this->Kg will be set as the geometric stiffness EXCLUDING the multiplication by the P pull force,
    /// in fact P multiplication happens in all terms, thus this allows making the Kg as a constant matrix that
    /// is computed only at the beginning, and later it is multiplied by P all times the real Kg is needed.
    /// If you later change some material property, call this or InitialSetup().
    void ComputeGeometricStiffnessMatrix();

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (e.g. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    /// Compute gravity forces, grouped in the Fg vector, one node after the other
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector3d& G_acc) override;

    //
    // Beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta, ChVector3d& u_displ, ChVector3d& u_rotaz) override;

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are corotated (expressed in world reference)
    virtual void EvaluateSectionFrame(const double eta, ChVector3d& point, ChQuaternion<>& rot) override;

    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector3d& Fforce, ChVector3d& Mtorque) override;

    /* To be completed: Created to be consistent with base class implementation*/
    virtual void EvaluateSectionStrain(const double eta, ChVector3d& StrainV) override {}

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)


    //virtual void EleDoIntegration() override {
    //    for (size_t i = 0; i < this->statev.size(); ++i) {
    //        this->statev_old[i]->Copy(*this->statev[i]);
    //    }
    //}
    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 2 * 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 2 * 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field: here the {x, y, z, rx, ry, rz} displacement
    virtual unsigned int GetNumFieldCoords() override { return 6; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 2; }

    /// Get the offset of the specified sub-block of DOFs in global vector.    
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return nodes[nblock]->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !nodes[nblock]->IsFixed(); }

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
	
    
    void SetLargeDeflection(bool mtrue) { LargeDeflection=mtrue; }
    bool GetLargeDeflection() { return LargeDeflection; }
    //
    void SetLumpedMass(bool mtrue) { LumpedMass=mtrue; }
    bool GetLumpedMass() { return LumpedMass; }
    //
    void SetEnableCoupleForces(bool mtrue) { EnableCoupleForces=mtrue; }
    bool GetEnableCoupleForces() { return EnableCoupleForces; }
	
	ChMatrixNM<double, 1, 9> ComputeMacroStressContribution();

    bool use_numerical_diff_for_KR = false;

  public:
    /// Initial setup. Precompute mass and matrices that do not change during the simulation, such as the local tangent
    /// stiffness Kl of each element, if needed, etc.
    virtual void SetupInitial(ChSystem* system) override;

    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes;

    std::shared_ptr<ChBeamSectionCBLCON> section;
	
    std::vector<std::vector<std::shared_ptr<ChNodeFEAxyzrot>>> V_vert_nodes;

    ChMatrixDynamic<> Km;  ///< local material  stiffness matrix
    ChMatrixDynamic<> Kg;  ///< local geometric stiffness matrix NORMALIZED by P

    ChQuaternion<> q_refrotA;
    ChQuaternion<> q_refrotB;

    ChQuaternion<> q_element_abs_rot;
    ChQuaternion<> q_element_ref_rot;
    
    ChVectorN<double, 12> dofs_increment;
    ChVectorN<double, 12> dofs_old;

    bool disable_corotate;
    bool force_symmetric_stiffness;

    bool use_geometric_stiffness;
    
    static bool LargeDeflection;
    static bool EnableCoupleForces;
    static bool LumpedMass;		
	
	std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain;
    
  
};



/// @} wood_elements

}  // end namespace wood
}  // end namespace chrono

#endif
