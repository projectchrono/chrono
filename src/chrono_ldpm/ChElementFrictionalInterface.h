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
// Authors: Erol Lale
// =============================================================================

#ifndef CH_ELEMENT_FRICTIONAL_INTERFACE_H
#define CH_ELEMENT_FRICTIONAL_INTERFACE_H

#include "chrono/physics/ChSystemSMC.h"  
#include "chrono/physics/ChSystemNSC.h"  
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include "chrono/physics/ChLinkMate.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChMeshSurface.h"
#include "chrono/fea/ChTetrahedronFace.h"
#include "chrono/fea/ChHexahedronFace.h"
#include "chrono/fea/ChLinkNodeNode.h"
#include "chrono/fea/ChLinkNodeFrame.h"

#include "chrono_ldpm/ChElementLDPM.h"
#include "chrono_ldpm/ChLinkNodeNodeRot.h"
#include "chrono_ldpm/ChMeshSurfaceLDPM.h"
#include "chrono_ldpm/ChLDPMFace.h"

using namespace chrono;
using namespace fea;

namespace chrono {
namespace ldpm {

/// @addtogroup fea_elements
/// @{

/// Frictional interface between a triangular face and a node insede a triangular face .
//   Triangle consists of 3 ChNodeFEAxyz nodes and inner node is also ChNodeFEAxyz
/// Node inside triangular face is mass-less, so if used in dynamic analysis, the mass must
/// be set with non-zero value.
class ChLdpmApi ChElementFrictionalInterface : public ChElementGeneric,
											public ChElementCorotational,
											public ChLoadableUVW {
  public:
    ChElementFrictionalInterface();
    ~ChElementFrictionalInterface();

    virtual unsigned int GetNumNodes() override { return 4; }
    virtual unsigned int GetNumCoordsPosLevel() override { return 4 * 3; }
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA, std::shared_ptr<ChNodeFEAxyz> nodeB, 
						std::shared_ptr<ChNodeFEAxyz> nodeC, std::shared_ptr<ChNodeFEAxyz> nodeP);
						
	 //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 4 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 4 * 3; }

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

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual unsigned int GetNumFieldCoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 4; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return nodes[nblock]->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !nodes[nblock]->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    //
    // FEA functions
    //
	virtual void Update() override;
	
	// Compute large rotation of element for corotational approach
    virtual void UpdateRotation() override {};
	
	/// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return 0; }
    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNumCoordsPosLevel(), it will be resized.
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;
	
	 virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override{};

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    /// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    //
    // Custom properties functions
    //

    /// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetSpringCoefficient(double ms) { spring_k = ms; }
    virtual double GetSpringCoefficient() { return spring_k; }
	
	/// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetInitialFrictionCoefficient(double mmu0) { mu0 = mmu0; }
    virtual double GetInitialFrictionCoefficient() { return mu0; }
	
	/// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetDynamicFrictionCoefficient(double mmudyn) { mudyn = mmudyn; }
    virtual double GetDynamicFrictionCoefficient() { return mudyn; }
	
	/// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetS0Coefficient(double ms0) { s0 = ms0; }
    virtual double GetS0Coefficient() { return s0; }

    /// Set the damping of the damper that connects the two nodes (Ns/M)
    virtual void SetDampingCoefficient(double md) { damper_r = md; }
    virtual double GetDampingCoefficient() { return damper_r; }
	
	virtual void SetConstraint(std::shared_ptr<ChLinkNodeFrame> mcons) { mconstraint = mcons; }
    virtual std::shared_ptr<ChLinkNodeFrame> GetConstraint() { return mconstraint; }
	
	virtual double GetCurrentFrictionCoefficient(double Dslip){
		double mu = mudyn+(mu0-mudyn)*s0/(s0+Dslip);
		return mu;
	}

    /// Get the current force transmitted along the spring direction,
    /// including the effect of the damper. Positive if pulled. (N)
    virtual ChVector3d GetCurrentForce();
	
	//
	// Create an interface between a list of nodes and ChMeshSurfece 
	//
	virtual bool CreateInteractionNodeToTriFace(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyz> m_node, double max_dist);
	
	virtual void CreateInteractionNodeToBody(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::vector<std::shared_ptr<ChNodeFEAbase>> node_list, std::shared_ptr<ChBody> m_body);
    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

  private:
    /// Initial setup.
    /// No override needed for the spring element because global K is computed on-the-fly in
    /// ComputeAddKRmatricesGlobal()
    ////virtual void SetupInitial(ChSystem* system) override {}
	std::shared_ptr<ChLinkNodeFrame> mconstraint=nullptr;
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
	double mu0 = 0.13; // 0.13; //static friction
	double mudyn = 0.015; //0.015;	//dynamic friction
	double s0 = 1.3; //mm    
    double spring_k;
    double damper_r;	
	double N1, N2, N3;
	ChVector3d Force={0,0,0};
	ChVectorDynamic<> DUn_1;
    ChVectorDynamic<> Un_1;	
	double slip_t = 0;
	double scale = 1.0;
	
};





/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

/// Frictional interface between a triangular face and a node insede a triangular face .
//   Triangle consists of 3 ChNodeFEAxyz nodes and inner node is also ChNodeFEAxyz
/// Node inside triangular face is mass-less, so if used in dynamic analysis, the mass must
/// be set with non-zero value.
class ChLdpmApi ChElementFrictionalInterfaceRot : public ChElementGeneric,
											public ChElementCorotational,
											public ChLoadableUVW {
  public:
    ChElementFrictionalInterfaceRot();
    ~ChElementFrictionalInterfaceRot();

    virtual unsigned int GetNumNodes() override { return 4; }
    virtual unsigned int GetNumCoordsPosLevel() override { return 4 * 6; }
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB, 
						std::shared_ptr<ChNodeFEAxyzrot> nodeC, std::shared_ptr<ChNodeFEAxyzrot> nodeP);
	
	
	 //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 4 * 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 4 * 6; }

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

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual unsigned int GetNumFieldCoords() override { return 6; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 4; }

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
	
    //
    // FEA functions
    //
	virtual void Update() override;
	
	// Compute large rotation of element for corotational approach
    virtual void UpdateRotation() override {};
	
	/// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return 0; }
	
    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNumCoordsPosLevel(), it will be resized.
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;
	//
	//
	virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override{};

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    /// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    //
    // Custom properties functions
    //

    /// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetSpringCoefficient(double ms) { spring_k = ms; }
    virtual double GetSpringCoefficient() { return spring_k; }
	
	/// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetInitialFrictionCoefficient(double mmu0) { mu0 = mmu0; }
    virtual double GetInitialFrictionCoefficient() { return mu0; }
	
	/// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetDynamicFrictionCoefficient(double mmudyn) { mudyn = mmudyn; }
    virtual double GetDynamicFrictionCoefficient() { return mudyn; }
	
	/// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetS0Coefficient(double ms0) { s0 = ms0; }
    virtual double GetS0Coefficient() { return s0; }

    /// Set the damping of the damper that connects the two nodes (Ns/M)
    virtual void SetDampingCoefficient(double md) { damper_r = md; }
    virtual double GetDampingCoefficient() { return damper_r; }
	
	virtual void SetConstraint(std::shared_ptr<ChLinkMateGeneric> mcons) { mconstraint = mcons; }
    virtual std::shared_ptr<ChLinkMateGeneric> GetConstraint() { return mconstraint; }
	
	virtual double GetCurrentFrictionCoefficient(double Dslip){
		double mu = mudyn+(mu0-mudyn)*s0/(s0+Dslip);
		return mu;
	}

    /// Get the current force transmitted along the spring direction,
    /// including the effect of the damper. Positive if pulled. (N)
    virtual ChVector3d GetCurrentForce();
	
	//
	// Create an interface between a list of nodes and ChMeshSurfece 
	//
	virtual bool CreateInteractionNodeToTriFace(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyzrot> m_node, double max_dist);
	
	virtual void CreateInteractionNodeToBody(ChSystem& sys, std::shared_ptr<ChMesh>& my_mesh, std::vector<std::shared_ptr<ChNodeFEAbase>> node_list, std::shared_ptr<ChBody> m_body);
    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

  private:
    /// Initial setup.
    /// No override needed for the spring element because global K is computed on-the-fly in
    /// ComputeAddKRmatricesGlobal()
    ////virtual void SetupInitial(ChSystem* system) override {}
	std::shared_ptr<ChLinkMateGeneric> mconstraint=nullptr;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes;
	double mu0 = 0.13;//0.13; //0.13; //static friction
	double mudyn = 0.015; //0.015; //0.015;	//dynamic friction
	double s0 = 1.3; //mm    
    double spring_k;
    double damper_r;	
	double N1, N2, N3;
	ChVector3d Force={0,0,0};
	ChVectorDynamic<> DUn_1;
    ChVectorDynamic<> Un_1;	
	double slip_t = 0;
	double scale=1.0;
	
};










/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
