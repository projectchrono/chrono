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
// Class for LDPM elements:  
//
//  i)   Internal forces
//  ii)  Stiffness matrix
//  iii) Mass matrix  
//  iv)  Body forces
//
// Formulation of the LDPM element can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

#ifndef CH_ELEMENT_LDPM_H
#define CH_ELEMENT_LDPM_H

#include <cmath>

#include "chrono_ldpm/ChLdpmApi.h"
//#include "chrono_ldpm/ChElementRVE.h"
#include "chrono_ldpm/ChElementTetrahedron_6DOFs.h"
#include "chrono_ldpm/ChSectionLDPM.h"
#include "chrono_ldpm/ChMaterialVECT.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChElementCorotational.h"
#include "chrono/fea/ChContinuumPoisson3D.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTensors.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace ldpm {

/// @addtogroup ldpm_elements
/// @{

/// Tetrahedron FEA element with 4 nodes.
/// This is a classical element with linear displacement, hence with constant stress and constant strain. It can be
/// easily used for 3D FEA problems.


class ChLdpmApi ChElementLDPM : public ChElementTetrahedron_6DOFs,
                                    public fea::ChElementGeneric,
                                    public fea::ChElementCorotational,
                                    public ChLoadableUVW {
  public:
    static chrono::ChMatrixNM<int,12,2> facetNodeNums;
    using ShapeVector = ChMatrixNM<double, 1, 4>;
    using Amatrix = ChMatrixNM<double,3,6> ;

    ChElementLDPM();
    ~ChElementLDPM();

    virtual unsigned int GetNumNodes() override { return 4; }
    virtual unsigned int GetNumCoordsPosLevel() override { return 4 * 6; }
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int n) override { return 6; }

    double GetVolume() { return Volume; }
    void SetVolume(double Vol) {Volume=Vol;}

    virtual std::shared_ptr<fea::ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; }

    /// Return the specified tetrahedron node (0 <= n <= 3).
   virtual std::shared_ptr<fea::ChNodeFEAxyzrot> GetTetrahedronNode(unsigned int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<fea::ChNodeFEAxyzrot> nodeA,
                          std::shared_ptr<fea::ChNodeFEAxyzrot> nodeB,
                          std::shared_ptr<fea::ChNodeFEAxyzrot> nodeC,
                          std::shared_ptr<fea::ChNodeFEAxyzrot> nodeD);
                          
                          
    int GetVertNodeVec_Size() { return V_vert_nodes.size(); }
	
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot>> GetVertNodeVec(int n) { return this->V_vert_nodes[n]; }
    void SetVertNodeVec(std::vector<std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot>>> mVvert) { V_vert_nodes=mVvert; }
    void AddVertNodeVec(std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot>> mvec) {		
		V_vert_nodes.push_back(mvec);
    }
	
    //
    // FEM functions
    //

    /// Update element at each time step.
    virtual void Update() override;
	//
	// The default implementation in ChElementBase is ok, but inefficient because it passes
    // through the computation of the M mass matrix via ComputeKRMmatricesGlobal(H,0,0,M).
    virtual void EleIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& error, const double c);
    
    /// Fills the N shape function matrix with the
    /// values of shape functions at r,s,t 'volume' coordinates, where
    /// r=1 at 2nd vertex, s=1 at 3rd, t = 1 at 4th. All ranging in [0...1].
    /// The last (u, u=1 at 1st vertex) is computed form the first 3 as 1.0-r-s-t.
    /// NOTE! actually N should be a 3row, 12 column sparse matrix,
    /// as  N = [n1*eye(3) n2*eye(3) n3*eye(3) n4*eye(3)]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the n1 n2 n3 n4 values in a 1 row, 4 columns matrix.
    void ShapeFunctions(ShapeVector& N, double r, double s, double t);
    
    void ComputeAmatrix( Amatrix& A, chrono::ChVector3d X , chrono::ChVector3d XI );
	
    /// Fills the D vector (displacement) with the currentfield values at the nodes of the element, with proper
    /// ordering. If the D vector has not the size of this->GetNdofs(), it will be resized.For corotational elements,
    /// field is assumed in local reference!
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;
    
    void GetLatticeStateBlock(unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mD);
    
    void GetField_dt(ChVectorDynamic<>& mD_dt);
    
    void GetLatticeField_dt(unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mD_dt);
    
    double GetCurrentTimeIncrement(ChSystem* sys) const { return sys->GetStep();};

    double ComputeVolume();
    
    ///
    ///
    /// Compute strain at a LDPM facet
    ///
    void ComputeStrain(std::shared_ptr<ChSectionLDPM> facet, unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mStrain);
    ///
    ///
    /// Compute stress at a CSL facet 
    ///
    void ComputeStress(std::shared_ptr<ChSectionLDPM> facet,unsigned int& ind, unsigned int& jnd, ChVectorDynamic<>& mstress);
    ///
    ///

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    virtual void ComputeStiffnessMatrix();

    /// compute large rotation of element for corotational approach
    virtual void UpdateRotation() override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;
    //
    virtual void ComputeMmatrixGlobal(ChMatrixRef M) override ;
    //
    // Custom properties functions
    //
    /// Set and Get the section & material of the element
    void SetSection(std::vector<std::shared_ptr<ChSectionLDPM>> section) { my_section = section; }    
    std::vector<std::shared_ptr<ChSectionLDPM>> GetSection() { return my_section; }

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChMaterialVECT> my_material) { Material = my_material; }
    std::shared_ptr<ChMaterialVECT> GetMaterial() { return Material; }
    
    // Set and Get the individual facet information
    void AddFacetI(std::shared_ptr<ChSectionLDPM> my_facet) { my_section.push_back(my_facet); }    
    std::shared_ptr<ChSectionLDPM> GetFacetI(int i) { return my_section[i]; }

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    const ChMatrixDynamic<>& GetMatrB() const { return MatrB; }
    const ChMatrixDynamic<>& GetStiffnessMatrix() const { return StiffnessMatrix; }

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

    /// Number of coordinates in the interpolated field: here the {x, y, z, rx, ry, rz} displacement
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
    virtual double GetDensity() override { return this->GetSection()[0]->Get_material()->Get_density(); }

    /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
    /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return false; }
    
    void SetLargeDeflection(bool mtrue) { LargeDeflection=mtrue; }
    bool GetLargeDeflection() { return LargeDeflection; }
    
    double ComputeTetVol( ChVector3d p1, ChVector3d p2, ChVector3d p3, ChVector3d p4);
    
    ChMatrixNM<double,6,6> ComputeTetMassN(std::shared_ptr<ChSectionLDPM> facet, ChVector3d pN, ChVector3d pC, ChVector3d pA, ChVector3d pB);
	
	//std::vector<ChMatrixNM<double,3,9>> ComputeProjectionMatrix();
	
	ChMatrixNM<double, 1, 9> ComputeMacroStressContribution();

  private:    
	
	/// Initial setup: set up the element's parameters and matrices
    virtual void SetupInitial(ChSystem* system) override;
	
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot> > nodes;
    std::vector<std::shared_ptr<ChSectionLDPM>> my_section;
    std::shared_ptr<ChMaterialVECT> Material;
    ChMatrixDynamic<> MatrB;            // matrix of shape function's partial derivatives
    ChMatrixDynamic<> StiffnessMatrix;  // undeformed local stiffness matrix
    ChMatrixNM<double, 4, 4> mM;        // for speeding up corotational approach	
    double Volume;
    bool LargeDeflection=false; 
    
    //ChSystem* mysystem;
    
    std::vector<std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot>>> V_vert_nodes;  
	
	//template <typename T>
	//friend class ChElementRVE;
  public:	
    ChVectorDynamic<> DUn_1;
    ChVectorDynamic<> Un_1;
	std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain;
  public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



// -----------------------------------------------------------------------------

/*
/// Tetrahedron FEM element with 4 nodes for scalar fields (for Poisson-like problems).
/// This is a classical element with linear displacement.
/// ***EXPERIMENTAL***
class ChLdpmApi ChElementTetraCorot_4_P : public ChElementGeneric, public ChElementCorotational, public ChLoadableUVW {
  public:
    using ShapeVector = ChMatrixNM<double, 1, 4>;

    ChElementTetraCorot_4_P();
    ~ChElementTetraCorot_4_P() {}

    virtual int GetNnodes() override { return 4; }
    virtual int GetNdofs() override { return 4 * 1; }
    virtual int GetNodeNdofs(int n) override { return 1; }

    double GetVolume() { return Volume; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzP> nodeA,
                          std::shared_ptr<ChNodeFEAxyzP> nodeB,
                          std::shared_ptr<ChNodeFEAxyzP> nodeC,
                          std::shared_ptr<ChNodeFEAxyzP> nodeD);

    //
    // FEM functions
    //

    /// Update element at each time step.
    virtual void Update() override;

    /// Fills the N shape function matrix with the
    /// values of shape functions at zi parametric coordinates, where
    /// z0=1 at 1st vertex, z1=1 at second, z2 = 1 at third (volumetric shape functions).
    /// The 4th is computed form the first 3.  All ranging in [0...1].
    /// NOTE! actually N should be a 3row, 12 column sparse matrix,
    /// as  N = [n1*eye(3) n2*eye(3) n3*eye(3) n4*eye(3)]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the n1 n2 n3 n4 values in a 1 row, 4 columns matrix!
    virtual void ShapeFunctions(ShapeVector& N, double z0, double z1, double z2);

    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering. If the D
    /// vector has not the size of this->GetNdofs(), it will be resized. For corotational elements, field is assumed in
    /// local reference!
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    double ComputeVolume();

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    virtual void ComputeStiffnessMatrix();

    // compute large rotation of element for corotational approach
    // Btw: NOT really needed for Poisson problems
    virtual void UpdateRotation() override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal 'pseudo-forces' and set values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    //
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumPoisson3D> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumPoisson3D> GetMaterial() { return Material; }

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    const ChMatrixDynamic<>& GetMatrB() const { return MatrB; }
    const ChMatrixDynamic<>& GetStiffnessMatrix() const { return StiffnessMatrix; }

    /// Returns the gradient of P (note that the tetrahedron 4 nodes is a linear
    /// element, thus the gradient is constant in the entire volume).
    /// It is in the original undeformed unrotated reference.
    ChVectorN<double, 3> GetPgradient();

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

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffsetW(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 1; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return !nodes[nblock]->IsFixed(); }

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

  private:
    /// Initial setup: set up the element's parameters and matrices
    virtual void SetupInitial(ChSystem* system) override;

    std::vector<std::shared_ptr<ChNodeFEAxyzP> > nodes;
    std::shared_ptr<ChContinuumPoisson3D> Material;
    ChMatrixDynamic<> MatrB;            // matrix of shape function's partial derivatives
    ChMatrixDynamic<> StiffnessMatrix;  // local stiffness matrix
    ChMatrixNM<double, 4, 4> mM;        // for speeding up corotational approach
    double Volume;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
*/

/// @} fea_elements

}  // end namespace ldpm
}  // end namespace chrono

#endif
