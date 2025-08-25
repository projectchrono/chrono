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

#ifndef ChElementSpringPPP_H
#define ChElementSpringPPP_H

#include "chrono_flow/ChNodeFEAxyzPPP.h"
#include "chrono_flow/ChContinuumPoissonFlow3D.h"
#include "chrono_flow/ChFlowApi.h"

#include <cmath>

#include "chrono/fea/ChElementTetrahedron.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChElementCorotational.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/core/ChTensors.h"
#include "chrono/core/ChMatrix.h"

using namespace chrono::fea;

namespace chrono {
namespace flow {

/// @addtogroup fea_elements
/// @{

/// Simple finite element with two nodes and a spring/damper between the two nodes.
/// This element is mass-less, so if used in dynamic analysis, the two nodes must
/// be set with non-zero point mass.
class ChFlowApi ChElementSpringPPP : public ChElementGeneric, 
                               public ChElementCorotational, 
                               public ChLoadableUVW {
  public:
    using ShapeVector = ChMatrixNM<double, 1, 2>;

    ChElementSpringPPP();
    ~ChElementSpringPPP();

    virtual unsigned int GetNumNodes() override { return 2; }
    virtual unsigned int GetNumCoordsPosLevel() override { return 2 * 3; }
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzPPP> nodeA, 
                          std::shared_ptr<ChNodeFEAxyzPPP> nodeB);

    /// Update element at each time step.
    virtual void Update() override;

    /// Fills the N shape function matrix with the values of shape functions 
    /// at zi parametric coordinates, where z0=1 at 1st vertex.
    virtual void ShapeFunctions(ShapeVector& N, double z0);

    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNumCoordsPosLevel(), it will be resized.
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    virtual void GetStateBlockDt(ChVectorDynamic<>& mD);

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    virtual void ComputeStiffnessMatrix();

    /// Computes the local MASS MATRIX of the element:
    virtual void ComputeMassMatrix();

    /// compute large rotation of element for corotational approach
    virtual void UpdateRotation() override {};

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
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumPoissonFlow3D> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumPoissonFlow3D> GetMaterial() { return Material; }

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    const ChMatrixDynamic<>& GetMatrB() const { return MatrB; }
    const ChMatrixDynamic<>& GetStiffnessMatrix() const { return StiffnessMatrix; }



    /// Returns the gradient of P (note that the tetrahedron 4 nodes is a linear
    /// element, thus the gradient is constant in the entire volume).
    /// It is in the original undeformed unrotated reference.
    ChVectorN<double, 1> GetPgradient();

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 2 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 2 * 3; }

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

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Number of coordinates in the interpolated field: here the {t} temperature
    virtual unsigned int GetNumFieldCoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 6; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return nodes[nblock]->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !nodes[nblock]->IsFixed(); }

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in 0..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           );

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
    virtual double GetDensity() override { return this->Material->GetDensity(); }
    
    /// Get element state variables
    const ChVectorDynamic<>& GetElementStateVariable() { return ElementState; }

    /// Set element state variables
    void SetElementStateVariable(ChVectorDynamic<>& ElState) { ElementState = ElState; }

    /// Compute element source terms
    void ComputeSourceTerm();

    /// Set element FreeCAD geometry inputs
    void SetElementL1(double l1) { elL1 = l1; };
    void SetElementL2(double l2) { elL2 = l2; };
    void SetElementA(double a) { elA = a; };
    void SetElementVol(double v) { elVol = v; };
    void SetElementType(double type) { elType = type; };

  private:
    /// Initial setup: set up the element's parameters and matrices
    virtual void SetupInitial(ChSystem* system) override;

    std::vector<std::shared_ptr<ChNodeFEAxyzPPP>> nodes;
    std::shared_ptr<ChContinuumPoissonFlow3D> Material;
    ChMatrixDynamic<> MatrB;               // matrix of shape function's partial derivatives
    ChMatrixDynamic<> StiffnessMatrix;     // local stiffness matrix
    ChMatrixDynamic<> MassMatrix;          // local mass matrix
    ChMatrixDynamic<> UpdatedConstitutiveMatrix;  // local stiffness matrix

    //ChMatrixNM<double, 4, 4> mM;         // for speeding up corotational approach
    double Volume;
    ChVectorDynamic<> ElementState;        // Vector to store element state variables  
    ChVectorDynamic<> ElementSourceTerm;   // Vector to store element source term
    double dt_current;   // To store the timestep of current step
    
    //ChVectorDynamic<> Alpha;    // Vector to store current hydration degree
    //ChVectorDynamic<> Alpha0;   // Vector to store hydration degree of last step (for increment caluculation)

    // Element FreeCAD geometry inputs                       
    double elL1;                            
    double elL2;
    double elA;
    double elVol;
    double elType;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Hydration function
    //void UpdateHydration(double dt, double T0);
    //ChVectorDynamic<> GetAlpha() const { return Alpha; }
    void SetDt(double dt) { dt_current = dt; }
    double GetDt() const { return dt_current; }
    
    void ComputeElementVar(double dt);
    double GetAlpha() const { return ElementState.size() > 0 ? ElementState(0) : 0.0; }
    double GetAlpha_dt() const { return ElementState.size() > 0 ? ElementState(1) : 0.0; }
    double GetAlphas() const { return ElementState.size() > 0 ? ElementState(2) : 0.0; }
    double GetAlphas_dt() const { return ElementState.size() > 0 ? ElementState(3) : 0.0; }

    double GetAA() const { return ElementState.size() > 0 ? ElementState(4) : 0.0; }
    double GetAAS() const { return ElementState.size() > 0 ? ElementState(5) : 0.0; }
    double Getwe() const { return ElementState.size() > 0 ? ElementState(6) : 0.0; }
    double GetmSaturation() const { return ElementState.size() > 0 ? ElementState(7) : 0.0; }
    double GetPorosity() const { return ElementState.size() > 0 ? ElementState(8) : 0.0; }


};

/// @} fea_elements

}  // end namespace flow
}  // end namespace chrono

#endif
