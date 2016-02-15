//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Andrea Favali, Alessandro Tasora

#ifndef CHELEMENTBASE_H
#define CHELEMENTBASE_H

#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/core/ChMath.h"
#include "chrono/lcp/ChLcpSystemDescriptor.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for all finite elements, that can be
/// used in the ChMesh physics item.
class ChApiFea ChElementBase {
  protected:
  public:
    ChElementBase(){};
    virtual ~ChElementBase(){};

    /// Gets the number of coordinates of the node positions in space;
    /// note this is not the coordinates of the field, use GetNdofs() instead
    virtual int GetNcoords() = 0;

    /// Gets the number of nodes used by this element
    virtual int GetNnodes() = 0;

    /// Gets the number of coordinates in the field used by the referenced nodes,
    /// this is for example the size (n.of rows/columns) of the local stiffness matrix
    virtual int GetNdofs() = 0;

    /// Access the nth node
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) = 0;

    //
    // FEM functions
    //

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) = 0;

    /// Sets M as the mass matrix.
    /// The matrix is expressed in global reference.
    /// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
    virtual void ComputeMmatrixGlobal(ChMatrix<>& M) = 0;

    /// Sets H as the stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and mass matrix M,
    /// scaled by Mfactor. Matrices are expressed in global reference.
    /// Corotational elements can take the local Kl & Rl matrices and rotate them.
    /// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) = 0;

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector, whith n.rows = n.of dof of element.
    /// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) = 0;

    /// Initial setup: This is used mostly to precompute matrices
    /// that do not change during the simulation, i.e. the local
    /// stiffness of each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

    /// Update: this is called at least at each time step. If the
    /// element has to keep updated some auxiliary data, such as the rotation
    /// matrices for corotational approach, this is the proper place.
    virtual void Update() {}

    //
    // Functions for interfacing to the state bookkeeping
    //

    /// Adds the internal forces (pasted at global nodes offsets) into
    /// a global vector R, multiplied by a scaling factor c, as
    ///   R += forces * c
    virtual void EleIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {}

    /// Adds the product of element mass M by a vector w (pasted at global nodes offsets) into
    /// a global vector R, multiplied by a scaling factor c, as
    ///   R += M * v * c
    virtual void EleIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {}

    //
    // Functions for interfacing to the LCP solver
    //

    /// Tell to a system descriptor that there are item(s) of type
    /// ChLcpKblock in this object (for further passing it to a LCP solver)
    /// Basically does nothing, but inherited classes must specialize this.
    virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) = 0;

    /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
    /// ChLcpKblock item(s), if any. The K, R, M matrices are added with scaling
    /// values Kfactor, Rfactor, Mfactor.
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) = 0;

    /// Adds the internal forces, expressed as nodal forces, into the
    /// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
    /// WILL BE DEPRECATED - see EleIntLoadResidual_F
    virtual void VariablesFbLoadInternalForces(double factor = 1.) {}

    /// Adds M*q (internal masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    /// WILL BE DEPRECATED
    virtual void VariablesFbIncrementMq() {}
};

/// @} fea_elements

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
