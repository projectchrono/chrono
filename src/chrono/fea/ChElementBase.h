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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHELEMENTBASE_H
#define CHELEMENTBASE_H

#include "chrono/physics/ChLoadable.h"
#include "chrono/core/ChMath.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono/fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

struct ChStrainStress3D {
    ChVectorN<double, 6> strain;
    ChVectorN<double, 6> stress;
};

/// Base class for all finite elements, that can be used in the ChMesh physics item.
class ChApi ChElementBase {
  public:
    ChElementBase() {}
    virtual ~ChElementBase() {}

    /// Gets the number of nodes used by this element.
    virtual int GetNnodes() = 0;

    /// Gets the number of coordinates in the field used by the referenced nodes.
    /// This is for example the size (n.of rows/columns) of the local stiffness matrix.
    virtual int GetNdofs() = 0;

    /// Get the number of coordinates from the n-th node that are used by this element.
    /// Note that this may be different from the value returned by
    ///    GetNodeN(n)->Get_ndof_w();
    virtual int GetNodeNdofs(int n) = 0;

    /// Access the nth node.
    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) = 0;

    //
    // FEM functions
    //

    /// Fills the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector size is not this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChVectorDynamic<>& mD) = 0;

    /// Sets M as the mass matrix.
    /// The matrix is expressed in global reference.
    virtual void ComputeMmatrixGlobal(ChMatrixRef M) = 0;

    /// Compute element's nodal masses.
    virtual void ComputeNodalMass() {}

    /// Sets H as the stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and mass matrix M,
    /// scaled by Mfactor. Matrices are expressed in global reference.
    /// Corotational elements can take the local Kl & Rl matrices and rotate them.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor = 0, double Mfactor = 0) = 0;

    /// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
    /// values in the Fi vector, with n.rows = n.of dof of element.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) = 0;

    /// Computes the gravitational forces and set
    /// values in the Fi vector, with n.rows = n.of dof of element.
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fi, const ChVector<>& G_acc) = 0;


    /// Update: this is called at least at each time step. If the
    /// element has to keep updated some auxiliary data, such as the rotation
    /// matrices for corotational approach, this is the proper place.
    virtual void Update() {}

    //
    // Functions for interfacing to the state bookkeeping
    //

	/// This is optionally implemented if there is some internal state
	/// that requires integration.
	virtual void EleDoIntegration() {}

    /// Adds the internal forces (pasted at global nodes offsets) into
    /// a global vector R, multiplied by a scaling factor c, as
    ///   R += forces * c
    virtual void EleIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {}

    /// Adds the product of element mass M by a vector w (pasted at global nodes offsets) into
    /// a global vector R, multiplied by a scaling factor c, as
    ///   R += M * w * c
    virtual void EleIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {}

    /// Adds the contribution of gravity loads, multiplied by a scaling factor c, as: 
    ///   R += M * g * c
    /// Note that it is up to the element implementation to build a proper g vector that 
    /// contains G_acc values in the proper stride (ex. tetahedrons have 4x copies of G_acc in g). 
    /// Note that elements can provide fast implementations that do not need to build any internal M matrix,
    /// and not even the g vector, for instance if using lumped masses. 
    virtual void EleIntLoadResidual_F_gravity(ChVectorDynamic<>& R, const ChVector<>& G_acc, const double c) = 0;


    //
    // Functions for interfacing to the solver
    //

    /// Tell to a system descriptor that there are item(s) of type
    /// ChKblock in this object (for further passing it to a solver)
    /// Basically does nothing, but inherited classes must specialize this.
    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) = 0;

    /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
    /// ChKblock item(s), if any. The K, R, M matrices are added with scaling
    /// values Kfactor, Rfactor, Mfactor.
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) = 0;

    /// Adds the internal forces, expressed as nodal forces, into the
    /// encapsulated ChVariables, in the 'fb' part: qf+=forces*factor
    /// WILL BE DEPRECATED - see EleIntLoadResidual_F
    virtual void VariablesFbLoadInternalForces(double factor = 1.) {}

    /// Adds M*q (internal masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    /// WILL BE DEPRECATED
    virtual void VariablesFbIncrementMq() {}

  private:
    /// Initial setup: This is used mostly to precompute matrices that do not change during the simulation, i.e. the
    /// local stiffness of each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

	friend class ChMesh;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
