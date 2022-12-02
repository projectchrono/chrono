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

#ifndef CHELEMENTGENERIC_H
#define CHELEMENTGENERIC_H

#include "chrono/solver/ChKblockGeneric.h"
#include "chrono/solver/ChVariablesNode.h"
#include "chrono/fea/ChElementBase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Class for all elements whose stiffness matrix can be seen as an NxN block-matrix split among N nodes.
/// Most FEA elements inherited from ChElementGeneric need to implement at most the two fundamental methods
/// ComputeKRMmatricesGlobal(), ComputeInternalForces(), and optionally ComputeGravityForces().
class ChApi ChElementGeneric : public ChElementBase {
  public:
    ChElementGeneric() {}
    virtual ~ChElementGeneric() {}

    /// Access the proxy to stiffness, for sparse solver
    ChKblockGeneric& Kstiffness() { return Kmatr; }

    // Functions for interfacing to the state bookkeeping

    /// Add the internal forces (pasted at global nodes offsets) into
    /// a global vector R, multiplied by a scaling factor c, as
    ///   R += forces * c
    /// This default implementation is SLIGHTLY INEFFICIENT.
    virtual void EleIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override;

    /// Add the product of element mass M by a vector w (pasted at global nodes offsets) into
    /// a global vector R, multiplied by a scaling factor c, as
    ///   R += M * w * c
    /// This default implementation is VERY INEFFICIENT.
    virtual void EleIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override;

    /// Add the contribution of gravity loads, multiplied by a scaling factor c, as:
    ///   R += M * g * c
    /// This default implementation is VERY INEFFICIENT.
    /// This fallback implementation uses a temp ChLoaderGravity that applies the load to elements
    /// only if they are inherited by ChLoadableUVW so it can use GetDensity() and Gauss quadrature.
    virtual void EleIntLoadResidual_F_gravity(ChVectorDynamic<>& R, const ChVector<>& G_acc, const double c) override;

    // FEM functions

    /// Compute the gravitational forces.
    /// This default implementation (POTENTIALLY INEFFICIENT) uses a temporary ChLoaderGravity that applies the load to
    /// elements only if they are inherited by ChLoadableUVW so it can use GetDensity() and Gauss quadrature.
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) override;

    /// Calculate the mass matrix, expressed in global reference.
    /// This default implementation (POTENTIALLY VERY INEFFICIENT) should be overriden by derived classes a more
    /// efficient version.
    virtual void ComputeMmatrixGlobal(ChMatrixRef M) override;

    // Functions for interfacing to the solver

    /// Tell to a system descriptor that there are item(s) of type
    /// ChKblock in this object (for further passing it to a solver)
    virtual void InjectKRMmatrices(ChSystemDescriptor& descriptor) override;

    /// Add the current stiffness K and damping R and mass M matrices in encapsulated ChKblock item(s), if any.
    /// The K, R, M matrices are load with scaling values Kfactor, Rfactor, Mfactor.
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

    /// Add the internal forces, expressed as nodal forces, into the encapsulated ChVariables.
    virtual void VariablesFbLoadInternalForces(double factor = 1.) override;

    /// Add M*q (internal masses multiplied current 'qb').
    virtual void VariablesFbIncrementMq() override;

  protected:
    ChKblockGeneric Kmatr;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
