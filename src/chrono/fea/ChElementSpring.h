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

#ifndef CHELEMENTSPRING_H
#define CHELEMENTSPRING_H

#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Simple finite element with two nodes and a spring/damper between the two nodes.
/// This element is mass-less, so if used in dynamic analysis, the two nodes must
/// be set with non-zero point mass.
class ChApi ChElementSpring : public ChElementGeneric {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    double spring_k;
    double damper_r;

  public:
    ChElementSpring();
    virtual ~ChElementSpring();

    virtual int GetNnodes() override { return 2; }
    virtual int GetNdofs() override { return 2 * 3; }
    virtual int GetNodeNdofs(int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA, std::shared_ptr<ChNodeFEAxyz> nodeB);

    //
    // FEA functions
    //

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override;

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    /// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.
    /// (**Not needed for the spring element because global K is computed on-the-fly in ComputeAddKRmatricesGlobal() )
    virtual void SetupInitial(ChSystem* system) override {}

    //
    // Custom properties functions
    //

    /// Set the stiffness of the spring that connects the two nodes (N/m)
    virtual void SetSpringK(double ms) { spring_k = ms; }
    virtual double GetSpringK() { return spring_k; }

    /// Set the damping of the damper that connects the two nodes (Ns/M)
    virtual void SetDamperR(double md) { damper_r = md; }
    virtual double GetDamperR() { return damper_r; }

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
