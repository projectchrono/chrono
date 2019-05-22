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

#ifndef CHELEMENTBAR_H
#define CHELEMENTBAR_H

#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Simple finite element with two nodes and a bar that connects them.
/// No bending and torsion stiffness, just like a bar with two spherical joints.
/// In practical terms, this element works a bit like the class ChElementSpring,
/// but also adds mass along the element, hence point-like mass in the two nodes
/// is not needed.
class ChApi ChElementBar : public ChElementGeneric {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    double area;
    double density;
    double E;
    double rdamping;
    double mass;
    double length;

  public:
    ChElementBar();
    virtual ~ChElementBar();

    virtual int GetNnodes() override { return 2; }
    virtual int GetNdofs() override { return 2 * 3; }
    virtual int GetNodeNdofs(int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA, std::shared_ptr<ChNodeFEAxyz> nodeB);

    //
    // FEM functions
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

    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.
    virtual void SetupInitial(ChSystem* system) override;

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override;

    //
    // Custom properties functions
    //

    /// Set the cross sectional area of the bar (m^2) (also changes stiffness keeping same E modulus)
    void SetBarArea(double ma) { this->area = ma; }
    double GetBarArea() { return this->area; }

    /// Set the density of the bar (kg/m^3)
    void SetBarDensity(double md) { this->density = md; }
    double GetBarDensity() { return this->density; }

    /// Set the Young elastic modulus (N/m^2) (also sets stiffness)
    void SetBarYoungModulus(double mE) { this->E = mE; }
    double GetBarYoungModulus() { return this->E; }

    /// Set the Rayleigh damping ratio r (as in: R = r * K )
    void SetBarRaleyghDamping(double mr) { this->rdamping = mr; }
    double GetBarRaleyghDamping() { return this->rdamping; }

    /// The full mass of the bar
    double GetMass() { return this->mass; }

    /// The rest length of the bar
    double GetRestLength() { return this->length; }

    /// The current length of the bar (might be after deformation)
    double GetCurrentLength() { return (nodes[1]->GetPos() - nodes[0]->GetPos()).Length(); }

    /// Get the strain epsilon, after deformation.
    double GetStrain() { return (GetCurrentLength() - GetRestLength()) / GetRestLength(); }

    /// Get the stress sigma, after deformation.
    double GetStress() { return GetBarYoungModulus() * GetStrain(); }

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
