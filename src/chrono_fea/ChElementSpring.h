// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono_fea/ChElementGeneric.h"
#include "chrono_fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Simple finite element with two nodes and a spring/damper between the two nodes.
/// This element is mass-less, so if used in dynamic analysis, the two nodes must
/// be set with non-zero point mass.
class ChApiFea ChElementSpring : public ChElementGeneric {
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

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA, std::shared_ptr<ChNodeFEAxyz> nodeB) {
        nodes[0] = nodeA;
        nodes[1] = nodeB;
        std::vector<ChVariables*> mvars;
        mvars.push_back(&nodes[0]->Variables());
        mvars.push_back(&nodes[1]->Variables());
        Kmatr.SetVariables(mvars);
    }

    //
    // FEA functions
    //

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override {
        mD.Reset(this->GetNdofs(), 1);
        mD.PasteVector(this->nodes[0]->GetPos(), 0, 0);
        mD.PasteVector(this->nodes[1]->GetPos(), 3, 0);
    }

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    /// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override {
        assert((H.GetRows() == 6) && (H.GetColumns() == 6));

        // compute stiffness matrix (this is already the explicit
        // formulation of the corotational stiffness matrix in 3D)

        ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
        ChMatrixNM<double, 3, 1> dircolumn;
        dircolumn.PasteVector(dir, 0, 0);

        ChMatrix33<> submatr;
        submatr.MatrMultiplyT(dircolumn, dircolumn);

        // note that stiffness and damping matrices are the same, so join stuff here
        double commonfactor = this->spring_k * Kfactor + this->damper_r * Rfactor;
        submatr.MatrScale(commonfactor);
        H.PasteMatrix(submatr, 0, 0);
        H.PasteMatrix(submatr, 3, 3);
        submatr.MatrNeg();
        H.PasteMatrix(submatr, 0, 3);
        H.PasteMatrix(submatr, 3, 0);

        // finally, do nothing about mass matrix because this element is mass-less
    }

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override {
        assert((Fi.GetRows() == 6) && (Fi.GetColumns() == 1));

        ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
        double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
        double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
        double L_dt = Vdot((nodes[1]->GetPos_dt() - nodes[0]->GetPos_dt()), dir);
        double internal_Kforce_local = this->spring_k * (L - L_ref);
        double internal_Rforce_local = this->damper_r * L_dt;
        double internal_force_local = internal_Kforce_local + internal_Rforce_local;
        ChVector<> int_forceA = dir * internal_force_local;
        ChVector<> int_forceB = -dir * internal_force_local;
        Fi.PasteVector(int_forceA, 0, 0);
        Fi.PasteVector(int_forceB, 3, 0);
    }

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
