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
// Authors: Antonio Recuero
// =============================================================================

#ifndef CHNODEFEAXYZDD_H
#define CHNODEFEAXYZDD_H

#include "chrono/solver/ChVariablesGenericDiagonalMass.h"
#include "chrono_fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fea {

/// Class for a generic 3D finite element node, with x,y,z displacement, direction, and one curvature vector OR
/// additional direction.
/// The variable DD represents the derivative of a gradient vector or an additional gradient, to be used in ANCF
/// elements.

class ChApiFea ChNodeFEAxyzDD : public ChNodeFEAxyzD {
  public:
    ChNodeFEAxyzDD(ChVector<> initial_pos = VNULL, ChVector<> initial_dir = VECT_X, ChVector<> initial_curv = VNULL);
    ChNodeFEAxyzDD(const ChNodeFEAxyzDD& other);
    ~ChNodeFEAxyzDD();

    ChNodeFEAxyzDD& operator=(const ChNodeFEAxyzDD& other);

    /// Set the direction
    void SetDD(ChVector<> mDD) { DD = mDD; }
    /// Get the direction
    const ChVector<>& GetDD() const { return DD; }

    /// Set the direction speed
    void SetDD_dt(ChVector<> mDD) { DD_dt = mDD; }
    /// Get the direction speed
    const ChVector<>& GetDD_dt() const { return DD_dt; }

    /// Set the direction acceleration
    void SetDD_dtdt(ChVector<> mDD) { DD_dtdt = mDD; }
    /// Get the direction acceleration
    const ChVector<>& GetDD_dtdt() const { return DD_dtdt; }

    ChVariables& Variables_DD() { return *variables_DD; }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override;

    /// Get mass of the node (for DD variables).
    virtual ChVectorDynamic<>& GetMassDiagonalDD() { return variables_DD->GetMassDiagonal(); }
    /// Sets the 'fixed' state of the node. If true, it does not move
    /// respect to the absolute world, despite constraints, forces, etc.
    virtual void SetFixed(bool mev) override;

    /// Gets the 'fixed' state of the node.
    virtual bool GetFixed() override { return variables_DD->IsDisabled(); }

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() const override { return 9; }

    //
    // Functions for interfacing to the state bookkeeping
    //

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override;
    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override;
    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) override;
    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override;
    virtual void NodeIntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R) override;
    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) override;

    //
    // Functions for interfacing to the solver
    //

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;

    //
    // INTERFACE to ChLoadable
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 9; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 9; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mDD) override {
        mDD.PasteVector(pos, block_offset, 0);
        mDD.PasteVector(D, block_offset + 3, 0);
        mDD.PasteVector(DD, block_offset + 6, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mDD) override {
        mDD.PasteVector(pos_dt, block_offset, 0);
        mDD.PasteVector(D_dt, block_offset + 3, 0);
        mDD.PasteVector(DD_dt, block_offset + 6, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        this->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, etc. Here is 9: xyz displ + 2 gradients or 1 gradient + 1 curv. vector
    virtual int Get_field_ncoords() override { return 9; }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 9; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        mvars.push_back(&Variables());
        mvars.push_back(&Variables_D());
        mvars.push_back(&Variables_DD());
    }

    /// Evaluate Q=N'*F , for Q generalized lagrangian load, where N is some type of matrix
    /// evaluated at point P(U,V,W) assumed in absolute coordinates, and
    /// F is a load assumed in absolute coordinates.
    /// The det[J] is unused.
    virtual void ComputeNF(
        const double U,              ///< x coordinate of application point in absolute space
        const double V,              ///< y coordinate of application point in absolute space
        const double W,              ///< z coordinate of application point in absolute space
        ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
        double& detJ,                ///< Return det[J] here
        const ChVectorDynamic<>& F,  ///< Input F vector, containing Force xyz in absolute coords and a 'pseudo' torque.
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
        ) override;

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    /// 3D node variable - the direction part: Dx,Dy,Dz (the position part is in parent class)
    ChVariablesGenericDiagonalMass* variables_DD;

  public:
    ChVector<> DD;
    ChVector<> DD_dt;
    ChVector<> DD_dtdt;
};

}  // end namespace fea
}  // end namespace chrono

#endif
