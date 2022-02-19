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
// Authors: Mike Taylor, Antonio Recuero
// =============================================================================

#ifndef CHNODEFEAXYZDDD_H
#define CHNODEFEAXYZDDD_H

#include "chrono/solver/ChVariablesGenericDiagonalMass.h"
#include "chrono/fea/ChNodeFEAxyzDD.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_nodes
/// @{

/// Class for a generic 3D finite element node, with x,y,z displacement, and 3 position vector gradients
/// For a Fully Parameterized ANCF element:
///   The variable D represents the position vector gradient with respect to the 1st element coordinate line
///   The variable DD represents the position vector gradient with respect to the 2nd element coordinate line
///   The variable DDD represents the position vector gradient with respect to the 3rd element coordinate line
class ChApi ChNodeFEAxyzDDD : public ChNodeFEAxyzDD {
  public:
    ChNodeFEAxyzDDD(ChVector<> initial_pos = VNULL, ChVector<> initial_dir_u = VECT_X, ChVector<> initial_dir_v = VECT_Y, ChVector<> initial_dir_w = VECT_Z);
    ChNodeFEAxyzDDD(const ChNodeFEAxyzDDD& other);
    ~ChNodeFEAxyzDDD();

    ChNodeFEAxyzDDD& operator=(const ChNodeFEAxyzDDD& other);

    /// Set the direction
    void SetDDD(ChVector<> mDDD) { DDD = mDDD; }
    /// Get the direction
    const ChVector<>& GetDDD() const { return DDD; }

    /// Set the direction speed
    void SetDDD_dt(ChVector<> mDDD) { DDD_dt = mDDD; }
    /// Get the direction speed
    const ChVector<>& GetDDD_dt() const { return DDD_dt; }

    /// Set the direction acceleration
    void SetDDD_dtdt(ChVector<> mDDD) { DDD_dtdt = mDDD; }
    /// Get the direction acceleration
    const ChVector<>& GetDDD_dtdt() const { return DDD_dtdt; }

    ChVariables& Variables_DDD() { return *variables_DDD; }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override;

    /// Get mass of the node (for DDD variables).
    virtual ChVectorDynamic<>& GetMassDiagonalDDD() { return variables_DDD->GetMassDiagonal(); }
    /// Sets the 'fixed' state of the node. If true, it does not move
    /// respect to the absolute world, despite constraints, forces, etc.
    virtual void SetFixed(bool mev) override;

    /// Gets the 'fixed' state of the node.
    virtual bool GetFixed() override { return variables_DDD->IsDisabled(); }

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() const override { return 12; }

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
    virtual void NodeIntStateGetIncrement(const unsigned int off_x,
                                       const ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& Dv) override;
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
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mDDD) override {
        mDDD.segment(block_offset + 0, 3) = pos.eigen();
        mDDD.segment(block_offset + 3, 3) = D.eigen();
        mDDD.segment(block_offset + 6, 3) = DD.eigen();
		mDDD.segment(block_offset + 9, 3) = DDD.eigen();
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mDDD) override {
        mDDD.segment(block_offset + 0, 3) = pos_dt.eigen();
        mDDD.segment(block_offset + 3, 3) = D_dt.eigen();
        mDDD.segment(block_offset + 6, 3) = DD_dt.eigen();
        mDDD.segment(block_offset + 9, 3) = DDD_dt.eigen();
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        this->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, etc. Here is 12: xyz displ + 3 position vector gradients
    virtual int Get_field_ncoords() override { return 12; }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 12; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        mvars.push_back(&Variables());
        mvars.push_back(&Variables_D());
        mvars.push_back(&Variables_DD());
		mvars.push_back(&Variables_DDD());
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
    ChVariablesGenericDiagonalMass* variables_DDD;

  public:
    ChVector<> DDD;
    ChVector<> DDD_dt;
    ChVector<> DDD_dtdt;
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
