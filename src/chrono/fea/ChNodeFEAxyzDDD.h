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

/// Class for a generic 3D finite element node, with x,y,z displacement, and 3 position vector derivatives.
/// For a Fully Parameterized ANCF element:
///   - The variable D represents the position vector gradient with respect to the 1st element coordinate line.
///   - The variable DD represents the position vector gradient with respect to the 2nd element coordinate line.
///   - The variable DDD represents the position vector gradient with respect to the 3rd element coordinate line.
/// Other ANCF elements may use these derivative vectors differently.
class ChApi ChNodeFEAxyzDDD : public ChNodeFEAxyzDD {
  public:
    ChNodeFEAxyzDDD(ChVector<> initial_pos = VNULL,
                    ChVector<> initial_dir_u = VECT_X,
                    ChVector<> initial_dir_v = VECT_Y,
                    ChVector<> initial_dir_w = VECT_Z);
    ChNodeFEAxyzDDD(const ChNodeFEAxyzDDD& other);
    ~ChNodeFEAxyzDDD();

    ChNodeFEAxyzDDD& operator=(const ChNodeFEAxyzDDD& other);

    /// Set the 3rd derivative vector.
    void SetDDD(const ChVector<>& d) { DDD = d; }
    /// Get the 3rd derivative vector.
    const ChVector<>& GetDDD() const { return DDD; }

    /// Set the speed of the 3rd derivative vector.
    void SetDDD_dt(const ChVector<>& dt) { DDD_dt = dt; }
    /// Get the speed of the 3rd derivative vector.
    const ChVector<>& GetDDD_dt() const { return DDD_dt; }

    /// Set the acceleration of the 3rd derivative vector.
    void SetDDD_dtdt(const ChVector<>& dtt) { DDD_dtdt = dtt; }
    /// Get the  acceleration of the 3rd derivative vector.
    const ChVector<>& GetDDD_dtdt() const { return DDD_dtdt; }

    ChVariables& Variables_DDD() { return *variables_DDD; }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override;

    /// Get mass of the node (for DDD variables).
    ChVectorDynamic<>& GetMassDiagonalDDD() { return variables_DDD->GetMassDiagonal(); }

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) override;

    /// Return true if the node is fixed (i.e., its state variables are not changed by the solver).
    virtual bool IsFixed() const override;

    /// Fix/release the 3rd derivative vector states.
    /// If fixed, these states are not changed by the solver. Note that releasing the 3rd derivative vector forces the
    /// first 2 derivatives vector to also be released.
    void SetFixedDDD(bool fixed);

    /// Return true if the 3rd derivative vector states are fixed.
    bool IsFixedDDD() const;

    /// Get the number of degrees of freedom.
    virtual int GetNdofX() const override { return 12; }

    /// Get the actual number of active degrees of freedom.
    virtual int GetNdofX_active() const override { return m_dof_actual; }

    // Functions for interfacing to the state bookkeeping

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

    // Functions for interfacing to the solver

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;

    // INTERFACE to ChLoadable

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return m_dof_actual; }

    /// Gets the number of DOFs affected by this element (speed part).
    virtual int LoadableGet_ndof_w() override { return m_dof_actual; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& S) override;

    /// Gets all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& S) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field.
    virtual int Get_field_ncoords() override { return m_dof_actual; }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return m_dof_actual; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& vars) override;

    /// Evaluate Q = N'*F, for Q generalized lagrangian load, where N is some type of matrix evaluated at point P(U,V,W)
    /// assumed in absolute coordinates, and F is a load assumed in absolute coordinates. Here, det[J] is unused.
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

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& archive) override;
    virtual void ArchiveIN(ChArchiveIn& archive) override;

  protected:
    /// Initial setup. Set number of degrees of freedom for this node.
    virtual void SetupInitial(ChSystem* system) override;

  private:
    ChVariablesGenericDiagonalMass* variables_DDD;  ///< 3rd derivative vector
    ChVector<> DDD;
    ChVector<> DDD_dt;
    ChVector<> DDD_dtdt;
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
