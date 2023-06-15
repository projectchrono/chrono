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

#ifndef CHNODEFEAXYZROT_H
#define CHNODEFEAXYZROT_H

#include "chrono/core/ChFrameMoving.h"
#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_nodes
/// @{

/// Class for a generic ED finite element node, with x,y,z displacement and a 3D rotation.
/// This is the typical node that can be used for beams, etc.
class ChApi ChNodeFEAxyzrot : public ChNodeFEAbase, public ChBodyFrame, public ChVariableTupleCarrier_1vars<6>, public ChLoadableUVW {
  public:
    ChNodeFEAxyzrot(ChFrame<> initialf = ChFrame<>());
    ChNodeFEAxyzrot(const ChNodeFEAxyzrot& other);
    ~ChNodeFEAxyzrot() {}

    ChNodeFEAxyzrot& operator=(const ChNodeFEAxyzrot& other);

    virtual ChVariables& Variables() override { return variables; }

    /// Set the rest position as the actual position.
    virtual void Relax() override {
        X0 = *this;
        SetNoSpeedNoAcceleration();
    }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override;

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) override;

    /// Return true if the node is fixed (i.e., its state variables are not changed by the solver).
    virtual bool IsFixed() const override;

    /// Get atomic mass of the node.
    double GetMass() { return variables.GetBodyMass(); }
    /// Set atomic mass of the node.
    void SetMass(double mm) { variables.SetBodyMass(mm); }

    /// Access atomic inertia of the node.
    ChMatrix33<>& GetInertia() { return variables.GetBodyInertia(); }

    /// Set the initial (reference) frame
    void SetX0(ChFrame<> mx) { X0 = mx; }
    /// Get the initial (reference) frame
    const ChFrame<>& GetX0() const { return X0; }
    /// Access  the initial (reference) frame
    ChFrame<>& GetX0ref() { return X0; }

    /// Set the 3d applied force, in absolute reference
    void SetForce(ChVector<> mf) { Force = mf; }
    /// Get the 3d applied force, in absolute reference
    const ChVector<>& GetForce() const { return Force; }

    /// Set the 3d applied torque, in node reference
    void SetTorque(ChVector<> mf) { Torque = mf; }
    /// Get the 3d applied torque, in node reference
    const ChVector<>& GetTorque() const { return Torque; }

    /// Access the frame of the node - in absolute csys,
    /// with infos on actual position, speed, acceleration, etc.
    ChFrameMoving<>& Frame() { return *this; }

    /// Get the number of degrees of freedom (7 because quaternion for rotation).
    virtual int GetNdofX() const override { return 7; }

    /// Get the number of degrees of freedom, derivative (6 because angular velocity for rotation derivative).
    virtual int GetNdofW() const override { return 6; }

    // INTERFACE to ChVariableTupleCarrier_1vars

    virtual ChVariables* GetVariables1() override { return &Variables(); }

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

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;

    // INTERFACE to ChLoadableUVW

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, etc. Here is 6: xyz displ + xyz rots
    virtual int Get_field_ncoords() override { return 6; }

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return 1; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return this->NodeGetOffsetW(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return true; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate Q=N'*F, for Q generalized lagrangian load, where N is some type of matrix evaluated at point P(U,V,W)
    /// assumed in absolute coordinates, and F is a load assumed in absolute coordinates. det[J] is unused.
    virtual void ComputeNF(
        const double U,              ///< x coordinate of application point in absolute space
        const double V,              ///< y coordinate of application point in absolute space
        const double W,              ///< z coordinate of application point in absolute space
        ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
        double& detJ,                ///< Return det[J] here
        const ChVectorDynamic<>& F,  ///< Input F vector, size is 6, it is {Force,Torque} both in absolute coords.
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
        ) override;

    /// This is not needed because not used in quadrature.
    virtual double GetDensity() override { return 1; }

    // SERIALIZATION

    virtual void ArchiveOut(ChArchiveOut& marchive) override;
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    ChVariablesBodyOwnMass variables;  ///< 3D node variables, with x,y,z displ. and 3D rot.
    ChFrame<> X0;                      ///< reference frame
    ChVector<> Force;                  ///< applied force
    ChVector<> Torque;                 ///< applied torque
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
