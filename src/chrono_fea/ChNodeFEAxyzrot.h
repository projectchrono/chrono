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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHNODEFEAXYZROT_H
#define CHNODEFEAXYZROT_H

#include "chrono/core/ChFrameMoving.h"
#include "chrono/physics/ChBodyFrame.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// Class for a generic ED finite element node, with x,y,z displacement and a 3D rotation.
/// This is the typical node that can be used for beams, etc.
class ChApiFea ChNodeFEAxyzrot : public ChNodeFEAbase, public ChBodyFrame, public ChVariableTupleCarrier_1vars<6> {
  public:
    ChNodeFEAxyzrot(ChFrame<> initialf = ChFrame<>());
    ChNodeFEAxyzrot(const ChNodeFEAxyzrot& other);
    ~ChNodeFEAxyzrot() {}

    ChNodeFEAxyzrot& operator=(const ChNodeFEAxyzrot& other);

    virtual ChVariables& Variables() override { return variables; }

    virtual ChVariablesBodyOwnMass& VariablesBody() override { return variables; }

    /// Set the rest position as the actual position.
    virtual void Relax() override {
        X0 = *this;
        SetNoSpeedNoAcceleration();
    }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override;

    /// Set the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual void SetFixed(bool mev) override { variables.SetDisabled(mev); }
    /// Get the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual bool GetFixed() override { return variables.IsDisabled(); }

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

    /// Set the 3d applied torque, in absolute reference
    void SetTorque(ChVector<> mf) { Torque = mf; }
    /// Get the 3d applied torque, in absolute reference
    const ChVector<>& GetTorque() const { return Torque; }

    /// Access the frame of the node - in absolute csys,
    /// with infos on actual position, speed, acceleration, etc.
    ChFrameMoving<>& Frame() { return *this; }

    /// Get the number of degrees of freedom (7 because quaternion for rotation).
    virtual int Get_ndof_x() const override { return 7; }

    /// Get the number of degrees of freedom, derivative (6 because angular velocity for rotation derivative).
    virtual int Get_ndof_w() const override { return 6; }

    //
    // INTERFACE to ChVariableTupleCarrier_1vars
    //
    virtual ChVariables* GetVariables1() override { return &Variables(); }

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
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    ChVariablesBodyOwnMass variables;  ///< 3D node variables, with x,y,z displ. and 3D rot.
    ChFrame<> X0;                      ///< reference frame
    ChVector<> Force;                  ///< applied force
    ChVector<> Torque;                 ///< applied torque
};

}  // end namespace fea
}  // end namespace chrono

#endif
