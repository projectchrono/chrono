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

#ifndef CHNODEFEAXYZ_H
#define CHNODEFEAXYZ_H

#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/solver/ChVariablesNode.h"
#include "chrono/fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_nodes
/// @{

// Forward declaration
class ChMesh;

/// Class for a generic 3D finite element node, with x,y,z displacement.
/// This is the typical node that can be used for tetrahedrons, etc.
class ChApi ChNodeFEAxyz : public ChNodeFEAbase, public ChNodeXYZ, public ChVariableTupleCarrier_1vars<3> {
  public:
    ChNodeFEAxyz(ChVector<> initial_pos = VNULL);
    ChNodeFEAxyz(const ChNodeFEAxyz& other);
    virtual ~ChNodeFEAxyz() {}

    ChNodeFEAxyz& operator=(const ChNodeFEAxyz& other);

    virtual ChVariablesNode& Variables() override { return variables; }

    /// Set the rest position as the actual position.
    virtual void Relax() override;

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override;

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) override;

    /// Return true if the node is fixed (i.e., its state variables are not changed by the solver).
    virtual bool IsFixed() const override;

    /// Get mass of the node.
    virtual double GetMass() const override { return variables.GetNodeMass(); }
    /// Set mass of the node.
    virtual void SetMass(double m) override { variables.SetNodeMass(m); }

    /// Set the initial (reference) position
    virtual void SetX0(ChVector<> x) { X0 = x; }
    /// Get the initial (reference) position
    virtual ChVector<> GetX0() { return X0; }

    /// Set the 3d applied force, in absolute reference
    virtual void SetForce(ChVector<> f) { Force = f; }
    /// Get the 3d applied force, in absolute reference
    virtual ChVector<> GetForce() { return Force; }

    /// Get the number of degrees of freedom
    virtual int GetNdofX() const override { return 3; }

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

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& archive) override;
    virtual void ArchiveIN(ChArchiveIn& archive) override;

  protected:
    ChVariablesNode variables;  ///< 3D node variables, with x,y,z
    ChVector<> X0;              ///< reference position
    ChVector<> Force;           ///< applied force
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
