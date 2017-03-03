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

#ifndef CHNODEFEAXYZ_H
#define CHNODEFEAXYZ_H

#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/solver/ChVariablesNode.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

// Forward declaration
class ChMesh;

/// Class for a generic 3D finite element node, with x,y,z displacement.
/// This is the typical node that can be used for tetahedrons, etc.
class ChApiFea ChNodeFEAxyz : public ChNodeFEAbase, public ChNodeXYZ, public ChVariableTupleCarrier_1vars<3> {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChNodeFEAxyz)

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

    /// Set the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual void SetFixed(bool mev) override { variables.SetDisabled(mev); }
    /// Get the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual bool GetFixed() override { return variables.IsDisabled(); }

    /// Get mass of the node.
    virtual double GetMass() const override { return variables.GetNodeMass(); }
    /// Set mass of the node.
    virtual void SetMass(double mm) override { variables.SetNodeMass(mm); }

    /// Set the initial (reference) position
    virtual void SetX0(ChVector<> mx) { X0 = mx; }
    /// Get the initial (reference) position
    virtual ChVector<> GetX0() { return X0; }

    /// Set the 3d applied force, in absolute reference
    virtual void SetForce(ChVector<> mf) { Force = mf; }
    /// Get the 3d applied force, in absolute reference
    virtual ChVector<> GetForce() { return Force; }

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() const override { return 3; }

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
                                    double& T) override {
        x.PasteVector(pos, off_x, 0);
        v.PasteVector(pos_dt, off_v, 0);
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override {
        SetPos(x.ClipVector(off_x, 0));
        SetPos_dt(v.ClipVector(off_v, 0));
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {
        a.PasteVector(pos_dtdt, off_a, 0);
    }

    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {
        SetPos_dtdt(a.ClipVector(off_a, 0));
    }

    virtual void NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) override {
        x_new(off_x) = x(off_x) + Dv(off_v);
        x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
        x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);
    }

    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override {
        R.PasteSumVector(Force * c, off, 0);
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override {
        R(off + 0) += c * GetMass() * w(off + 0);
        R(off + 1) += c * GetMass() * w(off + 1);
        R(off + 2) += c * GetMass() * w(off + 2);
    }

    virtual void NodeIntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R) override {
        variables.Get_qb().PasteClippedMatrix(v, off_v, 0, 3, 1, 0, 0);
        variables.Get_fb().PasteClippedMatrix(R, off_v, 0, 3, 1, 0, 0);
    }

    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) override {
        v.PasteMatrix(variables.Get_qb(), off_v, 0);
    }

    //
    // Functions for interfacing to the solver
    //

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override { mdescriptor.InsertVariables(&variables); }

    virtual void VariablesFbReset() override { variables.Get_fb().FillElem(0.0); }

    virtual void VariablesFbLoadForces(double factor = 1) override {
        variables.Get_fb().PasteSumVector(Force * factor, 0, 0);
    }

    virtual void VariablesQbLoadSpeed() override { variables.Get_qb().PasteVector(pos_dt, 0, 0); }

    virtual void VariablesQbSetSpeed(double step = 0) override {
        ChVector<> old_dt = pos_dt;
        SetPos_dt(variables.Get_qb().ClipVector(0, 0));
        if (step) {
            SetPos_dtdt((pos_dt - old_dt) / step);
        }
    }

    virtual void VariablesFbIncrementMq() override {
        variables.Compute_inc_Mb_v(variables.Get_fb(), variables.Get_qb());
    }

    virtual void VariablesQbIncrementPosition(double step) override {
        ChVector<> newspeed = variables.Get_qb().ClipVector(0, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        SetPos(GetPos() + newspeed * step);
    }

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    ChVariablesNode variables;  /// 3D node variables, with x,y,z
    ChVector<> X0;              ///< reference position
    ChVector<> Force;           ///< applied force
};

}  // end namespace fea
}  // end namespace chrono

#endif
