//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Andrea Favali, Alessandro Tasora

#ifndef CHNODEFEAXYZ_H
#define CHNODEFEAXYZ_H

#include "chrono/lcp/ChLcpVariablesNode.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

// Forward declaration
class ChMesh;

/// Class for a generic 3D finite element node, with x,y,z displacement.
/// This is the typical node that can be used for tetahedrons, etc.
class ChApiFea ChNodeFEAxyz : public ChNodeFEAbase, public ChNodeXYZ {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChNodeFEAxyz, ChNodeXYZ);

  public:
    ChNodeFEAxyz(ChVector<> initial_pos = VNULL) {
        X0 = initial_pos;
        pos = initial_pos;
        Force = VNULL;
        variables.SetNodeMass(0.0);
    }

    virtual ~ChNodeFEAxyz() {}

    ChNodeFEAxyz(const ChNodeFEAxyz& other) : ChNodeFEAbase(other), ChNodeXYZ(other) {
        this->X0 = other.X0;
        this->Force = other.Force;
        this->variables = other.variables;
    }

    ChNodeFEAxyz& operator=(const ChNodeFEAxyz& other) {
        if (&other == this)
            return *this;

        ChNodeFEAbase::operator=(other);
        ChNodeFEAxyz::operator=(other);

        this->X0 = other.X0;
        this->Force = other.Force;
        this->variables = other.variables;
        return *this;
    }

    virtual ChLcpVariablesNode& Variables() override { return this->variables; }

    /// Set the rest position as the actual position.
    virtual void Relax() override {
        X0 = this->pos;
        this->SetNoSpeedNoAcceleration();
    }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override {
        this->pos_dt = VNULL;
        this->pos_dtdt = VNULL;
    }

    /// Set the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual void SetFixed(bool mev) override { variables.SetDisabled(mev); }
    /// Get the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual bool GetFixed() override {return variables.IsDisabled(); }

    /// Get mass of the node.
    virtual double GetMass() const override { return this->variables.GetNodeMass(); }
    /// Set mass of the node.
    virtual void SetMass(double mm) override { this->variables.SetNodeMass(mm); }

    /// Set the initial (reference) position
    virtual void SetX0(ChVector<> mx) { X0 = mx; }
    /// Get the initial (reference) position
    virtual ChVector<> GetX0() { return X0; }

    /// Set the 3d applied force, in absolute reference
    virtual void SetForce(ChVector<> mf) { Force = mf; }
    /// Get the 3d applied force, in absolute reference
    virtual ChVector<> GetForce() { return Force; }

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() override { return 3; }

    //
    // Functions for interfacing to the state bookkeeping
    //

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override {
        x.PasteVector(this->pos, off_x, 0);
        v.PasteVector(this->pos_dt, off_v, 0);
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override {
        this->SetPos(x.ClipVector(off_x, 0));
        this->SetPos_dt(v.ClipVector(off_v, 0));
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {
        a.PasteVector(this->pos_dtdt, off_a, 0);
    }

    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {
        this->SetPos_dtdt(a.ClipVector(off_a, 0));
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
        R.PasteSumVector(this->Force * c, off, 0);
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override {
        R(off + 0) += c * GetMass() * w(off + 0);
        R(off + 1) += c * GetMass() * w(off + 1);
        R(off + 2) += c * GetMass() * w(off + 2);
    }

    virtual void NodeIntToLCP(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) override {
        this->variables.Get_qb().PasteClippedMatrix(&v, off_v, 0, 3, 1, 0, 0);
        this->variables.Get_fb().PasteClippedMatrix(&R, off_v, 0, 3, 1, 0, 0);
    }

    virtual void NodeIntFromLCP(const unsigned int off_v, ChStateDelta& v) override {
        v.PasteMatrix(&this->variables.Get_qb(), off_v, 0);
    }

    //
    // Functions for interfacing to the LCP solver
    //

    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor) override {
        mdescriptor.InsertVariables(&this->variables);
    }

    virtual void VariablesFbReset() override { this->variables.Get_fb().FillElem(0.0); }

    virtual void VariablesFbLoadForces(double factor = 1) override {
        this->variables.Get_fb().PasteSumVector(this->Force * factor, 0, 0);
    }

    virtual void VariablesQbLoadSpeed() override { this->variables.Get_qb().PasteVector(this->pos_dt, 0, 0); }

    virtual void VariablesQbSetSpeed(double step = 0) override {
        ChVector<> old_dt = this->pos_dt;
        this->SetPos_dt(this->variables.Get_qb().ClipVector(0, 0));
        if (step) {
            this->SetPos_dtdt((this->pos_dt - old_dt) / step);
        }
    }

    virtual void VariablesFbIncrementMq() override {
        this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
    }

    virtual void VariablesQbIncrementPosition(double step) override {
        ChVector<> newspeed = variables.Get_qb().ClipVector(0, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        this->SetPos(this->GetPos() + newspeed * step);
    }

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChNodeFEAbase::ArchiveOUT(marchive);
        // serialize parent class
        ChNodeXYZ::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(X0);
        marchive << CHNVP(Force);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChNodeFEAbase::ArchiveIN(marchive);
        // serialize parent class
        ChNodeXYZ::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(X0);
        marchive >> CHNVP(Force);
    }

  protected:
    ChLcpVariablesNode variables;  /// 3D node variables, with x,y,z

    ChVector<> X0;     ///< reference position
    ChVector<> Force;  ///< applied force
};

}  // end namespace fea
}  // end namespace chrono

#endif
