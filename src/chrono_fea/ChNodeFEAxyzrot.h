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
// File authors: Alessandro Tasora

#ifndef CHNODEFEAXYZROT_H
#define CHNODEFEAXYZROT_H

#include "chrono/core/ChFrameMoving.h"
#include "chrono/lcp/ChLcpVariablesBodyOwnMass.h"
#include "chrono/physics/ChBodyFrame.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// Class for a generic ED finite element node, with x,y,z displacement and a 3D rotation.
/// This is the typical node that can be used for beams, etc.
class ChApiFea ChNodeFEAxyzrot : public ChNodeFEAbase, public ChBodyFrame {
  public:
    ChNodeFEAxyzrot(ChFrame<> initialf = ChFrame<>()) {
        this->Frame() = initialf;

        X0 = ChFrame<>(initialf);

        Force = VNULL;
        Torque = VNULL;

        variables.SetBodyMass(0.0);
        variables.GetBodyInertia().FillElem(0.0);
    }

    ~ChNodeFEAxyzrot() {}

    ChNodeFEAxyzrot(const ChNodeFEAxyzrot& other) : ChNodeFEAbase(other), ChBodyFrame(other) {
        this->X0 = other.X0;

        this->Force = other.Force;
        this->Force = other.Torque;

        this->variables = other.variables;
    }

    ChNodeFEAxyzrot& operator=(const ChNodeFEAxyzrot& other) {
        if (&other == this)
            return *this;

        ChNodeFEAbase::operator=(other);
        ChBodyFrame::operator=(other);

        this->X0 = other.X0;

        this->Force = other.Force;
        this->Force = other.Torque;

        this->variables = other.variables;

        return *this;
    }

    virtual ChLcpVariables& Variables() override { return this->variables; }

    virtual ChLcpVariablesBodyOwnMass& VariablesBody() override { return this->variables; }

    /// Set the rest position as the actual position.
    virtual void Relax() override {
        this->X0 = *this;
        this->SetNoSpeedNoAcceleration();
    }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override {
        this->GetPos_dt() = VNULL;
        this->GetRot_dtdt() = QNULL;
        this->GetPos_dtdt() = VNULL;
        this->GetRot_dtdt() = QNULL;
    }

    /// Set the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual void SetFixed(bool mev) override { variables.SetDisabled(mev); }
    /// Get the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual bool GetFixed() override { return variables.IsDisabled(); }

    /// Get atomic mass of the node.
    virtual double GetMass() { return this->variables.GetBodyMass(); }
    /// Set atomic mass of the node.
    virtual void SetMass(double mm) { this->variables.SetBodyMass(mm); }

    /// Access atomic inertia of the node.
    virtual ChMatrix33<>& GetInertia() { return this->variables.GetBodyInertia(); }

    /// Set the initial (reference) frame
    virtual void SetX0(ChFrame<> mx) { X0 = mx; }
    /// Get the initial (reference) frame
    virtual ChFrame<> GetX0() { return X0; }

    /// Set the 3d applied force, in absolute reference
    virtual void SetForce(ChVector<> mf) { Force = mf; }
    /// Get the 3d applied force, in absolute reference
    virtual ChVector<> GetForce() { return Force; }

    /// Set the 3d applied torque, in absolute reference
    virtual void SetTorque(ChVector<> mf) { Torque = mf; }
    /// Get the 3d applied torque, in absolute reference
    virtual ChVector<> GetTorque() { return Torque; }

    /// Access the frame of the node - in absolute csys,
    /// with infos on actual position, speed, acceleration, etc.
    ChFrameMoving<>& Frame() { return *this; }

    /// Get the number of degrees of freedom (7 because quaternion for rotation).
    virtual int Get_ndof_x() override { return 7; }

    /// Get the number of degrees of freedom, derivative (6 because angular velocity for rotation derivative).
    virtual int Get_ndof_w() override { return 6; }

    //
    // Functions for interfacing to the state bookkeeping
    //

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override {
        x.PasteCoordsys(this->coord, off_x, 0);
        v.PasteVector(this->coord_dt.pos, off_v, 0);
        v.PasteVector(this->GetWvel_loc(), off_v + 3, 0);
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override {
        this->SetCoord(x.ClipCoordsys(off_x, 0));
        this->SetPos_dt(v.ClipVector(off_v, 0));
        this->SetWvel_loc(v.ClipVector(off_v + 3, 0));
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {
        a.PasteVector(this->coord_dtdt.pos, off_a, 0);
        a.PasteVector(this->GetWacc_loc(), off_a + 3, 0);
    }

    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {
        this->SetPos_dtdt(a.ClipVector(off_a, 0));
        this->SetWacc_loc(a.ClipVector(off_a + 3, 0));
    }

    virtual void NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) override {
        x_new(off_x) = x(off_x) + Dv(off_v);
        x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
        x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);

        ChQuaternion<> mdeltarot;
        ChQuaternion<> moldrot = x.ClipQuaternion(off_x + 3, 0);
        ChVector<> newwel_abs = Amatrix * Dv.ClipVector(off_v + 3, 0);
        double mangle = newwel_abs.Length();
        newwel_abs.Normalize();
        mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
        ChQuaternion<> mnewrot = mdeltarot * moldrot;  // quaternion product
        x_new.PasteQuaternion(mnewrot, off_x + 3, 0);
    }

    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override {
        ChVector<> gyro = Vcross(this->GetWvel_loc(), (variables.GetBodyInertia().Matr_x_Vect(this->GetWvel_loc())));

        R.PasteSumVector(this->Force * c, off, 0);
        R.PasteSumVector((this->Torque - gyro) * c, off + 3, 0);
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override {
        R(off + 0) += c * GetMass() * w(off + 0);
        R(off + 1) += c * GetMass() * w(off + 1);
        R(off + 2) += c * GetMass() * w(off + 2);
        ChVector<> Iw = GetInertia() * w.ClipVector(off + 3, 0);
        Iw *= c;
        R.PasteSumVector(Iw, off + 3, 0);
    }

    virtual void NodeIntToLCP(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) override {
        this->variables.Get_qb().PasteClippedMatrix(&v, off_v, 0, 6, 1, 0, 0);
        this->variables.Get_fb().PasteClippedMatrix(&R, off_v, 0, 6, 1, 0, 0);
    }

    virtual void NodeIntFromLCP(const unsigned int off_v, ChStateDelta& v) override {
        v.PasteMatrix(&this->variables.Get_qb(), off_v, 0);
    }

    //
    // Functions for interfacing to the LCP solver
    //

    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor) override {
        mdescriptor.InsertVariables(&this->variables);
    };

    virtual void VariablesFbReset() override { this->variables.Get_fb().FillElem(0.0); }

    virtual void VariablesFbLoadForces(double factor = 1) override {
        ChVector<> gyro = Vcross(this->GetWvel_loc(), (variables.GetBodyInertia().Matr_x_Vect(this->GetWvel_loc())));

        this->variables.Get_fb().PasteSumVector(this->Force * factor, 0, 0);
        this->variables.Get_fb().PasteSumVector((this->Torque - gyro) * factor, 3, 0);
    }

    virtual void VariablesQbLoadSpeed() override {
        // set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
        this->variables.Get_qb().PasteVector(this->GetCoord_dt().pos, 0, 0);
        this->variables.Get_qb().PasteVector(this->GetWvel_loc(), 3, 0);
    }

    virtual void VariablesQbSetSpeed(double step = 0) override {
        ChCoordsys<> old_coord_dt = this->GetCoord_dt();

        // from 'qb' vector, sets body speed, and updates auxiliary data
        this->SetPos_dt(this->variables.Get_qb().ClipVector(0, 0));
        this->SetWvel_loc(this->variables.Get_qb().ClipVector(3, 0));

        // apply limits (if in speed clamping mode) to speeds.
        // ClampSpeed();

        // Compute accel. by BDF (approximate by differentiation);
        if (step) {
            this->SetPos_dtdt((this->GetCoord_dt().pos - old_coord_dt.pos) / step);
            this->SetRot_dtdt((this->GetCoord_dt().rot - old_coord_dt.rot) / step);
        }
    }

    virtual void VariablesFbIncrementMq() override {
        this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
    }

    virtual void VariablesQbIncrementPosition(double step) override {
        // if (!this->IsActive())
        //	return;

        // Updates position with incremental action of speed contained in the
        // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

        ChVector<> newspeed = variables.Get_qb().ClipVector(0, 0);
        ChVector<> newwel = variables.Get_qb().ClipVector(3, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        this->SetPos(this->GetPos() + newspeed * step);

        // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
        ChQuaternion<> mdeltarot;
        ChQuaternion<> moldrot = this->GetRot();
        ChVector<> newwel_abs = this->Amatrix * newwel;
        double mangle = newwel_abs.Length() * step;
        newwel_abs.Normalize();
        mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
        ChQuaternion<> mnewrot = mdeltarot % moldrot;
        this->SetRot(mnewrot);
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
        ChBodyFrame::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(X0);
        marchive << CHNVP(Force);
        marchive << CHNVP(Torque);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChNodeFEAbase::ArchiveIN(marchive);
        // serialize parent class
        ChBodyFrame::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(X0);
        marchive >> CHNVP(Force);
        marchive >> CHNVP(Torque);
    }

  private:
    ChLcpVariablesBodyOwnMass variables;  /// 3D node variables, with x,y,z displ. and 3D rot.

    // ChFrameMoving<> frame;	///< frame

    ChFrame<> X0;  ///< reference frame

    ChVector<> Force;   ///< applied force
    ChVector<> Torque;  ///< applied torque
};

}  // end namespace fea
}  // end namespace chrono

#endif
