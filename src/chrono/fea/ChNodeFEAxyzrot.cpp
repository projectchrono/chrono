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

#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzrot::ChNodeFEAxyzrot(ChFrame<> initialf) : Force(VNULL), Torque(VNULL) {
    this->Frame() = initialf;
    X0 = ChFrame<>(initialf);

    variables.SetBodyMass(0.0);
    variables.GetBodyInertia().setZero();
}

ChNodeFEAxyzrot::ChNodeFEAxyzrot(const ChNodeFEAxyzrot& other) : ChNodeFEAbase(other), ChBodyFrame(other) {
    X0 = other.X0;

    Force = other.Force;
    Torque = other.Torque;

    variables = other.variables;
}

ChNodeFEAxyzrot& ChNodeFEAxyzrot::operator=(const ChNodeFEAxyzrot& other) {
    if (&other == this)
        return *this;

    ChNodeFEAbase::operator=(other);
    ChBodyFrame::operator=(other);

    X0 = other.X0;

    Force = other.Force;
    Torque = other.Torque;

    variables = other.variables;

    return *this;
}

void ChNodeFEAxyzrot::SetNoSpeedNoAcceleration() {
    this->GetPos_dt() = VNULL;
    this->GetRot_dtdt() = QNULL;
    this->GetPos_dtdt() = VNULL;
    this->GetRot_dtdt() = QNULL;
}

void ChNodeFEAxyzrot::SetFixed(bool fixed) {
    variables.SetDisabled(fixed);
}

bool ChNodeFEAxyzrot::IsFixed() const {
    return variables.IsDisabled();
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzrot::NodeIntStateGather(const unsigned int off_x,
                                         ChState& x,
                                         const unsigned int off_v,
                                         ChStateDelta& v,
                                         double& T) {
    x.segment(off_x + 0, 3) = this->coord.pos.eigen();
    x.segment(off_x + 3, 4) = this->coord.rot.eigen();

    v.segment(off_v + 0, 3) = this->coord_dt.pos.eigen();
    v.segment(off_v + 3, 3) = this->GetWvel_loc().eigen();
}

void ChNodeFEAxyzrot::NodeIntStateScatter(const unsigned int off_x,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          const ChStateDelta& v,
                                          const double T) {
    this->SetCoord(x.segment(off_x, 7));
    this->SetPos_dt(v.segment(off_v, 3));
    this->SetWvel_loc(v.segment(off_v + 3, 3));
}

void ChNodeFEAxyzrot::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.segment(off_a + 0, 3) = this->coord_dtdt.pos.eigen();
    a.segment(off_a + 3, 3) = this->GetWacc_loc().eigen();
}

void ChNodeFEAxyzrot::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    this->SetPos_dtdt(a.segment(off_a, 3));
    this->SetWacc_loc(a.segment(off_a + 3, 3));
}

void ChNodeFEAxyzrot::NodeIntStateIncrement(const unsigned int off_x,
                                            ChState& x_new,
                                            const ChState& x,
                                            const unsigned int off_v,
                                            const ChStateDelta& Dv) {
    x_new(off_x) = x(off_x) + Dv(off_v);
    x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
    x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);
    
    // ADVANCE ROTATION: R_new = DR_a * R_old 
    // (using quaternions, local or abs:  q_new = Dq_a * q_old =  q_old * Dq_l  )
    ChQuaternion<> q_old(x.segment(off_x + 3, 4));
    ChQuaternion<> rel_q; rel_q.Q_from_Rotv(Dv.segment(off_v + 3, 3));
    ChQuaternion<> q_new = q_old * rel_q;
    x_new.segment(off_x + 3, 4) = q_new.eigen();
}

void ChNodeFEAxyzrot::NodeIntStateGetIncrement(const unsigned int off_x,
                                            const ChState& x_new,
                                            const ChState& x,
                                            const unsigned int off_v,
                                            ChStateDelta& Dv) {
    // POSITION:
    Dv(off_v) = x_new(off_x) - x(off_x);
    Dv(off_v + 1) = x_new(off_x + 1) - x(off_x + 1);
    Dv(off_v + 2) = x_new(off_x + 2) - x(off_x + 2);

    // ROTATION (quaternions): Dq_loc = q_old^-1 * q_new,
    //  because   q_new = Dq_abs * q_old   = q_old * Dq_loc
    ChQuaternion<> q_old(x.segment(off_x + 3, 4));
    ChQuaternion<> q_new(x_new.segment(off_x + 3, 4));
    ChQuaternion<> rel_q = q_old.GetConjugate() % q_new;
    Dv.segment(off_v + 3, 3) = rel_q.Q_to_Rotv().eigen();
}

void ChNodeFEAxyzrot::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    ChVector<> gyro = Vcross(this->GetWvel_loc(), (variables.GetBodyInertia() * this->GetWvel_loc()));

    R.segment(off + 0, 3) += c * Force.eigen();
    R.segment(off + 3, 3) += c * (Torque - gyro).eigen();
}

void ChNodeFEAxyzrot::NodeIntLoadResidual_Mv(const unsigned int off,
                                             ChVectorDynamic<>& R,
                                             const ChVectorDynamic<>& w,
                                             const double c) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
    ChVector<> Iw(GetInertia() * w.segment(off + 3, 3));
    R.segment(off + 3, 3) += c * Iw.eigen();
}

void ChNodeFEAxyzrot::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    variables.Get_qb() = v.segment(off_v, 6);
    variables.Get_fb() = R.segment(off_v, 6);
}

void ChNodeFEAxyzrot::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    v.segment(off_v, 6) = variables.Get_qb();
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzrot::InjectVariables(ChSystemDescriptor& mdescriptor) {
    mdescriptor.InsertVariables(&variables);
}

void ChNodeFEAxyzrot::VariablesFbReset() {
    variables.Get_fb().setZero();
}

void ChNodeFEAxyzrot::VariablesFbLoadForces(double factor) {
    ChVector<> gyro = Vcross(this->GetWvel_loc(), variables.GetBodyInertia() * this->GetWvel_loc());

    variables.Get_fb().segment(0, 3) = factor * Force.eigen();
    variables.Get_fb().segment(3, 3) = factor * (Torque - gyro).eigen();
}

void ChNodeFEAxyzrot::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variables.Get_qb().segment(0, 3) = this->GetCoord_dt().pos.eigen();
    variables.Get_qb().segment(3, 3) = this->GetWvel_loc().eigen();
}

void ChNodeFEAxyzrot::VariablesQbSetSpeed(double step) {
    ChCoordsys<> old_coord_dt = this->GetCoord_dt();

    // from 'qb' vector, sets body speed, and updates auxiliary data
    this->SetPos_dt(this->variables.Get_qb().segment(0, 3));
    this->SetWvel_loc(this->variables.Get_qb().segment(3, 3));

    // apply limits (if in speed clamping mode) to speeds.
    // ClampSpeed();

    // Compute accel. by BDF (approximate by differentiation);
    if (step) {
        this->SetPos_dtdt((this->GetCoord_dt().pos - old_coord_dt.pos) / step);
        this->SetRot_dtdt((this->GetCoord_dt().rot - old_coord_dt.rot) / step);
    }
}

void ChNodeFEAxyzrot::VariablesFbIncrementMq() {
    variables.Compute_inc_Mb_v(variables.Get_fb(), variables.Get_qb());
}

void ChNodeFEAxyzrot::VariablesQbIncrementPosition(double step) {
    // if (!this->IsActive())
    //	return;

    // Updates position with incremental action of speed contained in the
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

    ChVector<> newspeed(variables.Get_qb().segment(0, 3));
    ChVector<> newwel(variables.Get_qb().segment(3, 3));

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


// -----------------------------------------------------------------------------

void ChNodeFEAxyzrot::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&this->Variables());
}

void ChNodeFEAxyzrot::LoadableStateIncrement(const unsigned int off_x,
                                    ChState& x_new,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& Dv) {
    this->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
}

void ChNodeFEAxyzrot::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = this->GetCoord().pos.eigen();
    mD.segment(block_offset + 3, 4) = this->GetCoord().rot.eigen();
}

void ChNodeFEAxyzrot::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = this->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = this->GetWvel_loc().eigen();
}

void ChNodeFEAxyzrot::ComputeNF(
    const double U,              // x coordinate of application point in absolute space
    const double V,              // y coordinate of application point in absolute space
    const double W,              // z coordinate of application point in absolute space
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is 6, it is {Force,Torque} both in absolute coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ChVector<> abs_pos(U, V, W);
    ChVector<> absF(F.segment(0, 3));
    ChVector<> absT(F.segment(3, 3));
    ChVector<> body_absF;
    ChVector<> body_locT;
    ChCoordsys<> nodecoord;
    if (state_x)
        nodecoord = state_x->segment(0, 7);  // the numerical jacobian algo might change state_x
    else
        nodecoord = this->coord;

    // compute Q components F,T, given current state of 'nodecoord'. Note T in Q is in local csys, F is an abs csys
    body_absF = absF;
    body_locT = nodecoord.rot.RotateBack(absT + ((abs_pos - nodecoord.pos) % absF));
    Qi.segment(0, 3) = body_absF.eigen();
    Qi.segment(3, 3) = body_locT.eigen();
    detJ = 1;  // not needed because not used in quadrature.
}


// -----------------------------------------------------------------------------

void ChNodeFEAxyzrot::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAxyzrot>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOut(marchive);
    // serialize parent class
    ChBodyFrame::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(X0);
    marchive << CHNVP(Force);
    marchive << CHNVP(Torque);
}

void ChNodeFEAxyzrot::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChNodeFEAxyzrot>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIn(marchive);
    // serialize parent class
    ChBodyFrame::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(X0);
    marchive >> CHNVP(Force);
    marchive >> CHNVP(Torque);
}

}  // end namespace fea
}  // end namespace chrono
