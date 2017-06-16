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

#include "chrono_fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzrot::ChNodeFEAxyzrot(ChFrame<> initialf) : Force(VNULL), Torque(VNULL) {
    this->Frame() = initialf;
    X0 = ChFrame<>(initialf);

    variables.SetBodyMass(0.0);
    variables.GetBodyInertia().FillElem(0.0);
}

ChNodeFEAxyzrot::ChNodeFEAxyzrot(const ChNodeFEAxyzrot& other) : ChNodeFEAbase(other), ChBodyFrame(other) {
    X0 = other.X0;

    Force = other.Force;
    Force = other.Torque;

    variables = other.variables;
}

ChNodeFEAxyzrot& ChNodeFEAxyzrot::operator=(const ChNodeFEAxyzrot& other) {
    if (&other == this)
        return *this;

    ChNodeFEAbase::operator=(other);
    ChBodyFrame::operator=(other);

    X0 = other.X0;

    Force = other.Force;
    Force = other.Torque;

    variables = other.variables;

    return *this;
}

void ChNodeFEAxyzrot::SetNoSpeedNoAcceleration() {
    this->GetPos_dt() = VNULL;
    this->GetRot_dtdt() = QNULL;
    this->GetPos_dtdt() = VNULL;
    this->GetRot_dtdt() = QNULL;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzrot::NodeIntStateGather(const unsigned int off_x,
                                         ChState& x,
                                         const unsigned int off_v,
                                         ChStateDelta& v,
                                         double& T) {
    x.PasteCoordsys(this->coord, off_x, 0);
    v.PasteVector(this->coord_dt.pos, off_v, 0);
    v.PasteVector(this->GetWvel_loc(), off_v + 3, 0);
}

void ChNodeFEAxyzrot::NodeIntStateScatter(const unsigned int off_x,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          const ChStateDelta& v,
                                          const double T) {
    this->SetCoord(x.ClipCoordsys(off_x, 0));
    this->SetPos_dt(v.ClipVector(off_v, 0));
    this->SetWvel_loc(v.ClipVector(off_v + 3, 0));
}

void ChNodeFEAxyzrot::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.PasteVector(this->coord_dtdt.pos, off_a, 0);
    a.PasteVector(this->GetWacc_loc(), off_a + 3, 0);
}

void ChNodeFEAxyzrot::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    this->SetPos_dtdt(a.ClipVector(off_a, 0));
    this->SetWacc_loc(a.ClipVector(off_a + 3, 0));
}

void ChNodeFEAxyzrot::NodeIntStateIncrement(const unsigned int off_x,
                                            ChState& x_new,
                                            const ChState& x,
                                            const unsigned int off_v,
                                            const ChStateDelta& Dv) {
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

void ChNodeFEAxyzrot::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    ChVector<> gyro = Vcross(this->GetWvel_loc(), (variables.GetBodyInertia().Matr_x_Vect(this->GetWvel_loc())));

    R.PasteSumVector(Force * c, off, 0);
    R.PasteSumVector((Torque - gyro) * c, off + 3, 0);
}

void ChNodeFEAxyzrot::NodeIntLoadResidual_Mv(const unsigned int off,
                                             ChVectorDynamic<>& R,
                                             const ChVectorDynamic<>& w,
                                             const double c) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
    ChVector<> Iw = GetInertia() * w.ClipVector(off + 3, 0);
    Iw *= c;
    R.PasteSumVector(Iw, off + 3, 0);
}

void ChNodeFEAxyzrot::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    variables.Get_qb().PasteClippedMatrix(v, off_v, 0, 6, 1, 0, 0);
    variables.Get_fb().PasteClippedMatrix(R, off_v, 0, 6, 1, 0, 0);
}

void ChNodeFEAxyzrot::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    v.PasteMatrix(variables.Get_qb(), off_v, 0);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzrot::InjectVariables(ChSystemDescriptor& mdescriptor) {
    mdescriptor.InsertVariables(&variables);
}

void ChNodeFEAxyzrot::VariablesFbReset() {
    variables.Get_fb().FillElem(0);
}

void ChNodeFEAxyzrot::VariablesFbLoadForces(double factor) {
    ChVector<> gyro = Vcross(this->GetWvel_loc(), (variables.GetBodyInertia().Matr_x_Vect(this->GetWvel_loc())));

    variables.Get_fb().PasteSumVector(Force * factor, 0, 0);
    variables.Get_fb().PasteSumVector((Torque - gyro) * factor, 3, 0);
}

void ChNodeFEAxyzrot::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variables.Get_qb().PasteVector(this->GetCoord_dt().pos, 0, 0);
    variables.Get_qb().PasteVector(this->GetWvel_loc(), 3, 0);
}

void ChNodeFEAxyzrot::VariablesQbSetSpeed(double step) {
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

void ChNodeFEAxyzrot::VariablesFbIncrementMq() {
    variables.Compute_inc_Mb_v(variables.Get_fb(), variables.Get_qb());
}

void ChNodeFEAxyzrot::VariablesQbIncrementPosition(double step) {
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

// -----------------------------------------------------------------------------

void ChNodeFEAxyzrot::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAxyzrot>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOUT(marchive);
    // serialize parent class
    ChBodyFrame::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(X0);
    marchive << CHNVP(Force);
    marchive << CHNVP(Torque);
}

void ChNodeFEAxyzrot::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChNodeFEAxyzrot>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIN(marchive);
    // serialize parent class
    ChBodyFrame::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(X0);
    marchive >> CHNVP(Force);
    marchive >> CHNVP(Torque);
}

}  // end namespace fea
}  // end namespace chrono
