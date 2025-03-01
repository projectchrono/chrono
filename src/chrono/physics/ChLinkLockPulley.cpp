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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cmath>

#include "chrono/physics/ChLinkLockPulley.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockPulley)

ChLinkLockPulley::ChLinkLockPulley()
    : tau(1),
      r1(1),
      r2(1),
      phase(0),
      checkphase(false),
      a1(0),
      a2(0),
      shaft_dist(0),
      belt_up1(VNULL),
      belt_up2(VNULL),
      belt_low1(VNULL),
      belt_low2(VNULL) {
    // initializes type
    local_shaft1.SetIdentity();
    local_shaft2.SetIdentity();

    // Mask: initialize our LinkMaskLF (lock formulation mask) to X  only
    mask.SetLockMask(true, false, false, false, false, false, false);
    BuildLink();
}

ChLinkLockPulley::ChLinkLockPulley(const ChLinkLockPulley& other) : ChLinkLockLock(other) {
    tau = other.tau;
    phase = other.phase;
    a1 = other.a1;
    a2 = other.a2;
    checkphase = other.checkphase;
    r1 = other.r1;
    r2 = other.r2;
    belt_up1 = other.belt_up1;
    belt_up2 = other.belt_up2;
    belt_low1 = other.belt_low1;
    belt_low2 = other.belt_low2;
    local_shaft1 = other.local_shaft1;
    local_shaft2 = other.local_shaft2;
    shaft_dist = other.shaft_dist;
}

void ChLinkLockPulley::SetRadius1(double mr) {
    r1 = mr;
    tau = r1 / r2;
}

void ChLinkLockPulley::SetRadius2(double mr) {
    r2 = mr;
    tau = r1 / r2;
}

ChVector3d ChLinkLockPulley::GetDirShaft1() {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_shaft1);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkLockPulley::GetDirShaft2() {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_shaft2);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkLockPulley::GetPosShaft1() {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_shaft1);
        return absframe.GetPos();
    } else
        return VNULL;
}

ChVector3d ChLinkLockPulley::GetPosShaft2() {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_shaft2);
        return absframe.GetPos();
    } else
        return VNULL;
}

void ChLinkLockPulley::UpdateTime(double time) {
    // First, inherit to parent class
    ChLinkLockLock::UpdateTime(time);

    ChFrame<double> abs_shaft1 = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_shaft1);
    ChFrame<double> abs_shaft2 = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_shaft2);

    ChVector3d dcc_w = Vsub(GetPosShaft2(), GetPosShaft1());

    // compute actual rotation of the two wheels (relative to truss).
    ChVector3d md1 = abs_shaft1.GetRotMat().transpose() * dcc_w;
    ChVector3d md2 = abs_shaft2.GetRotMat().transpose() * dcc_w;

    double periodic_a1 = std::atan2(md1.y(), md1.x());
    double periodic_a2 = std::atan2(md2.y(), md2.x());
    double old_a1 = a1;
    double old_a2 = a2;
    double turns_a1 = floor(old_a1 / CH_2PI);
    double turns_a2 = floor(old_a2 / CH_2PI);
    double a1U = turns_a1 * CH_2PI + periodic_a1 + CH_2PI;
    double a1M = turns_a1 * CH_2PI + periodic_a1;
    double a1L = turns_a1 * CH_2PI + periodic_a1 - CH_2PI;
    a1 = a1M;
    if (fabs(a1U - old_a1) < fabs(a1M - old_a1))
        a1 = a1U;
    if (fabs(a1L - a1) < fabs(a1M - a1))
        a1 = a1L;
    double a2U = turns_a2 * CH_2PI + periodic_a2 + CH_2PI;
    double a2M = turns_a2 * CH_2PI + periodic_a2;
    double a2L = turns_a2 * CH_2PI + periodic_a2 - CH_2PI;
    a2 = a2M;
    if (fabs(a2U - old_a2) < fabs(a2M - old_a2))
        a2 = a2U;
    if (fabs(a2L - a2) < fabs(a2M - a2))
        a2 = a2L;

    // correct marker positions if phasing is not correct
    double m_delta = 0;
    if (checkphase) {
        double realtau = tau;

        m_delta = a1 - phase - (a2 / realtau);

        if (m_delta > CH_PI)
            m_delta -= (CH_2PI);  // range -180..+180 is better than 0...360
        if (m_delta > (CH_PI / 4.0))
            m_delta = (CH_PI / 4.0);  // phase correction only in +/- 45�
        if (m_delta < -(CH_PI / 4.0))
            m_delta = -(CH_PI / 4.0);
        //// TODO
    }

    // Move markers 1 and 2 to align them as pulley ends

    ChVector3d d21_w = dcc_w - GetDirShaft1() * Vdot(GetDirShaft1(), dcc_w);
    ChVector3d D21_w = Vnorm(d21_w);

    shaft_dist = d21_w.Length();

    ChVector3d U1_w = Vcross(GetDirShaft1(), D21_w);

    double gamma1 = std::acos((r1 - r2) / shaft_dist);

    ChVector3d Ru_w = D21_w * std::cos(gamma1) + U1_w * std::sin(gamma1);
    ChVector3d Rl_w = D21_w * std::cos(gamma1) - U1_w * std::sin(gamma1);

    belt_up1 = GetPosShaft1() + Ru_w * r1;
    belt_low1 = GetPosShaft1() + Rl_w * r1;
    belt_up2 = GetPosShaft1() + d21_w + Ru_w * r2;
    belt_low2 = GetPosShaft1() + d21_w + Rl_w * r2;

    // marker alignment
    ChVector3d Dxu = Vnorm(belt_up2 - belt_up1);
    ChVector3d Dyu = Ru_w;
    ChVector3d Dzu = Vnorm(Vcross(Dxu, Dyu));
    Dyu = Vnorm(Vcross(Dzu, Dxu));
    ChMatrix33<> maU(Dxu, Dyu, Dzu);

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::MotionType::EXTERNAL);
    marker1->SetMotionType(ChMarker::MotionType::EXTERNAL);

    // move marker1 in proper position
    marker1->ImposeAbsoluteTransform(ChFrame<>(this->belt_up1, maU.GetQuaternion()));

    // move marker2 in proper position
    marker2->ImposeAbsoluteTransform(ChFrame<>(this->belt_up2, maU.GetQuaternion()));

    double phase_correction_up = m_delta * r1;
    double hU = Vlength(belt_up2 - belt_up1) + phase_correction_up;

    // imposed relative positions/speeds
    deltaC.pos = ChVector3d(-hU, 0, 0);
    deltaC_dt.pos = VNULL;
    deltaC_dtdt.pos = VNULL;

    deltaC.rot = QUNIT;  // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}

void ChLinkLockPulley::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockPulley>();

    // serialize parent class
    ChLinkLockLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(tau);
    archive_out << CHNVP(phase);
    archive_out << CHNVP(checkphase);
    archive_out << CHNVP(a1);
    archive_out << CHNVP(a2);
    archive_out << CHNVP(r1);
    archive_out << CHNVP(r2);
    archive_out << CHNVP(local_shaft1);
    archive_out << CHNVP(local_shaft2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLockPulley::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockPulley>();

    // deserialize parent class
    ChLinkLockLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(tau);
    archive_in >> CHNVP(phase);
    archive_in >> CHNVP(checkphase);
    archive_in >> CHNVP(a1);
    archive_in >> CHNVP(a2);
    archive_in >> CHNVP(r1);
    archive_in >> CHNVP(r2);
    archive_in >> CHNVP(local_shaft1);
    archive_in >> CHNVP(local_shaft2);
}

}  // end namespace chrono
