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

#include "chrono/physics/ChLinkLockGear.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockGear)

ChLinkLockGear::ChLinkLockGear()
    : tau(1),
      alpha(0),
      beta(0),
      phase(0),
      checkphase(false),
      epicyclic(false),
      a1(0),
      a2(0),
      r1(0),
      r2(0),
      contact_pt(VNULL) {
    local_shaft1.SetIdentity();
    local_shaft2.SetIdentity();

    // Mask: initialize our LinkMaskLF (lock formulation mask) to X  only
    mask.SetLockMask(true, false, false, false, false, false, false);
    BuildLink();
}

ChLinkLockGear::ChLinkLockGear(const ChLinkLockGear& other) : ChLinkLock(other) {
    tau = other.tau;
    alpha = other.alpha;
    beta = other.beta;
    phase = other.phase;
    a1 = other.a1;
    a2 = other.a2;
    epicyclic = other.epicyclic;
    checkphase = other.checkphase;
    r1 = other.r1;
    r2 = other.r2;
    contact_pt = other.contact_pt;
    local_shaft1 = other.local_shaft1;
    local_shaft2 = other.local_shaft2;
}

ChVector3d ChLinkLockGear::GetDirShaft1() const {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_shaft1);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkLockGear::GetDirShaft2() const {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_shaft2);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkLockGear::GetPosShaft1() const {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_shaft1);
        return absframe.GetPos();
    } else
        return VNULL;
}

ChVector3d ChLinkLockGear::GetPosShaft2() const {
    if (m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_shaft2);
        return absframe.GetPos();
    } else
        return VNULL;
}

void ChLinkLockGear::UpdateTime(double time) {
    // First, inherit to parent class
    ChLinkLock::UpdateTime(time);

    // Move markers 1 and 2 to align them as gear teeth

    ChVector3d mx;
    ChVector3d my;
    ChVector3d mz;
    ChVector3d mr;
    ChVector3d mmark1;
    ChVector3d mmark2;
    ChVector3d lastX;
    ChVector3d vrota;

    ChFrame<double> abs_shaft1 = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_shaft1);
    ChFrame<double> abs_shaft2 = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_shaft2);

    ChVector3d vbdist = Vsub(GetPosShaft1(), GetPosShaft2());
    ////ChVector3d Trad1 = Vnorm(Vcross(GetDirShaft1(), Vnorm(Vcross(GetDirShaft1(), vbdist))));
    ////ChVector3d Trad2 = Vnorm(Vcross(Vnorm(Vcross(GetDirShaft2(), vbdist)), GetDirShaft2()));

    double bdist = Vlength(vbdist);

    // compute actual rotation of the two wheels (relative to truss).
    ChVector3d md1 = abs_shaft1.GetRotMat().transpose() * (-vbdist);
    ChVector3d md2 = abs_shaft2.GetRotMat().transpose() * (-vbdist);

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

    // compute new markers coordsystem alignment
    my = Vnorm(vbdist);
    mz = GetDirShaft1();
    mx = Vnorm(Vcross(my, mz));
    mr = Vnorm(Vcross(mz, mx));
    mz = Vnorm(Vcross(mx, my));
    ChVector3d mz2, mx2, mr2, my2;
    my2 = my;
    mz2 = GetDirShaft2();
    mx2 = Vnorm(Vcross(my2, mz2));
    mr2 = Vnorm(Vcross(mz2, mx2));

    ChMatrix33<> ma1(mx, my, mz);

    // rotate csys because of beta
    vrota.x() = 0.0;
    vrota.y() = beta;
    vrota.z() = 0.0;
    ChMatrix33<> mrotma;
    mrotma.SetFromCardanAnglesXYZ(vrota);
    ChMatrix33<> marot_beta = ma1 * mrotma;
    // rotate csys because of alpha
    vrota.x() = 0.0;
    vrota.y() = 0.0;
    vrota.z() = alpha;
    if (react_force.x() < 0)
        vrota.z() = alpha;
    else
        vrota.z() = -alpha;
    mrotma.SetFromCardanAnglesXYZ(vrota);
    ma1 = marot_beta * mrotma;

    ChMatrix33<> ma2 = ma1;

    // is a bevel gear?
    double be = std::acos(Vdot(GetDirShaft1(), GetDirShaft2()));
    bool is_bevel = true;
    if (fabs(Vdot(GetDirShaft1(), GetDirShaft2())) > 0.96)
        is_bevel = false;

    // compute wheel radii so that:
    //            w2 = - tau * w1
    if (!is_bevel) {
        double pardist = Vdot(mr, vbdist);
        double inv_tau = 1.0 / tau;
        if (!epicyclic) {
            r2 = pardist - pardist / (inv_tau + 1.0);
        } else {
            r2 = pardist - (tau * pardist) / (tau - 1.0);
        }
        r1 = r2 * tau;
    } else {
        double gamma2;
        if (!epicyclic) {
            gamma2 = be / (tau + 1.0);
        } else {
            gamma2 = be / (-tau + 1.0);
        }
        double al = CH_PI - std::acos(Vdot(GetDirShaft2(), my));
        double te = CH_PI - al - be;
        double fd = sin(te) * (bdist / sin(be));
        r2 = fd * tan(gamma2);
        r1 = r2 * tau;
    }

    // compute markers positions, supposing they
    // stay on the ideal wheel contact point
    mmark1 = Vadd(GetPosShaft2(), Vmul(mr2, r2));
    mmark2 = mmark1;
    contact_pt = mmark1;

    // correct marker 1 position if phasing is not correct
    if (checkphase) {
        double realtau = tau;
        if (epicyclic)
            realtau = -tau;
        double m_delta;
        m_delta = -(a2 / realtau) - a1 - phase;

        if (m_delta > CH_PI)
            m_delta -= (CH_2PI);  // range -180..+180 is better than 0...360
        if (m_delta > (CH_PI / 4.0))
            m_delta = (CH_PI / 4.0);  // phase correction only in +/- 45�
        if (m_delta < -(CH_PI / 4.0))
            m_delta = -(CH_PI / 4.0);

        vrota.x() = vrota.y() = 0.0;
        vrota.z() = -m_delta;
        mrotma.SetFromCardanAnglesXYZ(vrota);  // rotate about Z of shaft to correct
        mmark1 = abs_shaft1.GetRotMat().transpose() * (mmark1 - GetPosShaft1());
        mmark1 = mrotma * mmark1;
        mmark1 = abs_shaft1.GetRotMat() * mmark1 + GetPosShaft1();
    }
    // Move Shaft 1 along its direction if not aligned to wheel
    double offset = Vdot(GetDirShaft1(), (contact_pt - GetPosShaft1()));
    ChVector3d moff = GetDirShaft1() * offset;
    if (fabs(offset) > 0.0001)
        local_shaft1.SetPos(local_shaft1.GetPos() + m_body1->TransformDirectionParentToLocal(moff));

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::MotionType::EXTERNAL);
    marker1->SetMotionType(ChMarker::MotionType::EXTERNAL);

    // move marker1 in proper positions
    marker1->ImposeAbsoluteTransform(ChFrame<>(mmark1, ma1.GetQuaternion()));
    // move marker2 in proper positions
    marker2->ImposeAbsoluteTransform(ChFrame<>(mmark2, ma2.GetQuaternion()));
}

void ChLinkLockGear::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockGear>();

    // serialize parent class
    ChLinkLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(tau);
    archive_out << CHNVP(alpha);
    archive_out << CHNVP(beta);
    archive_out << CHNVP(phase);
    archive_out << CHNVP(checkphase);
    archive_out << CHNVP(epicyclic);
    archive_out << CHNVP(a1);
    archive_out << CHNVP(a2);
    archive_out << CHNVP(r1);
    archive_out << CHNVP(r2);
    archive_out << CHNVP(local_shaft1);
    archive_out << CHNVP(local_shaft2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLockGear::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockGear>();

    // deserialize parent class
    ChLinkLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(tau);
    archive_in >> CHNVP(alpha);
    archive_in >> CHNVP(beta);
    archive_in >> CHNVP(phase);
    archive_in >> CHNVP(checkphase);
    archive_in >> CHNVP(epicyclic);
    archive_in >> CHNVP(a1);
    archive_in >> CHNVP(a2);
    archive_in >> CHNVP(r1);
    archive_in >> CHNVP(r2);
    archive_in >> CHNVP(local_shaft1);
    archive_in >> CHNVP(local_shaft2);

    mask.SetTwoBodiesVariables(&m_body1->Variables(), &m_body2->Variables());
    BuildLink();
}

}  // end namespace chrono
