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

#include "chrono/physics/ChLinkGear.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkGear)

ChLinkGear::ChLinkGear()
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

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(true, false, false, false, false, false, false);
    ChangedLinkMask();
}

ChLinkGear::ChLinkGear(const ChLinkGear& other) : ChLinkLock(other) {
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

ChVector<> ChLinkGear::Get_shaft_dir1() const {
    if (Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body1)->TransformLocalToParent(local_shaft1, absframe);
        return absframe.GetA().Get_A_Zaxis();
    } else
        return VECT_Z;
}

ChVector<> ChLinkGear::Get_shaft_dir2() const {
    if (Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body2)->TransformLocalToParent(local_shaft2, absframe);
        return absframe.GetA().Get_A_Zaxis();
    } else
        return VECT_Z;
}

ChVector<> ChLinkGear::Get_shaft_pos1() const {
    if (Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body1)->TransformLocalToParent(local_shaft1, absframe);
        return absframe.GetPos();
    } else
        return VNULL;
}

ChVector<> ChLinkGear::Get_shaft_pos2() const {
    if (Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body2)->TransformLocalToParent(local_shaft2, absframe);
        return absframe.GetPos();
    } else
        return VNULL;
}

void ChLinkGear::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

    // Move markers 1 and 2 to align them as gear teeth

    ChMatrix33<> ma1;
    ChMatrix33<> ma2;
    ChMatrix33<> mrotma;
    ChMatrix33<> marot_beta;
    ChVector<> mx;
    ChVector<> my;
    ChVector<> mz;
    ChVector<> mr;
    ChVector<> mmark1;
    ChVector<> mmark2;
    ChVector<> lastX;
    ChVector<> vrota;
    Coordsys newmarkpos;

    ChFrame<double> abs_shaft1;
    ChFrame<double> abs_shaft2;

    ((ChFrame<double>*)Body1)->TransformLocalToParent(local_shaft1, abs_shaft1);
    ((ChFrame<double>*)Body2)->TransformLocalToParent(local_shaft2, abs_shaft2);

    ChVector<> vbdist = Vsub(Get_shaft_pos1(), Get_shaft_pos2());
    ChVector<> Trad1 = Vnorm(Vcross(Get_shaft_dir1(), Vnorm(Vcross(Get_shaft_dir1(), vbdist))));
    ChVector<> Trad2 = Vnorm(Vcross(Vnorm(Vcross(Get_shaft_dir2(), vbdist)), Get_shaft_dir2()));

    double dist = Vlength(vbdist);

    // compute actual rotation of the two wheels (relative to truss).
    ChVector<> md1 = abs_shaft1.GetA().MatrT_x_Vect(-vbdist);
    md1.z() = 0;
    md1 = Vnorm(md1);
    ChVector<> md2 = abs_shaft2.GetA().MatrT_x_Vect(-vbdist);
    md2.z() = 0;
    md2 = Vnorm(md2);

    double periodic_a1 = ChAtan2(md1.x(), md1.y());
    double periodic_a2 = ChAtan2(md2.x(), md2.y());
    double old_a1 = a1;
    double old_a2 = a2;
    double turns_a1 = floor(old_a1 / CH_C_2PI);
    double turns_a2 = floor(old_a2 / CH_C_2PI);
    double a1U = turns_a1 * CH_C_2PI + periodic_a1 + CH_C_2PI;
    double a1M = turns_a1 * CH_C_2PI + periodic_a1;
    double a1L = turns_a1 * CH_C_2PI + periodic_a1 - CH_C_2PI;
    a1 = a1M;
    if (fabs(a1U - old_a1) < fabs(a1M - old_a1))
        a1 = a1U;
    if (fabs(a1L - a1) < fabs(a1M - a1))
        a1 = a1L;
    double a2U = turns_a2 * CH_C_2PI + periodic_a2 + CH_C_2PI;
    double a2M = turns_a2 * CH_C_2PI + periodic_a2;
    double a2L = turns_a2 * CH_C_2PI + periodic_a2 - CH_C_2PI;
    a2 = a2M;
    if (fabs(a2U - old_a2) < fabs(a2M - old_a2))
        a2 = a2U;
    if (fabs(a2L - a2) < fabs(a2M - a2))
        a2 = a2L;

    // compute new markers coordsystem alignment
    my = Vnorm(vbdist);
    mz = Get_shaft_dir1();
    mx = Vnorm(Vcross(my, mz));
    mr = Vnorm(Vcross(mz, mx));
    mz = Vnorm(Vcross(mx, my));
    ChVector<> mz2, mx2, mr2, my2;
    my2 = my;
    mz2 = Get_shaft_dir2();
    mx2 = Vnorm(Vcross(my2, mz2));
    mr2 = Vnorm(Vcross(mz2, mx2));

    ma1.Set_A_axis(mx, my, mz);

    // rotate csys because of beta
    vrota.x() = 0.0;
    vrota.y() = beta;
    vrota.z() = 0.0;
    mrotma.Set_A_Rxyz(vrota);
    marot_beta.MatrMultiply(ma1, mrotma);
    // rotate csys because of alpha
    vrota.x() = 0.0;
    vrota.y() = 0.0;
    vrota.z() = alpha;
    if (react_force.x() < 0)
        vrota.z() = alpha;
    else
        vrota.z() = -alpha;
    mrotma.Set_A_Rxyz(vrota);
    ma1.MatrMultiply(marot_beta, mrotma);

    ma2.CopyFromMatrix(ma1);

    // is a bevel gear?
    double be = acos(Vdot(Get_shaft_dir1(), Get_shaft_dir2()));
    bool is_bevel = true;
    if (fabs(Vdot(Get_shaft_dir1(), Get_shaft_dir2())) > 0.96)
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
        double al = CH_C_PI - acos(Vdot(Get_shaft_dir2(), my));
        double te = CH_C_PI - al - be;
        double fd = sin(te) * (dist / sin(be));
        r2 = fd * tan(gamma2);
        r1 = r2 * tau;
    }

    // compute markers positions, supposing they
    // stay on the ideal wheel contact point
    mmark1 = Vadd(Get_shaft_pos2(), Vmul(mr2, r2));
    mmark2 = mmark1;
    contact_pt = mmark1;

    // correct marker 1 position if phasing is not correct
    if (checkphase) {
        double realtau = tau;
        if (epicyclic)
            realtau = -tau;
        double m_delta;
        m_delta = -(a2 / realtau) - a1 - phase;

        if (m_delta > CH_C_PI)
            m_delta -= (CH_C_2PI);  // range -180..+180 is better than 0...360
        if (m_delta > (CH_C_PI / 4.0))
            m_delta = (CH_C_PI / 4.0);  // phase correction only in +/- 45°
        if (m_delta < -(CH_C_PI / 4.0))
            m_delta = -(CH_C_PI / 4.0);

        vrota.x() = vrota.y() = 0.0;
        vrota.z() = -m_delta;
        mrotma.Set_A_Rxyz(vrota);  // rotate about Z of shaft to correct
        mmark1 = abs_shaft1.GetA().MatrT_x_Vect(Vsub(mmark1, Get_shaft_pos1()));
        mmark1 = mrotma.Matr_x_Vect(mmark1);
        mmark1 = Vadd(abs_shaft1.GetA().Matr_x_Vect(mmark1), Get_shaft_pos1());
    }
    // Move Shaft 1 along its direction if not aligned to wheel
    double offset = Vdot(Get_shaft_dir1(), (contact_pt - Get_shaft_pos1()));
    ChVector<> moff = Get_shaft_dir1() * offset;
    if (fabs(offset) > 0.0001)
        local_shaft1.SetPos(local_shaft1.GetPos() + Body1->TransformDirectionParentToLocal(moff));

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);
    marker1->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

    // move marker1 in proper positions
    newmarkpos.pos = mmark1;
    newmarkpos.rot = ma1.Get_A_quaternion();
    marker1->Impose_Abs_Coord(newmarkpos);  // move marker1 into teeth position
    // move marker2 in proper positions
    newmarkpos.pos = mmark2;
    newmarkpos.rot = ma2.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);  // move marker2 into teeth position
}

void ChLinkGear::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkGear>();

    // serialize parent class
    ChLinkLock::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(tau);
    marchive << CHNVP(alpha);
    marchive << CHNVP(beta);
    marchive << CHNVP(phase);
    marchive << CHNVP(checkphase);
    marchive << CHNVP(epicyclic);
    marchive << CHNVP(a1);
    marchive << CHNVP(a2);
    marchive << CHNVP(r1);
    marchive << CHNVP(r2);
    marchive << CHNVP(local_shaft1);
    marchive << CHNVP(local_shaft2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkGear::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkGear>();

    // deserialize parent class
    ChLinkLock::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(tau);
    marchive >> CHNVP(alpha);
    marchive >> CHNVP(beta);
    marchive >> CHNVP(phase);
    marchive >> CHNVP(checkphase);
    marchive >> CHNVP(epicyclic);
    marchive >> CHNVP(a1);
    marchive >> CHNVP(a2);
    marchive >> CHNVP(r1);
    marchive >> CHNVP(r2);
    marchive >> CHNVP(local_shaft1);
    marchive >> CHNVP(local_shaft2);
}

}  // end namespace chrono
