//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLinkPointSpline.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLinkPointSpline.h"
#include "physics/ChSystem.h"
#include "geometry/ChCLineSegment.h"

namespace chrono {

using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkPointSpline> a_registration_ChLinkPointSpline;

// BUILDERS
ChLinkPointSpline::ChLinkPointSpline() {
    type = LNK_POINTSPLINE;  // initializes type

    // default trajectory is a segment
    trajectory_line = ChSharedPtr<ChLine>(new ChLineSegment());

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(false, true, true, false, false, false, false);

    ChangedLinkMask();
}

// DESTROYER
ChLinkPointSpline::~ChLinkPointSpline() {
}

void ChLinkPointSpline::Copy(ChLinkPointSpline* source) {
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy own data

    trajectory_line = ChSharedPtr<ChLine>((ChLine*)source->trajectory_line->Duplicate());  // deep copy
}

ChLink* ChLinkPointSpline::new_Duplicate() {
    ChLinkPointSpline* m_l;
    m_l = new ChLinkPointSpline;
    m_l->Copy(this);
    return (m_l);
}

//////////

void ChLinkPointSpline::Set_trajectory_line(ChSharedPtr<geometry::ChLine> mline) {
    trajectory_line = mline;
}

/////////    UPDATE TIME
/////////

void ChLinkPointSpline::UpdateTime(double time) {
    ChTime = time;

    double tol = 10e-9;
    if (GetSystem())
        tol = ((ChSystem*)GetSystem())->GetTol();

    if (trajectory_line) {
        Vector param, ptang, ptang2, vdir, vdir2, vnorm, vrad, vpoint;
        double mu, ds, dh, mrad;

        // find nearest point
        vpoint = marker1->GetAbsCoord().pos;
        vpoint = Body2->TransformPointParentToLocal(vpoint);
        trajectory_line->FindNearestLinePoint(vpoint, mu, 0, ((ChSystem*)GetSystem())->GetTol());

        param.y = 0;
        param.z = 0;
        param.x = mu;
        trajectory_line->Evaluate(ptang, param.x);

        if (param.x < 0)
            param.x = 0;
        trajectory_line->Derive(vdir, param.x);

        param.x = mu + BDF_STEP_HIGH;
        if (param.x > 1)
            param.x = 1;
        trajectory_line->Evaluate(ptang2, param.x);

        trajectory_line->Derive(vdir2, param.x);

        ChMatrix33<> ma;

        vdir = Vnorm(vdir);
        vdir2 = Vnorm(vdir2);
        vnorm = Vcross(vdir2, vdir);
        if (vnorm.Length() < 1e-7) {
            // on a straight segment, no curvature, so define normal and radial by these:
            ma.Set_A_Xdir(vdir, -VECT_Z);
        } else {
            vnorm.Normalize();
            vrad = Vnorm(Vcross(vdir, vnorm));
            ma.Set_A_axis(vdir, vnorm, vrad);
        }
        Quaternion qabsdir = ma.Get_A_quaternion();

        ptang = Body2->TransformPointLocalToParent(ptang);
        qabsdir = Body2->GetRot() * qabsdir;

        Coordsys newmarkpos;
        newmarkpos.pos = ptang;
        newmarkpos.rot = qabsdir;
        marker2->Impose_Abs_Coord(newmarkpos);  // move "main" marker2 into tangent position
        marker2->SetMotionType(
            ChMarker::M_MOTION_EXTERNAL);  // the BDF routine won't handle speed and acc.calculus of the moved marker!

        ds = Vlength(Vsub(ptang, ptang2));
        dh = Vdot(Vsub(ptang2, ptang), vrad);
        mrad = ((ds * ds) / (2 * dh));  // radius of curvature on spline

        ChMatrix33<> mw;
        mw.Set_A_quaternion(marker2->GetAbsCoord().rot);

        deltaC.pos = VNULL;
        deltaC_dt.pos = VNULL;
        deltaC_dtdt.pos.x = 0;  // csys X axis aligned to vdir: just
        deltaC_dtdt.pos.y = 0;  // impose centripetal acceleration
        // deltaC_dtdt.pos.z =   pow(Vdot(this->GetRelM_dt().pos, vdir), 2) / mrad;
        deltaC_dtdt.pos.z = pow(GetRelM_dt().pos.x, 2) / mrad;

        deltaC.rot = QUNIT;
        deltaC_dt.rot = QNULL;
        deltaC_dtdt.rot = QNULL;
    }
}

void ChLinkPointSpline::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);
    // serialize parent class too
    ChLinkLock::StreamOUT(mstream);

    // stream out all member data
    mstream.AbstractWrite(this->trajectory_line.get_ptr());  //***TODO*** proper serialize for ChSharedPtr
}

void ChLinkPointSpline::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChLinkLock::StreamIN(mstream);

    // stream in all member data
    ChLine* mline = 0;
    mstream.AbstractReadCreate(&mline);  //***TODO*** proper deserialize for ChSharedPtr
    this->trajectory_line = ChSharedPtr<ChLine>(mline);
}

///////////////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____
