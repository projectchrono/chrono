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

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMarker.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMarker)
CH_UPCASTING(ChMarker, ChObj)
CH_UPCASTING_SANITIZED(ChMarker, ChFrameMoving<double>, ChMarker_ChFrameMoving_double)

ChMarker::ChMarker()
    : Body(NULL),
      rest_coord(CSYSNORM),
      motion_type(M_MOTION_FUNCTIONS),
      motion_axis(VECT_Z),
      last_rel_coord(CSYSNORM),
      last_rel_coord_dt(CSYSNULL),
      last_time(0) {
    motion_X = chrono_types::make_shared<ChFunctionConst>(0);  // default: no motion
    motion_Y = chrono_types::make_shared<ChFunctionConst>(0);
    motion_Z = chrono_types::make_shared<ChFunctionConst>(0);
    motion_ang = chrono_types::make_shared<ChFunctionConst>(0);

    UpdateState();
}

ChMarker::ChMarker(const std::string& name,
                   ChBody* body,
                   const ChCoordsys<>& rel_pos,
                   const ChCoordsys<>& rel_pos_dt,
                   const ChCoordsys<>& rel_pos_dtdt) {
    SetNameString(name);
    Body = body;

    motion_X = chrono_types::make_shared<ChFunctionConst>(0);  // default: no motion
    motion_Y = chrono_types::make_shared<ChFunctionConst>(0);
    motion_Z = chrono_types::make_shared<ChFunctionConst>(0);
    motion_ang = chrono_types::make_shared<ChFunctionConst>(0);
    motion_axis = VECT_Z;

    rest_coord = CSYSNORM;

    motion_type = M_MOTION_FUNCTIONS;

    SetCsys(rel_pos);
    SetCsysDer(rel_pos_dt);
    SetCsysDer2(rel_pos_dtdt);

    last_rel_coord = CSYSNORM;
    last_rel_coord_dt = CSYSNULL;
    last_time = 0;

    UpdateState();
}

ChMarker::ChMarker(const ChMarker& other) : ChObj(other), ChFrameMoving<double>(other) {
    Body = NULL;

    motion_X = std::shared_ptr<ChFunction>(other.motion_X->Clone());
    motion_Y = std::shared_ptr<ChFunction>(other.motion_Y->Clone());
    motion_Z = std::shared_ptr<ChFunction>(other.motion_Z->Clone());
    motion_ang = std::shared_ptr<ChFunction>(other.motion_ang->Clone());

    motion_axis = other.motion_axis;

    rest_coord = other.rest_coord;

    motion_type = other.motion_type;

    abs_frame = other.abs_frame;

    last_rel_coord = other.last_rel_coord;
    last_rel_coord_dt = other.last_rel_coord_dt;
    last_time = other.last_time;
}

ChMarker::~ChMarker() {}

// Setup the functions when user changes them.

void ChMarker::SetMotion_X(std::shared_ptr<ChFunction> m_funct) {
    motion_X = m_funct;
}

void ChMarker::SetMotion_Y(std::shared_ptr<ChFunction> m_funct) {
    motion_Y = m_funct;
}

void ChMarker::SetMotion_Z(std::shared_ptr<ChFunction> m_funct) {
    motion_Z = m_funct;
}

void ChMarker::SetMotion_ang(std::shared_ptr<ChFunction> m_funct) {
    motion_ang = m_funct;
}

void ChMarker::SetMotion_axis(ChVector3d m_axis) {
    motion_axis = m_axis;
}

// Coordinate setting, for user access

void ChMarker::Impose_Rel_Coord(const ChCoordsysd& m_coord) {
    ChQuaterniond qtemp;
    // set the actual coordinates
    SetCsys(m_coord);
    // set the resting position coordinates
    rest_coord.pos.x() = m_coord.pos.x() - motion_X->GetVal(ChTime);
    rest_coord.pos.y() = m_coord.pos.y() - motion_Y->GetVal(ChTime);
    rest_coord.pos.z() = m_coord.pos.z() - motion_Z->GetVal(ChTime);
    qtemp = QuatFromAngleAxis(-(motion_ang->GetVal(ChTime)), motion_axis);
    rest_coord.rot = Qcross(m_coord.rot, qtemp);  // ***%%% check
                                                  // set also the absolute positions, and other.
    UpdateState();
}

void ChMarker::Impose_Abs_Coord(const ChCoordsysd& m_coord) {
    ChBody* my_body;
    my_body = GetBody();

    ChCoordsysd csys;
    // coordsys: transform the representation from the parent reference frame
    // to the local reference frame.
    csys.pos = my_body->TransformPointParentToLocal(m_coord.pos);
    csys.rot = Qcross(Qconjugate(my_body->GetCsys().rot), m_coord.rot);

    // apply the imposition on local  coordinate and resting coordinate:
    Impose_Rel_Coord(csys);
}

// Utilities for coordinate transformations

ChVector3d ChMarker::Point_World2Ref(const ChVector3d& point) const {
    return abs_frame / point;
}

ChVector3d ChMarker::Point_Ref2World(const ChVector3d& point) const {
    return *(ChFrame<double>*)&abs_frame * point;
}

ChVector3d ChMarker::Dir_World2Ref(const ChVector3d& dir) const {
    return abs_frame.GetRotMat().transpose() * dir;
}

ChVector3d ChMarker::Dir_Ref2World(const ChVector3d& dir) const {
    return abs_frame.GetRotMat().transpose() * dir;
}

// This handles the time-varying functions for the relative coordinates
void ChMarker::UpdateTime(double mytime) {
    ChCoordsysd csys, csys_dt, csys_dtdt;
    ChQuaterniond qtemp;
    double ang, ang_dt, ang_dtdt;

    ChTime = mytime;

    // if a imposed motion (keyframed movement) affects the marker position (example,from R3D animation system),
    // compute the speed and acceleration values by BDF (example,see the UpdatedExternalTime() function, later)
    // so the updating via motion laws can be skipped!
    if (motion_type == M_MOTION_KEYFRAMED)
        return;

    // skip relative-position-functions evaluation also if
    // someone is already handling this from outside..
    if (motion_type == M_MOTION_EXTERNAL)
        return;

    // positions:
    // update positions:    rel_pos
    csys.pos.x() = motion_X->GetVal(mytime);
    csys.pos.y() = motion_Y->GetVal(mytime);
    csys.pos.z() = motion_Z->GetVal(mytime);
    csys.pos += rest_coord.pos;

    // update speeds:		rel_pos_dt
    csys_dt.pos.x() = motion_X->GetDer(mytime);
    csys_dt.pos.y() = motion_Y->GetDer(mytime);
    csys_dt.pos.z() = motion_Z->GetDer(mytime);

    // update accelerations
    csys_dtdt.pos.x() = motion_X->GetDer2(mytime);
    csys_dtdt.pos.y() = motion_Y->GetDer2(mytime);
    csys_dtdt.pos.z() = motion_Z->GetDer2(mytime);

    // rotations:

    ang = motion_ang->GetVal(mytime);
    ang_dt = motion_ang->GetDer(mytime);
    ang_dtdt = motion_ang->GetDer2(mytime);

    if ((ang != 0) || (ang_dt != 0) || (ang_dtdt != 0)) {
        // update q
        ChVector3d motion_axis_versor = Vnorm(motion_axis);
        qtemp = QuatFromAngleAxis(ang, motion_axis_versor);
        csys.rot = Qcross(qtemp, rest_coord.rot);
        // update q_dt
        csys_dt.rot = QuatDerFromAngleAxis(csys.rot, ang_dt, motion_axis_versor);
        // update q_dtdt
        csys_dtdt.rot = QuatDer2FromAngleAxis(ang_dtdt, motion_axis_versor, csys.rot, csys_dt.rot);
    } else {
        csys.rot = GetRot();
        csys_dt.rot = QNULL;
        csys_dtdt.rot = QNULL;
    }

    // Set the position, speed and acceleration in relative space,
    // automatically getting also the absolute values,
    if (!(csys == this->Csys))
        SetCsys(csys);

    if (!(csys_dt == this->Csys_dt) || !(csys_dt.rot == QNULL))
        SetCsysDer(csys_dt);

    if (!(csys_dtdt == this->Csys_dtdt) || !(csys_dtdt.rot == QNULL))
        SetCsysDer2(csys_dtdt);
}

void ChMarker::UpdateState() {
    if (!GetBody())
        return;

    GetBody()->TransformLocalToParent(*this, abs_frame);
}

void ChMarker::Update(double mytime) {
    UpdateTime(mytime);
    UpdateState();
}

void ChMarker::UpdatedExternalTime(double prevtime, double mtime) {
    double mstep = mtime - prevtime;

    ChCoordsysd m_rel_pos_dt;
    ChCoordsysd m_rel_pos_dtdt;

    // do not try to switch on the M_MOTION_KEYFRAMED mode if
    // we are already in the M_MOTION_EXTERNAL mode, maybe because
    // a link point-surface is already moving the marker and
    // it will handle the accelerations by itself
    if (this->motion_type == M_MOTION_EXTERNAL)
        return;

    // otherwise see if a BDF is needed, cause an external 3rd party is moving the marker
    this->motion_type = M_MOTION_FUNCTIONS;

    // if POSITION or ROTATION ("rel_pos") has been changed in acceptable time step...
    if ((!(Vequal(Csys.pos, last_rel_coord.pos)) || !(Qequal(Csys.rot, last_rel_coord.rot))) && (fabs(mstep) < 0.1) &&
        (mstep != 0)) {
        // ... and if motion wasn't caused by motion laws, then it was a keyframed movement!
        if ((motion_X->GetVal(mtime) == 0) && (motion_Y->GetVal(mtime) == 0) && (motion_Z->GetVal(mtime) == 0) &&
            (motion_ang->GetVal(mtime) == 0) && (motion_X->GetType() == ChFunction::Type::CONSTANT) &&
            (motion_Y->GetType() == ChFunction::Type::CONSTANT) && (motion_Z->GetType() == ChFunction::Type::CONSTANT) &&
            (motion_ang->GetType() == ChFunction::Type::CONSTANT)) {
            // compute the relative speed by BDF !
            m_rel_pos_dt.pos = Vmul(Vsub(Csys.pos, last_rel_coord.pos), 1 / mstep);
            m_rel_pos_dt.rot = Qscale(Qsub(Csys.rot, last_rel_coord.rot), 1 / mstep);

            // compute the relative acceleration by BDF !
            m_rel_pos_dtdt.pos = Vmul(Vsub(m_rel_pos_dt.pos, last_rel_coord_dt.pos), 1 / mstep);
            m_rel_pos_dtdt.rot = Qscale(Qsub(m_rel_pos_dt.rot, last_rel_coord_dt.rot), 1 / mstep);

            // Set the position, speed and acceleration in relative space,
            // automatically getting also the absolute values,
            SetCsysDer(m_rel_pos_dt);
            SetCsysDer2(m_rel_pos_dtdt);

            // update the remaining state variables
            this->UpdateState();

            // remember that the movement of this guy won't need further update
            // of speed and acc. via motion laws!
            this->motion_type = M_MOTION_KEYFRAMED;
        }
    }

    // restore state buffers and that's all.
    last_time = ChTime;
    last_rel_coord = Csys;
    last_rel_coord_dt = Csys_dt;
}

//  FILE I/O

void ChMarker::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChMarker>();

    // serialize parent class
    ChObj::ArchiveOut(marchive);
    // serialize parent class
    ChFrameMoving<double>::ArchiveOut(marchive);

    // serialize all member data:
    eChMarkerMotion_mapper mmapper;
    marchive << CHNVP(mmapper(motion_type), "motion_type");
    marchive << CHNVP(motion_X);
    marchive << CHNVP(motion_Y);
    marchive << CHNVP(motion_Z);
    marchive << CHNVP(motion_ang);
    marchive << CHNVP(motion_axis);
    marchive << CHNVP(Body);
}

/// Method to allow de serialization of transient data from archives.
void ChMarker::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChMarker>();

    // deserialize parent class
    ChObj::ArchiveIn(marchive);
    // deserialize parent class
    ChFrameMoving<double>::ArchiveIn(marchive);

    // stream in all member data:
    eChMarkerMotion_mapper mmapper;
    marchive >> CHNVP(mmapper(motion_type), "motion_type");
    marchive >> CHNVP(motion_X);
    marchive >> CHNVP(motion_Y);
    marchive >> CHNVP(motion_Z);
    marchive >> CHNVP(motion_ang);
    marchive >> CHNVP(motion_axis);
    marchive >> CHNVP(Body);

    UpdateState();                          // updates the ChMarker::abs_frame first
    Impose_Abs_Coord(this->GetAbsCoord());  // from ChMarker::abs_frame update ChMarker::rest_coord and ChFrame::coord
}

}  // end namespace chrono
