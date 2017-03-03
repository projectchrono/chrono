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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/core/ChTransform.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChMarker.h"

namespace chrono {

#define MARKER_BDF_STEP 0.0001

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMarker)

ChMarker::ChMarker()
    : Body(NULL),
      rest_coord(CSYSNORM),
      motion_type(M_MOTION_FUNCTIONS),
      motion_axis(VECT_Z),
      last_rel_coord(CSYSNORM),
      last_rel_coord_dt(CSYSNULL),
      last_time(0) {
    motion_X = std::make_shared<ChFunction_Const>(0);  // default: no motion
    motion_Y = std::make_shared<ChFunction_Const>(0); 
    motion_Z = std::make_shared<ChFunction_Const>(0); 
    motion_ang = std::make_shared<ChFunction_Const>(0); 

    UpdateState();
}

ChMarker::ChMarker(char myname[], ChBody* myBody, Coordsys myrel_pos, Coordsys myrel_pos_dt, Coordsys myrel_pos_dtdt) {
    SetName(myname);
    Body = myBody;

    motion_X = std::make_shared<ChFunction_Const>(0);   // default: no motion
    motion_Y = std::make_shared<ChFunction_Const>(0); 
    motion_Z = std::make_shared<ChFunction_Const>(0); 
    motion_ang = std::make_shared<ChFunction_Const>(0); 
    motion_axis = VECT_Z;

    rest_coord = CSYSNORM;

    motion_type = M_MOTION_FUNCTIONS;

    SetCoord(myrel_pos);
    SetCoord_dt(myrel_pos_dt);
    SetCoord_dtdt(myrel_pos_dtdt);

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

ChMarker::~ChMarker() {

}

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

void ChMarker::SetMotion_axis(Vector m_axis) {
    motion_axis = m_axis;
}

// Coordinate setting, for user access

void ChMarker::Impose_Rel_Coord(const Coordsys& m_coord) {
    Quaternion qtemp;
    // set the actual coordinates
    SetCoord(m_coord);
    // set the resting position coordinates
    rest_coord.pos.x() = m_coord.pos.x() - motion_X->Get_y(ChTime);
    rest_coord.pos.y() = m_coord.pos.y() - motion_Y->Get_y(ChTime);
    rest_coord.pos.z() = m_coord.pos.z() - motion_Z->Get_y(ChTime);
    qtemp = Q_from_AngAxis(-(motion_ang->Get_y(ChTime)), motion_axis);
    rest_coord.rot = Qcross(m_coord.rot, qtemp);  // ***%%% check
                                                  // set also the absolute positions, and other.
    UpdateState();
}

void ChMarker::Impose_Abs_Coord(const Coordsys& m_coord) {
    ChBody* my_body;
    my_body = GetBody();

    Coordsys csys;
    // coordsys: transform the representation from the parent reference frame
    // to the local reference frame.
    csys.pos = ChTransform<>::TransformParentToLocal(m_coord.pos, my_body->GetCoord().pos, my_body->GetA());
    csys.rot = Qcross(Qconjugate(my_body->GetCoord().rot), m_coord.rot);
    // apply the imposition on local  coordinate and resting coordinate:
    Impose_Rel_Coord(csys);
}

//// Utilities for coordinate transformations

Vector ChMarker::Point_World2Ref(Vector* mpoint) {
    return abs_frame / *mpoint;
}

Vector ChMarker::Point_Ref2World(Vector* mpoint) {
    return *(ChFrame<double>*)&abs_frame * *mpoint;
}

Vector ChMarker::Dir_World2Ref(Vector* mpoint) {
    return abs_frame.GetA().MatrT_x_Vect(*mpoint);
}

Vector ChMarker::Dir_Ref2World(Vector* mpoint) {
    return abs_frame.GetA().Matr_x_Vect(*mpoint);
}

// This handles the time-varying functions for the relative
// coordinates

void ChMarker::UpdateTime(double mytime) {
    Coordsys csys, csys_dt, csys_dtdt;
    Quaternion qtemp;
    double ang, ang_dt, ang_dtdt;

    ChTime = mytime;

    // if a imposed motion (keyframed movement) affects the marker postion (example,from R3D animation system),
    // compute the speed and acceleration values by BDF (example,see the UpdatedExternalTime() function, later)
    // so the updating via motion laws can be skipped!
    if (motion_type == M_MOTION_KEYFRAMED)
        return;

    // skip realtive-position-functions evaluation also if
    // someone is already handling this from outside..
    if (motion_type == M_MOTION_EXTERNAL)
        return;

    // positions:
    // update positions:    rel_pos
    csys.pos.x() = motion_X->Get_y(mytime);
    csys.pos.y() = motion_Y->Get_y(mytime);
    csys.pos.z() = motion_Z->Get_y(mytime);
    if (motion_X->Get_Type() != ChFunction::FUNCT_MOCAP)
        csys.pos += rest_coord.pos;

    // update speeds:		rel_pos_dt
    csys_dt.pos.x() = motion_X->Get_y_dx(mytime);
    csys_dt.pos.y() = motion_Y->Get_y_dx(mytime);
    csys_dt.pos.z() = motion_Z->Get_y_dx(mytime);

    // update accelerations
    csys_dtdt.pos.x() = motion_X->Get_y_dxdx(mytime);
    csys_dtdt.pos.y() = motion_Y->Get_y_dxdx(mytime);
    csys_dtdt.pos.z() = motion_Z->Get_y_dxdx(mytime);

    // rotations:

    ang = motion_ang->Get_y(mytime);
    ang_dt = motion_ang->Get_y_dx(mytime);
    ang_dtdt = motion_ang->Get_y_dxdx(mytime);

    if ((ang != 0) || (ang_dt != 0) || (ang_dtdt != 0)) {
        // update q
        Vector motion_axis_versor = Vnorm(motion_axis);
        qtemp = Q_from_AngAxis(ang, motion_axis_versor);
        csys.rot = Qcross(qtemp, rest_coord.rot);
        // update q_dt
        csys_dt.rot = chrono::Qdt_from_AngAxis(csys.rot, ang_dt, motion_axis_versor);
        // update q_dtdt
        csys_dtdt.rot = chrono::Qdtdt_from_AngAxis(ang_dtdt, motion_axis_versor, csys.rot, csys_dt.rot);
    } else {
        csys.rot = coord.rot;  // rel_pos.rot;
        csys_dt.rot = QNULL;
        csys_dtdt.rot = QNULL;
    }

    // Set the position, speed and acceleration in relative space,
    // automatically getting also the absolute values,
    if (!(csys == this->coord))
        SetCoord(csys);

    if (!(csys_dt == this->coord_dt) || !(csys_dt.rot == QNULL))
        SetCoord_dt(csys_dt);

    if (!(csys_dtdt == this->coord_dtdt) || !(csys_dtdt.rot == QNULL))
        SetCoord_dtdt(csys_dtdt);
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

    Coordsys m_rel_pos_dt;
    Coordsys m_rel_pos_dtdt;

    // do not try to switch on the M_MOTION_KEYFRAMED mode if
    // we are already in the M_MOTION_EXTERNAL mode, maybe because
    // a link point-surface is already moving the marker and
    // it will handle the accelerations by itself
    if (this->motion_type == M_MOTION_EXTERNAL)
        return;

    // otherwise see if a BDF is needed, cause an external 3rd party is moving the marker
    this->motion_type = M_MOTION_FUNCTIONS;

    // if POSITION or ROTATION ("rel_pos") has been changed in acceptable time step...
    if ((!(Vequal(coord.pos, last_rel_coord.pos)) || !(Qequal(coord.rot, last_rel_coord.rot))) && (fabs(mstep) < 0.1) &&
        (mstep != 0)) {
        // ... and if motion wasn't caused by motion laws, then it was a keyframed movement!
        if ((motion_X->Get_y(mtime) == 0) && (motion_Y->Get_y(mtime) == 0) && (motion_Z->Get_y(mtime) == 0) &&
            (motion_ang->Get_y(mtime) == 0) && (motion_X->Get_Type() == ChFunction::FUNCT_CONST) &&
            (motion_Y->Get_Type() == ChFunction::FUNCT_CONST) && (motion_Z->Get_Type() == ChFunction::FUNCT_CONST) &&
            (motion_ang->Get_Type() == ChFunction::FUNCT_CONST)) {
            // compute the relative speed by BDF !
            m_rel_pos_dt.pos = Vmul(Vsub(coord.pos, last_rel_coord.pos), 1 / mstep);
            m_rel_pos_dt.rot = Qscale(Qsub(coord.rot, last_rel_coord.rot), 1 / mstep);

            // compute the relative acceleration by BDF !
            m_rel_pos_dtdt.pos = Vmul(Vsub(m_rel_pos_dt.pos, last_rel_coord_dt.pos), 1 / mstep);
            m_rel_pos_dtdt.rot = Qscale(Qsub(m_rel_pos_dt.rot, last_rel_coord_dt.rot), 1 / mstep);

            // Set the position, speed and acceleration in relative space,
            // automatically getting also the absolute values,
            SetCoord_dt(m_rel_pos_dt);
            SetCoord_dtdt(m_rel_pos_dtdt);

            // update the remaining state variables
            this->UpdateState();

            // remember that the movement of this guy won't need further update
            // of speed and acc. via motion laws!
            this->motion_type = M_MOTION_KEYFRAMED;
        }
    }

    // restore state buffers and that's all.
    last_time = ChTime;
    last_rel_coord = coord;
    last_rel_coord_dt = coord_dt;
}

//  FILE I/O

void ChMarker::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChMarker>();

    // serialize parent class
    ChObj::ArchiveOUT(marchive);
    // serialize parent class
    ChFrameMoving<double>::ArchiveOUT(marchive);

    // serialize all member data:
    eChMarkerMotion_mapper mmapper;
    marchive << CHNVP(mmapper(motion_type), "motion_type");
    marchive << CHNVP(motion_X);
    marchive << CHNVP(motion_Y);
    marchive << CHNVP(motion_Z);
    marchive << CHNVP(motion_ang);
    marchive << CHNVP(motion_axis);
}

/// Method to allow de serialization of transient data from archives.
void ChMarker::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChMarker>();

    // deserialize parent class
    ChObj::ArchiveIN(marchive);
    // deserialize parent class
    ChFrameMoving<double>::ArchiveIN(marchive);

    // stream in all member data:
    eChMarkerMotion_mapper mmapper;
    marchive >> CHNVP(mmapper(motion_type), "motion_type");
    marchive >> CHNVP(motion_X);
    marchive >> CHNVP(motion_Y);
    marchive >> CHNVP(motion_Z);
    marchive >> CHNVP(motion_ang);
    marchive >> CHNVP(motion_axis);
}

}  // end namespace chrono
