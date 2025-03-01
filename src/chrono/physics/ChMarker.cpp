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
    : m_body(NULL),
      m_rest_csys(CSYSNORM),
      m_motion_type(MotionType::FUNCTIONS),
      m_motion_axis(VECT_Z),
      m_last_rel_csys(CSYSNORM),
      m_last_rel_csys_dt(CSYSNULL),
      m_last_time(0) {
    m_motion_X = chrono_types::make_shared<ChFunctionConst>(0);  // default: no motion
    m_motion_Y = chrono_types::make_shared<ChFunctionConst>(0);
    m_motion_Z = chrono_types::make_shared<ChFunctionConst>(0);
    m_motion_ang = chrono_types::make_shared<ChFunctionConst>(0);

    UpdateState();
}

ChMarker::ChMarker(const std::string& name,
                   ChBody* body,
                   const ChCoordsys<>& rel_csys,
                   const ChCoordsys<>& rel_csys_dt,
                   const ChCoordsys<>& rel_csys_dtdt) {
    SetName(name);
    m_body = body;

    m_motion_X = chrono_types::make_shared<ChFunctionConst>(0);  // default: no motion
    m_motion_Y = chrono_types::make_shared<ChFunctionConst>(0);
    m_motion_Z = chrono_types::make_shared<ChFunctionConst>(0);
    m_motion_ang = chrono_types::make_shared<ChFunctionConst>(0);
    m_motion_axis = VECT_Z;

    m_rest_csys = CSYSNORM;

    m_motion_type = MotionType::FUNCTIONS;

    SetCoordsys(rel_csys);
    SetCoordsysDt(rel_csys_dt);
    SetCoordsysDt2(rel_csys_dtdt);

    m_last_rel_csys = CSYSNORM;
    m_last_rel_csys_dt = CSYSNULL;
    m_last_time = 0;

    UpdateState();
}

ChMarker::ChMarker(const ChMarker& other) : ChObj(other), ChFrameMoving<double>(other) {
    m_body = NULL;

    m_motion_X = std::shared_ptr<ChFunction>(other.m_motion_X->Clone());
    m_motion_Y = std::shared_ptr<ChFunction>(other.m_motion_Y->Clone());
    m_motion_Z = std::shared_ptr<ChFunction>(other.m_motion_Z->Clone());
    m_motion_ang = std::shared_ptr<ChFunction>(other.m_motion_ang->Clone());

    m_motion_axis = other.m_motion_axis;

    m_rest_csys = other.m_rest_csys;

    m_motion_type = other.m_motion_type;

    m_abs_frame = other.m_abs_frame;

    m_last_rel_csys = other.m_last_rel_csys;
    m_last_rel_csys_dt = other.m_last_rel_csys_dt;
    m_last_time = other.m_last_time;
}

ChMarker::~ChMarker() {}

// Setup the functions when user changes them.

void ChMarker::SetMotionX(std::shared_ptr<ChFunction> funct) {
    m_motion_X = funct;
}

void ChMarker::SetMotionY(std::shared_ptr<ChFunction> funct) {
    m_motion_Y = funct;
}

void ChMarker::SetMotionZ(std::shared_ptr<ChFunction> funct) {
    m_motion_Z = funct;
}

void ChMarker::SetMotionAngle(std::shared_ptr<ChFunction> funct) {
    m_motion_ang = funct;
}

void ChMarker::SetMotionAxis(ChVector3d axis) {
    m_motion_axis = axis;
}

// Coordinate setting, for user access

void ChMarker::ImposeRelativeTransform(const ChFrame<>& frame) {
    // set the actual coordinates
    SetCoordsys(frame.GetPos(), frame.GetRot());

    // set the resting position coordinates
    m_rest_csys.pos.x() = m_csys.pos.x() - m_motion_X->GetVal(ChTime);
    m_rest_csys.pos.y() = m_csys.pos.y() - m_motion_Y->GetVal(ChTime);
    m_rest_csys.pos.z() = m_csys.pos.z() - m_motion_Z->GetVal(ChTime);
    auto q = QuatFromAngleAxis(-(m_motion_ang->GetVal(ChTime)), m_motion_axis);
    m_rest_csys.rot = Qcross(m_csys.rot, q);  //// TODO: check
                                              //// set also the absolute positions and other
    UpdateState();
}

void ChMarker::ImposeAbsoluteTransform(const ChFrame<>& frame) {
    // transform representation from the parent reference frame to the local reference frame
    auto pos = GetBody()->TransformPointParentToLocal(frame.GetPos());
    auto rot = Qcross(Qconjugate(GetBody()->GetRot()), frame.GetRot());

    // impose relative transform and set resting coordinate
    ImposeRelativeTransform(ChFrame<>(pos, rot));
}

// This handles the time-varying functions for the relative coordinates
void ChMarker::UpdateTime(double time) {
    ChCoordsysd csys, csys_dt, csys_dtdt;
    ChQuaterniond qtemp;
    double ang, ang_dt, ang_dtdt;

    // If an imposed motion (keyframed movement) affects the marker position, compute the speed and acceleration
    // values by finite differenting so the updating via motion laws can be skipped
    if (m_motion_type == MotionType::KEYFRAMED)
        return;

    // Skip relative-position-functions evaluation also if already handled from outside
    if (m_motion_type == MotionType::EXTERNAL)
        return;

    // update positions
    csys.pos.x() = m_motion_X->GetVal(time);
    csys.pos.y() = m_motion_Y->GetVal(time);
    csys.pos.z() = m_motion_Z->GetVal(time);
    csys.pos += m_rest_csys.pos;

    // update speeds
    csys_dt.pos.x() = m_motion_X->GetDer(time);
    csys_dt.pos.y() = m_motion_Y->GetDer(time);
    csys_dt.pos.z() = m_motion_Z->GetDer(time);

    // update accelerations
    csys_dtdt.pos.x() = m_motion_X->GetDer2(time);
    csys_dtdt.pos.y() = m_motion_Y->GetDer2(time);
    csys_dtdt.pos.z() = m_motion_Z->GetDer2(time);

    // rotations
    ang = m_motion_ang->GetVal(time);
    ang_dt = m_motion_ang->GetDer(time);
    ang_dtdt = m_motion_ang->GetDer2(time);

    if ((ang != 0) || (ang_dt != 0) || (ang_dtdt != 0)) {
        // update q
        ChVector3d m_motion_axis_versor = Vnorm(m_motion_axis);
        qtemp = QuatFromAngleAxis(ang, m_motion_axis_versor);
        csys.rot = Qcross(qtemp, m_rest_csys.rot);
        // update q_dt
        csys_dt.rot = QuatDtFromAngleAxis(csys.rot, ang_dt, m_motion_axis_versor);
        // update q_dtdt
        csys_dtdt.rot = QuatDt2FromAngleAxis(ang_dtdt, m_motion_axis_versor, csys.rot, csys_dt.rot);
    } else {
        csys.rot = GetRot();
        csys_dt.rot = QNULL;
        csys_dtdt.rot = QNULL;
    }

    // Set the position, speed and acceleration in relative space, automatically getting also the absolute values
    if (!(csys == m_csys))
        SetCoordsys(csys);

    if (!(csys_dt == m_csys_dt) || !(csys_dt.rot == QNULL))
        SetCoordsysDt(csys_dt);

    if (!(csys_dtdt == m_csys_dtdt) || !(csys_dtdt.rot == QNULL))
        SetCoordsysDt2(csys_dtdt);
}

void ChMarker::UpdateState() {
    if (!GetBody())
        return;
    m_abs_frame = GetBody()->TransformLocalToParent(*this);
}

void ChMarker::Update(double time, bool update_assets) {
    ChObj::Update(time, update_assets);

    UpdateTime(time);
    UpdateState();
}

void ChMarker::UpdatedExternalTime(double prevtime, double mtime) {
    double step = mtime - prevtime;

    ChCoordsysd rel_pos_dt;
    ChCoordsysd rel_pos_dtdt;

    // do not try to switch on the KEYFRAMED mode if we are already in the EXTERNAL mode, maybe because a link
    // point-surface is already moving the marker and it will handle the accelerations by itself
    if (m_motion_type == MotionType::EXTERNAL)
        return;

    // otherwise see if finite differencing is needed, beccause an external 3rd party is moving the marker
    m_motion_type = MotionType::FUNCTIONS;

    // if POSITION or ROTATION ("rel_pos") has been changed in acceptable time step...
    if ((!(Vequal(m_csys.pos, m_last_rel_csys.pos)) || !(Qequal(m_csys.rot, m_last_rel_csys.rot))) &&
        (fabs(step) < 0.1) && (step != 0)) {
        // ... and if motion wasn't caused by motion laws, then it was a keyframed movement
        if ((m_motion_X->GetVal(mtime) == 0) && (m_motion_Y->GetVal(mtime) == 0) && (m_motion_Z->GetVal(mtime) == 0) &&
            (m_motion_ang->GetVal(mtime) == 0) && (m_motion_X->GetType() == ChFunction::Type::CONSTANT) &&
            (m_motion_Y->GetType() == ChFunction::Type::CONSTANT) &&
            (m_motion_Z->GetType() == ChFunction::Type::CONSTANT) &&
            (m_motion_ang->GetType() == ChFunction::Type::CONSTANT)) {
            // compute the relative speed by finite differences
            rel_pos_dt.pos = Vmul(Vsub(m_csys.pos, m_last_rel_csys.pos), 1 / step);
            rel_pos_dt.rot = Qscale(Qsub(m_csys.rot, m_last_rel_csys.rot), 1 / step);

            // compute the relative acceleration by finite differences
            rel_pos_dtdt.pos = Vmul(Vsub(rel_pos_dt.pos, m_last_rel_csys_dt.pos), 1 / step);
            rel_pos_dtdt.rot = Qscale(Qsub(rel_pos_dt.rot, m_last_rel_csys_dt.rot), 1 / step);

            // set the position, speed, and acceleration
            SetCoordsysDt(rel_pos_dt);
            SetCoordsysDt2(rel_pos_dtdt);

            // update the remaining state variables
            UpdateState();

            // that the movement of this marker does need further update of speed and acc. via motion laws
            m_motion_type = MotionType::KEYFRAMED;
        }
    }

    // restore state buffers
    m_last_time = ChTime;
    m_last_rel_csys = m_csys;
    m_last_rel_csys_dt = m_csys_dt;
}

// -----------------------------------------------------------------------------

class ChMarker_MotionType_enum_mapper : public ChMarker {
  public:
    CH_ENUM_MAPPER_BEGIN(MotionType);
    CH_ENUM_VAL(MotionType::FUNCTIONS);
    CH_ENUM_VAL(MotionType::KEYFRAMED);
    CH_ENUM_VAL(MotionType::EXTERNAL);
    CH_ENUM_MAPPER_END(MotionType);
};

void ChMarker::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChMarker>();

    // serialize parent class
    ChObj::ArchiveOut(archive_out);
    // serialize parent class
    ChFrameMoving<double>::ArchiveOut(archive_out);

    // serialize all member data:
    ChMarker_MotionType_enum_mapper::MotionType_mapper typemapper;
    archive_out << CHNVP(typemapper(m_motion_type), "m_motion_type");
    archive_out << CHNVP(m_motion_X);
    archive_out << CHNVP(m_motion_Y);
    archive_out << CHNVP(m_motion_Z);
    archive_out << CHNVP(m_motion_ang);
    archive_out << CHNVP(m_motion_axis);
    archive_out << CHNVP(m_body);
}

// Method to allow de serialization of transient data from archives.
void ChMarker::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChMarker>();

    // deserialize parent class
    ChObj::ArchiveIn(archive_in);
    // deserialize parent class
    ChFrameMoving<double>::ArchiveIn(archive_in);

    // stream in all member data:
    ChMarker_MotionType_enum_mapper::MotionType_mapper typemapper;
    archive_in >> CHNVP(typemapper(m_motion_type), "m_motion_type");
    archive_in >> CHNVP(m_motion_X);
    archive_in >> CHNVP(m_motion_Y);
    archive_in >> CHNVP(m_motion_Z);
    archive_in >> CHNVP(m_motion_ang);
    archive_in >> CHNVP(m_motion_axis);
    archive_in >> CHNVP(m_body);

    UpdateState();                         // update abs_frame first
    ImposeAbsoluteTransform(m_abs_frame);  // use abs_frame to update rest_coord and coord
}

}  // end namespace chrono
