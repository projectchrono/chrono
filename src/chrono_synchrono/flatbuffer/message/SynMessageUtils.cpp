// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Wrapper for several constructs that are common to many flatbuffer messages
// (Vectors, Quaternions, frames)
// See also flatbuffer/fbs/Utils.fbs
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynMessageUtils.h"

namespace chrono {
namespace synchrono {

SynPose::SynPose(const ChVector<>& mv, const ChQuaternion<>& mq) {
    m_frame = ChFrameMoving<>(mv, mq);
}

SynPose::SynPose(const ChFrameMoving<>& frame) {
    m_frame = frame;
}

SynPose::SynPose(const SynFlatBuffers::Pose* pose) {
    m_frame = ChFrameMoving<>({pose->pos()->x(), pose->pos()->y(), pose->pos()->z()},
                              {pose->rot()->e0(), pose->rot()->e1(), pose->rot()->e2(), pose->rot()->e3()});

    m_frame.coord_dt.pos = {pose->pos_dt()->x(), pose->pos_dt()->y(), pose->pos_dt()->z()};
    m_frame.coord_dt.rot = {pose->rot_dt()->e0(), pose->rot_dt()->e1(), pose->rot_dt()->e2(), pose->rot_dt()->e3()};
    m_frame.coord_dtdt.pos = {pose->pos_dtdt()->x(), pose->pos_dtdt()->y(), pose->pos_dtdt()->z()};
    m_frame.coord_dtdt.rot = {pose->rot_dtdt()->e0(), pose->rot_dtdt()->e1(), pose->rot_dtdt()->e2(),
                              pose->rot_dtdt()->e3()};
}

flatbuffers::Offset<SynFlatBuffers::Pose> SynPose::ToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto fb_pos =
        SynFlatBuffers::CreateVector(builder, m_frame.coord.pos.x(), m_frame.coord.pos.y(), m_frame.coord.pos.z());
    auto fb_rot = SynFlatBuffers::CreateQuaternion(builder,
                                                   m_frame.coord.rot.e0(),   //
                                                   m_frame.coord.rot.e1(),   //
                                                   m_frame.coord.rot.e2(),   //
                                                   m_frame.coord.rot.e3());  //

    auto fb_pos_dt = SynFlatBuffers::CreateVector(builder, m_frame.coord_dt.pos.x(), m_frame.coord_dt.pos.y(),
                                                  m_frame.coord_dt.pos.z());
    auto fb_rot_dt = SynFlatBuffers::CreateQuaternion(builder,
                                                      m_frame.coord_dt.rot.e0(),   //
                                                      m_frame.coord_dt.rot.e1(),   //
                                                      m_frame.coord_dt.rot.e2(),   //
                                                      m_frame.coord_dt.rot.e3());  //

    auto fb_pos_dtdt = SynFlatBuffers::CreateVector(builder,                      //
                                                    m_frame.coord_dtdt.pos.x(),   //
                                                    m_frame.coord_dtdt.pos.y(),   //
                                                    m_frame.coord_dtdt.pos.z());  //
    auto fb_rot_dtdt = SynFlatBuffers::CreateQuaternion(builder,
                                                        m_frame.coord_dtdt.rot.e0(),   //
                                                        m_frame.coord_dtdt.rot.e1(),   //
                                                        m_frame.coord_dtdt.rot.e2(),   //
                                                        m_frame.coord_dtdt.rot.e3());  //
    auto fb_pose = SynFlatBuffers::CreatePose(builder, fb_pos, fb_rot, fb_pos_dt, fb_rot_dt, fb_pos_dtdt, fb_rot_dtdt);

    return fb_pose;
}

}  // namespace synchrono
}  // namespace chrono
