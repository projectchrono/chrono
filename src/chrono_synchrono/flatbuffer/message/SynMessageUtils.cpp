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

AgentKey::AgentKey(int node_id, int agent_id) {
    m_node_id = node_id;
    m_agent_id = agent_id;

    // Matthew Szudzik pairing function (Cantor pair would also work)
    m_unique_id = node_id >= agent_id ? (node_id * node_id) + node_id + agent_id : (agent_id * agent_id) + node_id;
}

std::string AgentKey::GetKeyString() const {
    return std::to_string(m_node_id) + "." + std::to_string(m_agent_id);
}

const SynFlatBuffers::AgentKey* const AgentKey::GetFlatbuffersKey() const {
    // This memory is freed when we clear out messages
    return new SynFlatBuffers::AgentKey(m_node_id, m_agent_id);
}

SynPose::SynPose(const ChVector3d& mv, const ChQuaternion<>& mq) {
    m_frame = ChFrameMoving<>(mv, mq);
}

SynPose::SynPose(const ChFrameMoving<>& frame) {
    m_frame = frame;
}

SynPose::SynPose(const SynFlatBuffers::Pose* pose) {
    m_frame = ChFrameMoving<>({pose->pos()->x(), pose->pos()->y(), pose->pos()->z()},
                              {pose->rot()->e0(), pose->rot()->e1(), pose->rot()->e2(), pose->rot()->e3()});

    m_frame.SetPosDt({pose->pos_dt()->x(), pose->pos_dt()->y(), pose->pos_dt()->z()});
    m_frame.SetRotDt({pose->rot_dt()->e0(), pose->rot_dt()->e1(), pose->rot_dt()->e2(), pose->rot_dt()->e3()});
    m_frame.SetPosDt2({pose->pos_dtdt()->x(), pose->pos_dtdt()->y(), pose->pos_dtdt()->z()});
    m_frame.SetRotDt2({pose->rot_dtdt()->e0(), pose->rot_dtdt()->e1(), pose->rot_dtdt()->e2(), pose->rot_dtdt()->e3()});
}

flatbuffers::Offset<SynFlatBuffers::Pose> SynPose::ToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto fb_pos =
        SynFlatBuffers::CreateVector(builder, m_frame.GetPos().x(), m_frame.GetPos().y(), m_frame.GetPos().z());
    auto fb_rot = SynFlatBuffers::CreateQuaternion(builder,
                                                   m_frame.GetRot().e0(),   //
                                                   m_frame.GetRot().e1(),   //
                                                   m_frame.GetRot().e2(),   //
                                                   m_frame.GetRot().e3());  //

    auto fb_pos_dt =
        SynFlatBuffers::CreateVector(builder, m_frame.GetPosDt().x(), m_frame.GetPosDt().y(), m_frame.GetPosDt().z());
    auto fb_rot_dt = SynFlatBuffers::CreateQuaternion(builder,
                                                      m_frame.GetRotDt().e0(),   //
                                                      m_frame.GetRotDt().e1(),   //
                                                      m_frame.GetRotDt().e2(),   //
                                                      m_frame.GetRotDt().e3());  //

    auto fb_pos_dtdt = SynFlatBuffers::CreateVector(builder,                   //
                                                    m_frame.GetPosDt2().x(),   //
                                                    m_frame.GetPosDt2().y(),   //
                                                    m_frame.GetPosDt2().z());  //
    auto fb_rot_dtdt = SynFlatBuffers::CreateQuaternion(builder,
                                                        m_frame.GetRotDt2().e0(),   //
                                                        m_frame.GetRotDt2().e1(),   //
                                                        m_frame.GetRotDt2().e2(),   //
                                                        m_frame.GetRotDt2().e3());  //
    auto fb_pose = SynFlatBuffers::CreatePose(builder, fb_pos, fb_rot, fb_pos_dt, fb_rot_dt, fb_pos_dtdt, fb_rot_dtdt);

    return fb_pose;
}

}  // namespace synchrono
}  // namespace chrono
