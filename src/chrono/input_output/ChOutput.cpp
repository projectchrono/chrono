// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Implementation of the base class for a Chrono output database.
//
// =============================================================================

#include<numeric>

#include "chrono/input_output/ChOutput.h"

namespace chrono {

ChOutput::ChOutput(Mode mode) : m_mode(mode), m_buf_allocated(false) {}

void ChOutput::Write(int frame, double time, const ChAssembly::Components& components) {
    Write(frame, time, {&components});
}

void ChOutput::Write(int frame, double time, const std::vector<const ChAssembly::Components*>& components) {
    switch (m_mode) {
        case Mode::FRAMES:
            WriteTimeStamp(frame, time);

            for (auto c : components)
                WriteBodies(c->bodies);
            for (auto c : components)
                WriteShafts(c->shafts);
            for (auto c : components)
                WriteJoints(c->joints);
            for (auto c : components)
                WriteCouples(c->couples);
            for (auto c : components)
                WriteBodyBodyLoads(c->bushings);
            for (auto c : components)
                WriteLinSprings(c->tsdas);
            for (auto c : components)
                WriteRotSprings(c->rsdas);
            for (auto c : components)
                WriteLinMotors(c->lin_motors);
            for (auto c : components)
                WriteRotMotors(c->rot_motors);
            
            break;

        case Mode::SERIES:
            if (!m_buf_allocated) {
                size_t n_bodies = 0;
                size_t n_shafts = 0;
                size_t n_joints = 0;
                size_t n_tsdas = 0;
                size_t n_rsdas = 0;
                for (auto c : components) {
                    n_bodies += c->bodies.size();
                    n_shafts += c->shafts.size();
                    n_joints += c->joints.size();
                    n_tsdas += c->tsdas.size();
                    n_rsdas += c->rsdas.size();
                }
                m_body_buf.resize(n_bodies);
                m_shaft_buf.resize(n_shafts);
                m_joint_buf.resize(n_joints);
                m_tsda_buf.resize(n_tsdas);
                m_rsda_buf.resize(n_rsdas);
                m_buf_allocated = true;
            }

            m_time.push_back(time);

            size_t i_body = 0;
            size_t i_shaft = 0;
            size_t i_joint = 0;
            size_t i_tsda = 0;
            size_t i_rsda = 0;
            for (auto c : components) {
                for (const auto& body : c->bodies) {
                    auto& buf = m_body_buf[i_body];
                    const auto& ref_frame = body->GetFrameRefToAbs();
                    buf.pos.push_back(ref_frame.GetPos());
                    buf.rot.push_back(ref_frame.GetRot().GetCardanAnglesXYZ());
                    buf.lin_vel.push_back(ref_frame.GetPosDt());
                    buf.ang_vel.push_back(ref_frame.GetAngVelParent());
                    i_body++;
                }

                for (const auto& shaft : c->shafts) {
                    auto& buf = m_shaft_buf[i_shaft];
                    buf.pos.push_back(shaft->GetPos());
                    buf.vel.push_back(shaft->GetPosDt());
                    i_shaft++;
                }

                for (const auto& joint : c->joints) {
                    auto& buf = m_joint_buf[i_joint];
                    buf.react1.push_back({joint->GetReaction1().force, joint->GetReaction1().torque});
                    buf.react2.push_back({joint->GetReaction2().force, joint->GetReaction2().torque});
                    i_joint++;
                }

                for (const auto& tsda : c->tsdas) {
                    auto& buf = m_tsda_buf[i_tsda];
                    buf.point1.push_back(tsda->GetPoint1Abs());
                    buf.point2.push_back(tsda->GetPoint2Abs());
                    buf.len.push_back(tsda->GetLength());
                    buf.vel.push_back(tsda->GetVelocity());
                    buf.force.push_back(tsda->GetForce());
                    i_tsda++;
                }

                for (const auto& rsda : c->rsdas) {
                    auto& buf = m_rsda_buf[i_rsda];
                    buf.ang.push_back(rsda->GetAngle());
                    buf.vel.push_back(rsda->GetVelocity());
                    buf.torque.push_back(rsda->GetTorque());
                    i_rsda++;
                }
            }

            break;
    }
}

std::string ChOutput::GetFormatAsString(Format type) {
    switch (type) {
        case Format::NONE:
            return "NONE";
        case Format::ASCII:
            return "ASCII";
        case Format::HDF5:
            return "HDF5";
    }
    return "";
}

std::string ChOutput::GetModeAsString(Mode mode) {
    switch (mode) {
        case Mode::FRAMES:
            return "FRAMES";
        case Mode::SERIES:
            return "SERIES";
    }
    return "";
}

}  // end namespace chrono
