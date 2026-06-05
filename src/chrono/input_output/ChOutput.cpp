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

#include "chrono/input_output/ChOutput.h"

namespace chrono {

ChOutput::ChOutput(Mode mode) : m_mode(mode), m_buf_allocated(false), m_num_times(0) {}

void ChOutput::Write(double time, int frame, const ChAssembly::Components& components) {
    switch (m_mode) {
        case Mode::FRAMES:
            WriteTime(frame, time);
            break;
        case Mode::SERIES:
            m_time.push_back(time);
            break;
    }

    Write(components);
}

void ChOutput::Write(const ChAssembly::Components& components) {
    switch (m_mode) {
        case Mode::FRAMES:
            WriteBodies(components.bodies);
            WriteShafts(components.shafts);
            WriteJoints(components.joints);
            WriteCouples(components.couples);
            WriteBodyBodyLoads(components.bushings);
            ////WriteConstraints(components.constraints);
            WriteLinSprings(components.tsdas);
            WriteRotSprings(components.rsdas);
            WriteLinMotors(components.lin_motors);
            WriteRotMotors(components.rot_motors);

            break;
        case Mode::SERIES:
            if (!m_buf_allocated) {
                m_body_buf.resize(components.bodies.size());
                m_shaft_buf.resize(components.shafts.size());
                m_joint_buf.resize(components.joints.size());
                m_tsda_buf.resize(components.tsdas.size());
                m_rsda_buf.resize(components.rsdas.size());
                m_buf_allocated = true;
            }

            for (size_t i = 0; i < components.bodies.size(); i++) {
                const auto& body = *components.bodies[i];
                auto& buf = m_body_buf[i];
                const auto& ref_frame = body.GetFrameRefToAbs();
                buf.pos.push_back(ref_frame.GetPos());
                buf.rot.push_back(ref_frame.GetRot().GetCardanAnglesXYZ());
                buf.lin_vel.push_back(ref_frame.GetPosDt());
                buf.ang_vel.push_back(ref_frame.GetAngVelParent());
            }

            for (size_t i = 0; i < components.shafts.size(); i++) {
                const auto& shaft = *components.shafts[i];
                auto& buf = m_shaft_buf[i];
                buf.pos.push_back(shaft.GetPos());
                buf.vel.push_back(shaft.GetPosDt());
            }

            for (size_t i = 0; i < components.joints.size(); i++) {
                const auto& joint = *components.joints[i];
                auto& buf = m_joint_buf[i];
                buf.react1.push_back({joint.GetReaction1().force, joint.GetReaction1().torque});
                buf.react2.push_back({joint.GetReaction2().force, joint.GetReaction2().torque});
            }

            for (size_t i = 0; i < components.tsdas.size(); i++) {
                const auto& tsda = *components.tsdas[i];
                auto& buf = m_tsda_buf[i];
                buf.point1.push_back(tsda.GetPoint1Abs());
                buf.point2.push_back(tsda.GetPoint2Abs());
                buf.len.push_back(tsda.GetLength());
                buf.vel.push_back(tsda.GetVelocity());
                buf.force.push_back(tsda.GetForce());
            }

            for (size_t i = 0; i < components.rsdas.size(); i++) {
                const auto& rsda = *components.rsdas[i];
                auto& buf = m_rsda_buf[i];
                buf.ang.push_back(rsda.GetAngle());
                buf.vel.push_back(rsda.GetVelocity());
                buf.torque.push_back(rsda.GetTorque());
            }

            m_num_times++;

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
