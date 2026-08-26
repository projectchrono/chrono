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

#include <numeric>

#include "chrono/input_output/ChOutput.h"
#include "chrono/utils/ChUtils.h"

using std::cout;
using std::cerr;
using std::endl;

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
                    const auto& ref_frame = body->GetFrameRefToAbs();
                    const auto& p = ref_frame.GetPos();
                    const auto r = ref_frame.GetRot().GetCardanAnglesXYZ();
                    const auto& v = ref_frame.GetPosDt();
                    const auto w = ref_frame.GetAngVelParent();
                    auto& buf = m_body_buf[i_body++];
                    buf.name = body->GetName();
                    buf.pos.insert(buf.pos.end(), {p.x(), p.y(), p.z()});
                    buf.rot.insert(buf.rot.end(), {r.x(), r.y(), r.z()});
                    buf.lin_vel.insert(buf.lin_vel.end(), {v.x(), v.y(), v.z()});
                    buf.ang_vel.insert(buf.ang_vel.end(), {w.x(), w.y(), w.z()});
                }

                for (const auto& shaft : c->shafts) {
                    auto& buf = m_shaft_buf[i_shaft++];
                    buf.name = shaft->GetName();
                    buf.pos.push_back(shaft->GetPos());
                    buf.vel.push_back(shaft->GetPosDt());
                }

                for (const auto& joint : c->joints) {
                    const auto& f1 = joint->GetReaction1().force;
                    const auto& t1 = joint->GetReaction1().torque;
                    const auto& f2 = joint->GetReaction2().force;
                    const auto& t2 = joint->GetReaction2().torque;
                    auto& buf = m_joint_buf[i_joint++];
                    buf.name = joint->GetName();
                    buf.force1.insert(buf.force1.end(), {f1.x(), f1.y(), f1.z()});
                    buf.torque1.insert(buf.torque1.end(), {t1.x(), t1.y(), t1.z()});
                    buf.force2.insert(buf.force2.end(), {f2.x(), f2.y(), f2.z()});
                    buf.torque2.insert(buf.torque2.end(), {t2.x(), t2.y(), t2.z()});
                }

                for (const auto& tsda : c->tsdas) {
                    const auto& p1 = tsda->GetPoint1Abs();
                    const auto& p2 = tsda->GetPoint2Abs();
                    auto& buf = m_tsda_buf[i_tsda++];
                    buf.name = tsda->GetName();
                    buf.point1.insert(buf.point1.end(), {p1.x(), p1.y(), p1.z()});
                    buf.point2.insert(buf.point2.end(), {p2.x(), p2.y(), p2.z()});
                    buf.len.push_back(tsda->GetLength());
                    buf.vel.push_back(tsda->GetVelocity());
                    buf.force.push_back(tsda->GetForce());
                }

                for (const auto& rsda : c->rsdas) {
                    auto& buf = m_rsda_buf[i_rsda++];
                    buf.name = rsda->GetName();
                    buf.ang.push_back(rsda->GetAngle());
                    buf.vel.push_back(rsda->GetVelocity());
                    buf.torque.push_back(rsda->GetTorque());
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

// -----------------------------------------------------------------------------

ChOutput::Settings::Settings() : format(ChOutput::Format::NONE), mode(ChOutput::Mode::FRAMES), fps(100) {}

ChOutput::Settings::Settings(const Settings& other) {
    format = other.format;
    mode = other.mode;
    fps = other.fps;
}

ChOutput::Settings& ChOutput::Settings::operator=(const Settings& other) {
    format = other.format;
    mode = other.mode;
    fps = other.fps;
    return *this;
}

#ifdef CHRONO_HAS_YAML

static ChOutput::Format ReadOutputFormat(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "ASCII")
        return ChOutput::Format::ASCII;
    if (type == "HDF5")
        return ChOutput::Format::HDF5;
    return ChOutput::Format::NONE;
}

static ChOutput::Mode ReadOutputMode(const YAML::Node& a) {
    auto mode = ChToUpper(a.as<std::string>());
    if (mode == "SERIES")
        return ChOutput::Mode::SERIES;
    if (mode == "FRAMES")
        return ChOutput::Mode::FRAMES;
    return ChOutput::Mode::FRAMES;
}

ChOutput::Settings::Settings(const YAML::Node& a) : Settings() {
    ChAssertAlways(a["format"]);
    format = ReadOutputFormat(a["format"]);
    #ifndef CHRONO_HAS_HDF5
    if (format == ChOutput::Format::HDF5) {
        std::cerr << "HDF5 output support not available.\nOutput disabled." << std::endl;
        format = ChOutput::Format::NONE;
        return;
    }
    #endif

    if (a["mode"])
        mode = ReadOutputMode(a["mode"]);
    if (a["fps"])
        fps = a["fps"].as<double>();
}

ChOutput::Settings ChOutput::Settings::Read(const YAML::Node& a) {
    Settings params(a);
    return params;
}

#endif

void ChOutput::Settings::PrintInfo() const {
    if (format == ChOutput::Format::NONE) {
        cout << "no output" << endl;
        return;
    }

    cout << "output" << endl;
    cout << "  format:               " << ChOutput::GetFormatAsString(format) << endl;
    cout << "  mode:                 " << ChOutput::GetModeAsString(mode) << endl;
    cout << "  output FPS:           " << fps << endl;
}

}  // end namespace chrono
