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
// Implementation of ASCII Chrono output database.
//
// =============================================================================

#include <iostream>

#include "chrono/input_output/ChOutputASCII.h"

using std::endl;

namespace chrono {

ChOutputASCII::ChOutputASCII(const std::string& filename) : m_file_stream(filename), m_stream(m_file_stream), m_initialized(false) {}

ChOutputASCII::ChOutputASCII(std::ostream& stream) : m_file_stream(), m_stream(stream), m_initialized(false) {}

ChOutputASCII::~ChOutputASCII() {
    if (m_file_stream.is_open())
        m_file_stream.close();
}

void ChOutputASCII::Initialize(Mode mode) {
    ChOutput::Initialize(mode);
    m_initialized = true;
}

void ChOutputASCII::WriteTime(int frame, double time) {
    m_stream << "=====================================\n";
    m_stream << "Time: " << time << endl;
}

void ChOutputASCII::WriteSection(const std::string& name) {
    m_stream << "  \"" << name << "\"" << endl;
}

void ChOutputASCII::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    for (const auto& body : bodies) {
        auto& ref_frame = body->GetFrameRefToAbs();

        m_stream << "Body " << body->GetIdentifier() << " '" << body->GetName() << "'" << endl;
        m_stream << "  COM frame: " << body->GetPos() << "  |  " << body->GetRot() << endl;
        m_stream << "  COM vel:   " << body->GetPosDt() << "  |  " << body->GetAngVelParent() << endl;
        m_stream << "  COM acc:   " << body->GetPosDt2() << "  |  " << body->GetAngAccParent() << endl;
        m_stream << "  REF frame: " << ref_frame.GetPos() << "  |  " << ref_frame.GetRot() << endl;
        m_stream << "  REF vel:   " << ref_frame.GetPosDt() << "  |  " << ref_frame.GetAngVelParent() << endl;
        m_stream << "  REF acc:   " << ref_frame.GetPosDt2() << "  |  " << ref_frame.GetAngAccParent() << endl;
    }
}

void ChOutputASCII::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    for (const auto& marker : markers) {
        m_stream << "Marker " << marker->GetIdentifier() << " '" << marker->GetName() << "'" << endl;
        m_stream << "  POS: " << marker->GetAbsCoordsys().pos << endl;
        m_stream << "  VEL: " << marker->GetAbsCoordsysDt().pos << endl;
        m_stream << "  ACC: " << marker->GetAbsCoordsysDt2().pos << endl;
    }
}

void ChOutputASCII::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    for (const auto& shaft : shafts) {
        m_stream << "Shaft " << shaft->GetIdentifier() << " '" << shaft->GetName() << "'" << endl;
        m_stream << "  POS/VEL/ACC: " << shaft->GetPos() << " " << shaft->GetPosDt() << " " << shaft->GetPosDt2() << endl;
        m_stream << "  LOAD:        " << shaft->GetAppliedLoad() << endl;
    }
}

void ChOutputASCII::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    for (const auto& joint : joints) {
        std::vector<double> violations;
        auto C = joint->GetConstraintViolation();
        for (int i = 0; i < C.size(); i++)
            violations.push_back(C(i));

        m_stream << "Joint " << joint->GetIdentifier() << " '" << joint->GetName() << "'" << endl;
        ////m_stream << "  BODIES 1/2:            " << joint->GetBody1()->GetName() << " " << joint->GetBody2()->GetName() << endl;
        m_stream << "  REACTION FORCE 1/2:  " << joint->GetReaction1().force << "  |  " << joint->GetReaction2().force << endl;
        m_stream << "  REACTION TORQUE 1/2: " << joint->GetReaction1().torque << "  |  " << joint->GetReaction2().torque << endl;
        m_stream << "  VIOLATIONS:      ";
        for (const auto& val : violations)
            m_stream << val << " ";
        m_stream << endl;
    }
}

void ChOutputASCII::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    for (const auto& couple : couples) {
        m_stream << "Couple " << couple->GetIdentifier() << " '" << couple->GetName() << "'" << endl;
        m_stream << "  SHAFTS 1/2:      " << couple->GetShaft1()->GetName() << " " << couple->GetShaft2()->GetName() << endl;
        m_stream << "  REL POS/VEL/ACC: " << couple->GetRelativePos() << " " << couple->GetRelativePosDt() << " " << couple->GetRelativePosDt2() << endl;
        m_stream << "  REACTION 1/2:    " << couple->GetReaction1() << " " << couple->GetReaction2() << endl;
    }
}

void ChOutputASCII::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    for (const auto& spring : springs) {
        m_stream << "TSDA " << spring->GetIdentifier() << " '" << spring->GetName() << "'" << endl;
        ////m_stream << "  BODIES 1/2: " << spring->GetBody1()->GetName() << " " << spring->GetBody2()->GetName() << endl;
        m_stream << "  POINTS 1/2: " << spring->GetPoint1Abs() << "  |  " << spring->GetPoint2Abs() << endl;
        m_stream << "  LEN/VEL:    " << spring->GetLength() << " " << spring->GetVelocity() << endl;
        m_stream << "  FORCE:      " << spring->GetForce() << endl;
    }
}

void ChOutputASCII::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    for (const auto& spring : springs) {
        m_stream << "RSDA " << spring->GetIdentifier() << " '" << spring->GetName() << "'" << endl;
        ////m_stream << "  BODIES 1/2: " << spring->GetBody1()->GetName() << " " << spring->GetBody2()->GetName() << endl;
        m_stream << "  ANG/VEL: " << spring->GetAngle() << " " << spring->GetVelocity() << endl;
        m_stream << "  TORQUE:  " << spring->GetTorque() << endl;
    }
}

void ChOutputASCII::WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    for (const auto& load : loads) {
        m_stream << "Body-body load " << load->GetIdentifier() << " '" << load->GetName() << "'" << endl;
        m_stream << "  FORCE:  " << load->GetForce() << endl;
        m_stream << "  TORQUE: " << load->GetTorque() << endl;
    }
}

void ChOutputASCII::WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    for (const auto& motor : motors) {
        m_stream << "Lin motor " << motor->GetIdentifier() << " '" << motor->GetName() << "'" << endl;
        m_stream << "  POS/VEL: " << motor->GetMotorPos() << " " << motor->GetMotorPosDt() << endl;
        m_stream << "  FORCE:   " << motor->GetMotorForce() << endl;
    }
}

void ChOutputASCII::WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    for (const auto& motor : motors) {
        m_stream << "Rot motor " << motor->GetIdentifier() << " '" << motor->GetName() << "'" << endl;
        m_stream << "  ANG/VEL: " << motor->GetMotorAngle() << " " << motor->GetMotorAngleDt() << endl;
        m_stream << "  TORQUE:  " << motor->GetMotorTorque() << endl;
    }
}

}  // namespace chrono
