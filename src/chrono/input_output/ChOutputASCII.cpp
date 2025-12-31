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
// Base class for a Chrono output database.
//
// =============================================================================

#include <iostream>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono/input_output/ChOutputASCII.h"

namespace chrono {

ChOutputASCII::ChOutputASCII(const std::string& filename)
    : m_file_stream(filename), m_stream(m_file_stream), m_initialized(false) {}

ChOutputASCII::ChOutputASCII(std::ostream& stream) : m_file_stream(), m_stream(stream), m_initialized(false) {}

ChOutputASCII::~ChOutputASCII() {
    if (m_file_stream.is_open())
        m_file_stream.close();
}

void ChOutputASCII::Initialize() {
    m_initialized = true;
}

void ChOutputASCII::WriteTime(int frame, double time) {
    m_stream << "=====================================\n";
    m_stream << "Time: " << time << std::endl;
}

void ChOutputASCII::WriteSection(const std::string& name) {
    m_stream << "  \"" << name << "\"" << std::endl;
}

void ChOutputASCII::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    for (const auto& body : bodies) {
        auto& ref_pos = body->GetFrameRefToAbs().GetPos();
        auto& ref_vel = body->GetFrameRefToAbs().GetPosDt();
        auto& ref_acc = body->GetFrameRefToAbs().GetPosDt2();

        m_stream << "    body auxref: " << body->GetIdentifier() << " \"" << body->GetName() << "\" ";
        m_stream << body->GetPos() << " " << body->GetRot() << " ";
        m_stream << body->GetPosDt() << " " << body->GetAngVelParent() << " ";
        m_stream << body->GetPosDt2() << " " << body->GetAngAccParent() << " ";
        m_stream << ref_pos << " " << ref_vel << " " << ref_acc << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    for (const auto& marker : markers) {
        m_stream << "    marker: " << marker->GetIdentifier() << " \"" << marker->GetName() << "\" ";
        m_stream << marker->GetAbsCoordsys().pos << " ";
        m_stream << marker->GetAbsCoordsysDt().pos << " ";
        m_stream << marker->GetAbsCoordsysDt2().pos << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    for (const auto& shaft : shafts) {
        m_stream << "    shaft: " << shaft->GetIdentifier() << " \"" << shaft->GetName() << "\" ";
        m_stream << shaft->GetPos() << " " << shaft->GetPosDt() << " " << shaft->GetPosDt2() << " ";
        m_stream << shaft->GetAppliedLoad() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    for (const auto& joint : joints) {
        std::vector<double> violations;
        auto C = joint->GetConstraintViolation();
        for (int i = 0; i < C.size(); i++)
            violations.push_back(C(i));

        auto reaction = joint->GetReaction2();
        m_stream << "    joint: " << joint->GetIdentifier() << " \"" << joint->GetName() << "\" ";
        m_stream << reaction.force << " " << reaction.torque << " ";
        for (const auto& val : violations) {
            m_stream << val << " ";
        }
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    for (const auto& couple : couples) {
        m_stream << "    couple: " << couple->GetIdentifier() << " \"" << couple->GetName() << "\" ";
        m_stream << couple->GetRelativePos() << " " << couple->GetRelativePosDt() << " " << couple->GetRelativePosDt2()
                 << " ";
        m_stream << couple->GetReaction1() << " " << couple->GetReaction2() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    for (const auto& spring : springs) {
        m_stream << "    lin spring: " << spring->GetIdentifier() << " \"" << spring->GetName() << "\" ";
        m_stream << spring->GetPoint1Abs() << " " << spring->GetPoint2Abs() << " ";
        m_stream << spring->GetLength() << " " << spring->GetVelocity() << " ";
        m_stream << spring->GetForce() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    for (const auto& spring : springs) {
        m_stream << "    rot spring: " << spring->GetIdentifier() << " \"" << spring->GetName() << "\" ";
        m_stream << spring->GetAngle() << " " << spring->GetVelocity() << " ";
        m_stream << spring->GetTorque() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    for (const auto& load : loads) {
        m_stream << "    body-body load: " << load->GetIdentifier() << " \"" << load->GetName() << "\" ";
        m_stream << load->GetForce() << " " << load->GetTorque() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChOutputASCII::WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    for (const auto& motor : motors) {
        m_stream << "    lin motor: " << motor->GetIdentifier() << " \"" << motor->GetName() << "\" ";
        m_stream << motor->GetMotorPos() << " " << motor->GetMotorPosDt() << " " << motor->GetMotorForce();
        m_stream << std::endl;
    }
}

void ChOutputASCII::WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    for (const auto& motor : motors) {
        m_stream << "    lin motor: " << motor->GetIdentifier() << " \"" << motor->GetName() << "\" ";
        m_stream << motor->GetMotorAngle() << " " << motor->GetMotorAngleDt() << " " << motor->GetMotorTorque();
        m_stream << std::endl;
    }
}

}  // namespace chrono
