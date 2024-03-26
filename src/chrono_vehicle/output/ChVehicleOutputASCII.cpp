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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a vehicle output database.
//
// =============================================================================

#include <iostream>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

namespace chrono {
namespace vehicle {

ChVehicleOutputASCII::ChVehicleOutputASCII(const std::string& filename)
    : m_file_stream(filename), m_stream(m_file_stream) {}

ChVehicleOutputASCII::ChVehicleOutputASCII(std::ostream& stream) : m_file_stream(), m_stream(stream) {}

ChVehicleOutputASCII::~ChVehicleOutputASCII() {
    if (m_file_stream.is_open())
        m_file_stream.close();
}

void ChVehicleOutputASCII::WriteTime(int frame, double time) {
    m_stream << "=====================================\n";
    m_stream << "Time: " << time << std::endl;
}

void ChVehicleOutputASCII::WriteSection(const std::string& name) {
    m_stream << "  \"" << name << "\"" << std::endl;
}

void ChVehicleOutputASCII::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    for (auto body : bodies) {
        m_stream << "    body: " << body->GetIdentifier() << " \"" << body->GetName() << "\" ";
        m_stream << body->GetPos() << " " << body->GetRot() << " ";
        m_stream << body->GetPosDt() << " " << body->GetAngVelParent() << " ";
        m_stream << body->GetPosDt2() << " " << body->GetAngAccParent() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) {
    for (auto body : bodies) {
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

void ChVehicleOutputASCII::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    for (auto marker : markers) {
        m_stream << "    marker: " << marker->GetIdentifier() << " \"" << marker->GetName() << "\" ";
        m_stream << marker->GetAbsCoordsys().pos << " ";
        m_stream << marker->GetAbsCoordsysDt().pos << " ";
        m_stream << marker->GetAbsCoordsysDt2().pos << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    for (auto shaft : shafts) {
        m_stream << "    shaft: " << shaft->GetIdentifier() << " \"" << shaft->GetName() << "\" ";
        m_stream << shaft->GetPos() << " " << shaft->GetPosDt() << " " << shaft->GetPosDt2() << " ";
        m_stream << shaft->GetAppliedLoad() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
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

void ChVehicleOutputASCII::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    for (auto couple : couples) {
        m_stream << "    couple: " << couple->GetIdentifier() << " \"" << couple->GetName() << "\" ";
        m_stream << couple->GetRelativePos() << " " << couple->GetRelativePosDt() << " " << couple->GetRelativePosDt2()
                 << " ";
        m_stream << couple->GetReaction1() << " " << couple->GetReaction2() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    for (auto spring : springs) {
        m_stream << "    lin spring: " << spring->GetIdentifier() << " \"" << spring->GetName() << "\" ";
        m_stream << spring->GetPoint1Abs() << " " << spring->GetPoint2Abs() << " ";
        m_stream << spring->GetLength() << " " << spring->GetVelocity() << " ";
        m_stream << spring->GetForce() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    for (auto spring : springs) {
        m_stream << "    rot spring: " << spring->GetIdentifier() << " \"" << spring->GetName() << "\" ";
        m_stream << spring->GetAngle() << " " << spring->GetVelocity() << " ";
        m_stream << spring->GetTorque() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    for (auto load : loads) {
        m_stream << "    body-body load: " << load->GetIdentifier() << " \"" << load->GetName() << "\" ";
        m_stream << load->GetForce() << " " << load->GetTorque() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

}  // end namespace vehicle
}  // end namespace chrono
