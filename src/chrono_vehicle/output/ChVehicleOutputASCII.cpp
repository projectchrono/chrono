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

ChVehicleOutputASCII::ChVehicleOutputASCII(const std::string& filename) : m_file_stream(filename), m_stream(m_file_stream) {}

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
        m_stream << "    body: " << body->GetIdentifier() << " \"" << body->GetNameString() << "\" ";
        m_stream << body->GetPos() << " " << body->GetRot() << " ";
        m_stream << body->GetPos_dt() << " " << body->GetWvel_par() << " ";
        m_stream << body->GetPos_dtdt() << " " << body->GetWacc_par() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) {
    for (auto body : bodies) {
        auto& ref_pos = body->GetFrame_REF_to_abs().GetPos();
        auto& ref_vel = body->GetFrame_REF_to_abs().GetPos_dt();
        auto& ref_acc = body->GetFrame_REF_to_abs().GetPos_dtdt();

        m_stream << "    body auxref: " << body->GetIdentifier() << " \"" << body->GetNameString() << "\" ";
        m_stream << body->GetPos() << " " << body->GetRot() << " ";
        m_stream << body->GetPos_dt() << " " << body->GetWvel_par() << " ";
        m_stream << body->GetPos_dtdt() << " " << body->GetWacc_par() << " ";
        m_stream << ref_pos << " " << ref_vel << " " << ref_acc << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    for (auto marker : markers) {
        m_stream << "    marker: " << marker->GetIdentifier() << " \"" << marker->GetNameString() << "\" ";
        m_stream << marker->GetAbsCoord().pos << " ";
        m_stream << marker->GetAbsCoord_dt().pos << " ";
        m_stream << marker->GetAbsCoord_dtdt().pos << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    for (auto shaft : shafts) {
        m_stream << "    shaft: " << shaft->GetIdentifier() << " \"" << shaft->GetNameString() << "\" ";
        m_stream << shaft->GetPos() << " " << shaft->GetPos_dt() << " " << shaft->GetPos_dtdt() << " ";
        m_stream << shaft->GetAppliedTorque() << " ";
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

        m_stream << "    joint: " << joint->GetIdentifier() << " \"" << joint->GetNameString() << "\" ";
        m_stream << joint->Get_react_force() << " " << joint->Get_react_torque() << " ";
        for (const auto& val : violations) {
            m_stream << val << " ";
        }
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    for (auto couple : couples) {
        m_stream << "    couple: " << couple->GetIdentifier() << " \"" << couple->GetNameString() << "\" ";
        m_stream << couple->GetRelativeRotation() << " " << couple->GetRelativeRotation_dt() << " "
                 << couple->GetRelativeRotation_dtdt() << " ";
        m_stream << couple->GetTorqueReactionOn1() << " " << couple->GetTorqueReactionOn2() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    for (auto spring : springs) {
        m_stream << "    lin spring: " << spring->GetIdentifier() << " \"" << spring->GetNameString() << "\" ";
        m_stream << spring->GetPoint1Abs() << " " << spring->GetPoint2Abs() << " ";
        m_stream << spring->GetLength() << " " << spring->GetVelocity() << " ";
        m_stream << spring->GetForce() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    for (auto spring : springs) {
        m_stream << "    rot spring: " << spring->GetIdentifier() << " \"" << spring->GetNameString() << "\" ";
        m_stream << spring->GetAngle() << " " << spring->GetVelocity() << " ";
        m_stream << spring->GetTorque() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

void ChVehicleOutputASCII::WriteBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    for (auto load : loads) {
        m_stream << "    body-body load: " << load->GetIdentifier() << " \"" << load->GetNameString() << "\" ";
        m_stream << load->GetForce() << " " << load->GetTorque() << " ";
        m_stream << std::endl;
        //// TODO
    }
}

}  // end namespace vehicle
}  // end namespace chrono
