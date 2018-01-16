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

#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

namespace chrono {
namespace vehicle {

ChVehicleOutputASCII::ChVehicleOutputASCII(const std::string& filename) {
    m_stream.open(filename, std::ios_base::out);
}

ChVehicleOutputASCII::~ChVehicleOutputASCII() {
    m_stream.close();
    std::cout << "OK.  Closing output stream." << std::endl;
}

void ChVehicleOutputASCII::WriteTime(double time) {
    m_stream << "=====================================\n";
    m_stream << "Time: " << time << std::endl;
}

void ChVehicleOutputASCII::WriteSection(const std::string& name) {
    m_stream << "  ----------\n";
    m_stream << "  \"" << name << "\"" << std::endl;
}

void ChVehicleOutputASCII::WriteBody(std::shared_ptr<ChBody> body) {
    m_stream << "    body: \"" << body->GetNameString() << "\" ";
    m_stream << body->GetPos() << " " << body->GetRot() << " ";
    m_stream << body->GetPos_dt() << " " << body->GetWvel_par() << std::endl;
    //// TODO
}

void ChVehicleOutputASCII::WriteBodyAuxRef(std::shared_ptr<ChBodyAuxRef> body) {
    //// TODO: include BodyAuxRef specific outputs
    WriteBody(body);
}

void ChVehicleOutputASCII::WriteMarker(std::shared_ptr<ChMarker> marker) {
    m_stream << "    marker: \"" << marker->GetNameString() << "\" ";
    m_stream << marker->GetAbsCoord().pos << " " << marker->GetAbsCoord_dt().pos << std::endl;
    //// TODO
}

void ChVehicleOutputASCII::WriteShaft(std::shared_ptr<ChShaft> shaft) {
    m_stream << "    shaft: \"" << shaft->GetNameString() << "\" ";
    m_stream << std::endl;
    //// TODO
}

void ChVehicleOutputASCII::WriteJoint(std::shared_ptr<ChLink> joint) {
    m_stream << "    joint: \"" << joint->GetNameString() << "\" ";
    m_stream << std::endl;
    //// TODO
}

void ChVehicleOutputASCII::WriteLinSpring(std::shared_ptr<ChLinkSpringCB> spring) {
    m_stream << "    lin spring: \"" << spring->GetNameString() << "\" ";
    m_stream << std::endl;
    //// TODO
}

void ChVehicleOutputASCII::WriteRotSpring(std::shared_ptr<ChLinkRotSpringCB> spring) {
    m_stream << "    rot spring: \"" << spring->GetNameString() << "\" ";
    m_stream << std::endl;
    //// TODO
}

}  // end namespace vehicle
}  // end namespace chrono
