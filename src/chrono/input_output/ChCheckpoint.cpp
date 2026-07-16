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
// Implementation of the base class for a Chrono checkpoint database.
//
// =============================================================================

#include "chrono/input_output/ChCheckpoint.h"

namespace chrono {

ChCheckpoint::ChCheckpoint(Type type) : m_type(type), m_time(0) {}

void ChCheckpoint::Save(ChSystem* sys) {
    ChAssertAlways(m_type == Type::SYSTEM);
    SetTime(sys->GetChTime());
    SaveState(sys);
}

void ChCheckpoint::Save(double time, const ChAssembly::Components& components) {
    ChAssertAlways(m_type == Type::COMPONENT);
    SetTime(time);
    Save(components);
}

void ChCheckpoint::Save(const ChAssembly::Components& components) {
    ChAssertAlways(m_type == Type::COMPONENT);
    SaveBodies(components.bodies);
    SaveShafts(components.shafts);
    SaveJoints(components.joints);
    SaveCouples(components.couples);
    SaveLinSprings(components.tsdas);
    SaveRotSprings(components.rsdas);
    SaveBodyBodyLoads(components.bushings);
    SaveLinMotors(components.lin_motors);
    SaveRotMotors(components.rot_motors);
}

void ChCheckpoint::Load(ChSystem* sys) {
    ChAssertAlways(m_type == Type::SYSTEM);
    LoadState(sys);
    sys->SetChTime(m_time);
}

void ChCheckpoint::Load(double& time, ChAssembly::Components& components) {
    Load(components);
    time = m_time;
}

void ChCheckpoint::Load(ChAssembly::Components& components) {
    ChAssertAlways(m_type == Type::COMPONENT);
    LoadBodies(components.bodies);
    LoadShafts(components.shafts);
    LoadJoints(components.joints);
    LoadCouples(components.couples);
    LoadLinSprings(components.tsdas);
    LoadRotSprings(components.rsdas);
    LoadBodyBodyLoads(components.bushings);
    LoadLinMotors(components.lin_motors);
    LoadRotMotors(components.rot_motors);
}

std::string ChCheckpoint::GetFormatAsString(Format format) {
    switch (format) {
        case Format::ASCII:
            return "ASCII";
    }
    return "";
}

std::string ChCheckpoint::GetTypeAsString(Type type) {
    switch (type) {
        case Type::SYSTEM:
            return "SYSTEM";
        case Type::COMPONENT:
            return "COMPONENT";
    }
    return "";
}

void ChCheckpoint::CheckIfSystemType() const {
    if (m_type != Type::SYSTEM) {
        std::cerr << "Error: Invalid function call; not a SYSTEM checkpoint" << std::endl;
        throw std::runtime_error("Invalid function call; not a SYSTEM checkpoint");
    }
}

void ChCheckpoint::CheckIfComponentType() const {
    if (m_type != Type::COMPONENT) {
        std::cerr << "Error: Invalid function call; not a COMPONENT checkpoint" << std::endl;
        throw std::runtime_error("Invalid function call; not a COMPONENT checkpoint");
    }
}

}  // end namespace chrono
