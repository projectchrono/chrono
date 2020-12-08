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
// Class for an agent that wraps a Chrono::Vehicle wheeled vehicle. The
// underlying dynamics are those of a wheeled vehicle, state data consists of
// the position and orientation of the COM and the wheels of the vehicle
//
// =============================================================================

#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"

#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/terrain/SynTerrainFactory.h"

using namespace chrono::vehicle;
using namespace rapidjson;

namespace chrono {
namespace synchrono {

SynWheeledVehicleAgent::SynWheeledVehicleAgent(unsigned int rank, ChSystem* system) : SynVehicleAgent(rank, system) {
    // Create zombie vehicle
    m_wheeled_vehicle = chrono_types::make_shared<SynWheeledVehicle>();

    m_msg = chrono_types::make_shared<SynWheeledVehicleMessage>(rank,                               //
                                                                m_wheeled_vehicle->m_state,         //
                                                                m_wheeled_vehicle->m_description);  //
}

SynWheeledVehicleAgent::SynWheeledVehicleAgent(unsigned int rank,
                                               ChCoordsys<> coord_sys,
                                               const std::string& filename,
                                               ChSystem* system)
    : SynVehicleAgent(rank, system) {
    VehicleAgentFromJSON(coord_sys, filename, system);

    m_msg = chrono_types::make_shared<SynWheeledVehicleMessage>(rank,                               //
                                                                filename,                           //
                                                                m_wheeled_vehicle->m_state,         //
                                                                m_wheeled_vehicle->m_description);  //
}

SynWheeledVehicleAgent::SynWheeledVehicleAgent(unsigned int rank,
                                               ChCoordsys<> coord_sys,
                                               const std::string& filename,
                                               ChContactMethod contact_method)
    : SynVehicleAgent(rank) {
    VehicleAgentFromJSON(coord_sys, filename, contact_method);

    m_msg = chrono_types::make_shared<SynWheeledVehicleMessage>(rank,                               //
                                                                filename,                           //
                                                                m_wheeled_vehicle->m_state,         //
                                                                m_wheeled_vehicle->m_description);  //
}

SynWheeledVehicleAgent::SynWheeledVehicleAgent(unsigned int rank, const std::string& filename) : SynVehicleAgent(rank) {
    VehicleAgentFromJSON(filename);

    m_msg = chrono_types::make_shared<SynWheeledVehicleMessage>(rank,                               //
                                                                filename,                           //
                                                                m_wheeled_vehicle->m_state,         //
                                                                m_wheeled_vehicle->m_description);  //
}

void SynWheeledVehicleAgent::Synchronize(double time, ChDriver::Inputs driver_inputs) {
    m_wheeled_vehicle->Synchronize(time, driver_inputs, *m_terrain->GetTerrain());
}

std::shared_ptr<SynMessageState> SynWheeledVehicleAgent::GetState() {
    return m_msg->GetState();
}

void SynWheeledVehicleAgent::GenerateMessagesToSend(std::vector<SynMessage*>& messages) {
    m_terrain->GenerateMessagesToSend(messages, m_rank);
    m_brain->GenerateMessagesToSend(messages);

    messages.push_back(new SynWheeledVehicleMessage(m_rank, m_msg->GetWheeledState()));
}

void SynWheeledVehicleAgent::SetVehicle(std::shared_ptr<SynWheeledVehicle> wheeled_vehicle) {
    m_wheeled_vehicle = wheeled_vehicle;
    m_msg = chrono_types::make_shared<SynWheeledVehicleMessage>(m_rank,                             //
                                                                m_wheeled_vehicle->m_state,         //
                                                                m_wheeled_vehicle->m_description);  //

    if (!m_system && m_wheeled_vehicle->GetSystem())
        m_system = m_wheeled_vehicle->GetSystem();
}

void SynWheeledVehicleAgent::VehicleAgentFromJSON(const std::string& filename) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_wheeled_vehicle = chrono_types::make_shared<SynWheeledVehicle>(synchrono::GetDataFile(vehicle_filename));
}

void SynWheeledVehicleAgent::VehicleAgentFromJSON(ChCoordsys<> coord_sys,
                                                  const std::string& filename,
                                                  ChSystem* system) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_wheeled_vehicle = chrono_types::make_shared<SynWheeledVehicle>(
        coord_sys, synchrono::GetDataFile(vehicle_filename), system);

    // Create the rest of the agent
    SynVehicleAgent::VehicleAgentFromJSON(d);
}

void SynWheeledVehicleAgent::VehicleAgentFromJSON(ChCoordsys<> coord_sys,
                                                  const std::string& filename,
                                                  ChContactMethod contact_method) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_wheeled_vehicle = chrono_types::make_shared<SynWheeledVehicle>(
        coord_sys, synchrono::GetDataFile(vehicle_filename), contact_method);

    m_system = m_wheeled_vehicle->GetSystem();

    // Create the rest of the agent
    SynVehicleAgent::VehicleAgentFromJSON(d);
}
}  // namespace synchrono
}  // namespace chrono
