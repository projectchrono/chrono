#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"

#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/terrain/SynTerrainFactory.h"

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

SynWheeledVehicleAgent::SynWheeledVehicleAgent(unsigned int rank, const std::string& filename, ChSystem* system)
    : SynVehicleAgent(rank, system) {
    VehicleAgentFromJSON(filename, system);

    m_msg = chrono_types::make_shared<SynWheeledVehicleMessage>(rank,                               //
                                                                filename,                           //
                                                                m_wheeled_vehicle->m_state,         //
                                                                m_wheeled_vehicle->m_description);  //
}

SynWheeledVehicleAgent::SynWheeledVehicleAgent(unsigned int rank,
                                               const std::string& filename,
                                               ChContactMethod contact_method)
    : SynVehicleAgent(rank) {
    VehicleAgentFromJSON(filename, contact_method);

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
    m_wheeled_vehicle = chrono_types::make_shared<SynWheeledVehicle>(GetSynDataFile(vehicle_filename));
}

void SynWheeledVehicleAgent::VehicleAgentFromJSON(const std::string& filename, ChSystem* system) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_wheeled_vehicle = chrono_types::make_shared<SynWheeledVehicle>(GetSynDataFile(vehicle_filename), system);

    // Create the rest of the agent
    SynVehicleAgent::VehicleAgentFromJSON(d);
}

void SynWheeledVehicleAgent::VehicleAgentFromJSON(const std::string& filename, ChContactMethod contact_method) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_wheeled_vehicle = chrono_types::make_shared<SynWheeledVehicle>(GetSynDataFile(vehicle_filename), contact_method);

    m_system = m_wheeled_vehicle->GetSystem();

    // Create the rest of the agent
    SynVehicleAgent::VehicleAgentFromJSON(d);
}
}  // namespace synchrono
}  // namespace chrono
