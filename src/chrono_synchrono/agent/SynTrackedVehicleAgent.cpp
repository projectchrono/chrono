#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"

#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/terrain/SynTerrainFactory.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

SynTrackedVehicleAgent::SynTrackedVehicleAgent(unsigned int rank, ChSystem* system) : SynVehicleAgent(rank, system) {
    // Create zombie vehicle
    m_tracked_vehicle = chrono_types::make_shared<SynTrackedVehicle>();

    m_msg = chrono_types::make_shared<SynTrackedVehicleMessage>(rank,                               //
                                                                m_tracked_vehicle->m_state,         //
                                                                m_tracked_vehicle->m_description);  //
}

SynTrackedVehicleAgent::SynTrackedVehicleAgent(unsigned int rank, const std::string& filename, ChSystem* system)
    : SynVehicleAgent(rank, system) {
    VehicleAgentFromJSON(filename, system);

    m_msg = chrono_types::make_shared<SynTrackedVehicleMessage>(rank,                               //
                                                                filename,                           //
                                                                m_tracked_vehicle->m_state,         //
                                                                m_tracked_vehicle->m_description);  //
}

SynTrackedVehicleAgent::SynTrackedVehicleAgent(unsigned int rank,
                                               const std::string& filename,
                                               ChContactMethod contact_method)
    : SynVehicleAgent(rank) {
    VehicleAgentFromJSON(filename, contact_method);

    m_msg = chrono_types::make_shared<SynTrackedVehicleMessage>(rank,                               //
                                                                filename,                           //
                                                                m_tracked_vehicle->m_state,         //
                                                                m_tracked_vehicle->m_description);  //
}

SynTrackedVehicleAgent::SynTrackedVehicleAgent(unsigned int rank, const std::string& filename) : SynVehicleAgent(rank) {
    VehicleAgentFromJSON(filename);

    m_msg = chrono_types::make_shared<SynTrackedVehicleMessage>(rank,                               //
                                                                filename,                           //
                                                                m_tracked_vehicle->m_state,         //
                                                                m_tracked_vehicle->m_description);  //
}

void SynTrackedVehicleAgent::Synchronize(double time, ChDriver::Inputs driver_inputs) {
    m_tracked_vehicle->Synchronize(time, driver_inputs);
}

std::shared_ptr<SynMessageState> SynTrackedVehicleAgent::GetState() {
    return m_msg->GetState();
}

void SynTrackedVehicleAgent::GenerateMessagesToSend(std::vector<SynMessage*>& messages) {
    m_terrain->GenerateMessagesToSend(messages, m_rank);
    m_brain->GenerateMessagesToSend(messages);

    messages.push_back(new SynTrackedVehicleMessage(m_rank, m_msg->GetTrackedState()));
}

void SynTrackedVehicleAgent::SetVehicle(std::shared_ptr<SynTrackedVehicle> tracked_vehicle) {
    m_tracked_vehicle = tracked_vehicle;
    m_msg = chrono_types::make_shared<SynTrackedVehicleMessage>(m_rank,                             //
                                                                m_tracked_vehicle->m_state,         //
                                                                m_tracked_vehicle->m_description);  //

    if (!m_system && m_tracked_vehicle->GetSystem())
        m_system = m_tracked_vehicle->GetSystem();
}

void SynTrackedVehicleAgent::VehicleAgentFromJSON(const std::string& filename) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_tracked_vehicle = chrono_types::make_shared<SynTrackedVehicle>(GetSynDataFile(vehicle_filename));
}

void SynTrackedVehicleAgent::VehicleAgentFromJSON(const std::string& filename, ChSystem* system) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_tracked_vehicle = chrono_types::make_shared<SynTrackedVehicle>(GetSynDataFile(vehicle_filename), system);

    // Create the rest of the agent
    SynVehicleAgent::VehicleAgentFromJSON(d);
}

void SynTrackedVehicleAgent::VehicleAgentFromJSON(const std::string& filename, ChContactMethod contact_method) {
    // Parse file
    Document d = ParseVehicleAgentFileJSON(filename);

    // Create the vehicle
    std::string vehicle_filename = d["Vehicle Input File"].GetString();
    m_tracked_vehicle = chrono_types::make_shared<SynTrackedVehicle>(GetSynDataFile(vehicle_filename), contact_method);

    m_system = m_tracked_vehicle->GetSystem();

    // Create the rest of the agent
    SynVehicleAgent::VehicleAgentFromJSON(d);
}
}  // namespace synchrono
}  // namespace chrono
