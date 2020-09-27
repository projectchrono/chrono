#include "chrono_synchrono/agent/SynVehicleAgent.h"

#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/terrain/SynTerrainFactory.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

SynVehicleAgent::SynVehicleAgent(unsigned int rank, ChSystem* system) : SynAgent(rank, SynAgentType::VEHICLE, system) {}

void SynVehicleAgent::Advance(double time_of_next_sync) {
    while (m_system->GetChTime() < time_of_next_sync) {
        double time = m_system->GetChTime();

        ChDriver::Inputs driver_inputs = m_brain->GetDriverInputs();

        m_brain->Synchronize(time);
        m_terrain->Synchronize(time);
        Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        m_brain->Advance(STEP_SIZE);
        m_terrain->Advance(STEP_SIZE);
        GetVehicle()->Advance(STEP_SIZE);

        m_vis_manager->Update(STEP_SIZE);
    }

    GetVehicle()->Update();
}

// std::shared_ptr<SynMessageState> SynVehicleAgent::GetState() {
//     m_msg = chrono_types::make_shared<SynWheeledVehicleMessage>(m_rank, GetVehicle()->GetState());
//     return m_msg->GetState();
// }

void SynVehicleAgent::ProcessMessage(SynMessage* msg) {
    switch (msg->GetType()) {
        case SynMessageType::SCM_TERRAIN:
            m_terrain->ProcessMessage(msg);
            break;
        default:
            m_brain->ProcessMessage(msg);
            break;
    }
}

// void SynVehicleAgent::GenerateMessagesToSend(std::vector<SynMessage*>& messages) {
//     m_terrain->GenerateMessagesToSend(messages, m_rank);
//     m_brain->GenerateMessagesToSend(messages);

//     messages.push_back(new SynWheeledVehicleMessage(m_rank, GetVehicle()->GetState()));
// }

Document SynVehicleAgent::ParseVehicleAgentFileJSON(const std::string& filename) {
    // Open and parse the input file
    auto d = SynAgent::ParseAgentFileJSON(filename);

    // Read subtype
    std::string type = d["Type"].GetString();
    std::string subtype = d["Template"].GetString();
    assert(type.compare("Agent") == 0);
    assert(subtype.compare("VehicleAgent") == 0);

    // ----------------------------
    // Validations of the JSON file
    // ----------------------------
    assert(d.HasMember("Vehicle Input File"));
    assert(d.HasMember("Terrain Input File"));
    // assert(d.HasMember("Brain Input File"));

    return d;
}

void SynVehicleAgent::VehicleAgentFromJSON(Document& d) {
    // Create the terrain
    std::string terrain_filename = d["Terrain Input File"].GetString();
    m_terrain = SynTerrainFactory::CreateTerrain(m_system, GetSynDataFile(terrain_filename));

    // Create the brain
    auto driver = chrono_types::make_shared<ChDriver>(GetChVehicle());
    m_brain = chrono_types::make_shared<SynVehicleBrain>(m_rank, driver, GetChVehicle());
}

}  // namespace synchrono
}  // namespace chrono
