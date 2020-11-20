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
// Class that uses either json files or a flatbuffer description message to
// construct a SynAgent that will be of the corresponding underlying type.
// - Communication managers use the construct-from-message functionality in the
//      setup phase when they receive description messages from other ranks
// - Demos use the json functionality to construct agents
//
// =============================================================================

#include "chrono_synchrono/agent/SynAgentFactory.h"

#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

std::shared_ptr<SynAgent> SynAgentFactory::CreateAgent(SynAgentMessage* message) {
    // Create the agent
    std::shared_ptr<SynAgent> agent = nullptr;

    // For convenience
    auto description = message->GetDescription();
    auto rank = message->GetRank();
    auto type = message->GetType();

    // Parse the JSON file if it exists
    if (description->json.compare("") != 0) {
        // Parse JSON file to get the agent type
        Document d = SynAgent::ParseAgentFileJSON(description->json);
        std::string type = d["Template"].GetString();

        if (type.compare("WheeledVehicleAgent") == 0) {
            agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, description->json);
        } else if (type.compare("TrackedVehicleAgent") == 0) {
            agent = chrono_types::make_shared<SynTrackedVehicleAgent>(rank, description->json);
        } else {
            std::string message = "SynAgentFactory::CreateAgent: Agent type \"" + type + "\" not recognized.";
            throw ChException(message);
        }
    } else if (type == SynMessageType::WHEELED_VEHICLE) {
        auto vehicle_description = std::static_pointer_cast<SynWheeledVehicleDescription>(description);
        auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
        vehicle_agent->GetWheeledVehicle()->SetZombieVisualizationFiles(vehicle_description->m_chassis_vis_file,  //
                                                                        vehicle_description->m_wheel_vis_file,    //
                                                                        vehicle_description->m_tire_vis_file);    //
        vehicle_agent->GetWheeledVehicle()->SetNumWheels(vehicle_description->m_num_wheels);

        agent = vehicle_agent;
    } else if (type == SynMessageType::TRACKED_VEHICLE) {
        auto vehicle_description = std::static_pointer_cast<SynTrackedVehicleDescription>(description);
        auto vehicle_agent = chrono_types::make_shared<SynTrackedVehicleAgent>(rank);
        vehicle_agent->GetTrackedVehicle()->SetZombieVisualizationFiles(
            vehicle_description->m_chassis_vis_file,            //
            vehicle_description->m_track_shoe_vis_file,         //
            vehicle_description->m_left_sprocket_vis_file,      //
            vehicle_description->m_right_sprocket_vis_file,     //
            vehicle_description->m_left_idler_vis_file,         //
            vehicle_description->m_right_idler_vis_file,        //
            vehicle_description->m_left_road_wheel_vis_file,    //
            vehicle_description->m_right_road_wheel_vis_file);  //

        vehicle_agent->GetTrackedVehicle()->SetNumAssemblyComponents(vehicle_description->m_num_track_shoes,   //
                                                                     vehicle_description->m_num_sprockets,     //
                                                                     vehicle_description->m_num_idlers,        //
                                                                     vehicle_description->m_num_road_wheels);  //

        agent = vehicle_agent;
    } else if (type == SynMessageType::ENVIRONMENT) {
        agent = chrono_types::make_shared<SynEnvironmentAgent>(rank);
    } else {
        std::string message = "SynAgentFactory::CreateAgent: Passed SynAgentDescription is not supported.";
        throw ChException(message);
    }

    return agent;
}

std::shared_ptr<SynAgent> SynAgentFactory::CreateAgent(unsigned int rank,
                                                       ChCoordsys<> coord_sys,
                                                       const std::string& filename,
                                                       ChSystem* system) {
    // Create the agent
    std::shared_ptr<SynAgent> agent;

    // Parse JSON file to get the agent type
    Document d = SynAgent::ParseAgentFileJSON(filename);
    std::string type = d["Template"].GetString();

    if (type.compare("VehicleAgent") == 0) {
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, coord_sys, filename, system);
    } else {
        std::string message = "Agent type \"" + type + "\" not recognized.";
        throw ChException(message);
    }

    return agent;
}

std::shared_ptr<SynAgent> SynAgentFactory::CreateAgent(unsigned int rank,
                                                       ChCoordsys<> coord_sys,
                                                       const std::string& filename,
                                                       ChContactMethod contact_method) {
    // Create the agent
    std::shared_ptr<SynAgent> agent;

    // Parse JSON file to get the agent type
    Document d = SynAgent::ParseAgentFileJSON(filename);
    std::string type = d["Template"].GetString();

    if (type.compare("VehicleAgent") == 0) {
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, coord_sys, filename, contact_method);
    } else {
        std::string message = "Agent type \"" + type + "\" not recognized.";
        throw ChException(message);
    }

    return agent;
}

}  // namespace synchrono
}  // namespace chrono
