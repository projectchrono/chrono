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

#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/agent/SynCopterAgent.h"
#include "chrono_synchrono/agent/SynSCMTerrainAgent.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

std::shared_ptr<SynAgent> SynAgentFactory::CreateAgent(std::shared_ptr<SynMessage> description) {
    // Create the agent
    std::shared_ptr<SynAgent> agent = nullptr;

    // For convenience
    auto source_id = description->GetSourceID();

    /// TODO: Add json support

    // Parse the JSON file if it exists
    // if (description->json.compare("") != 0) {
    //     std::cout << description->json << std::endl;
    //     // Parse JSON file to get the agent type
    //     Document d = SynAgent::ParseAgentFileJSON(description->json);
    //     std::string type = d["Template"].GetString();

    //     if (type.compare("WheeledVehicleAgent") == 0) {
    //         agent = chrono_types::make_shared<SynWheeledVehicleAgent>(source_id, description->json);
    //     } else if (type.compare("TrackedVehicleAgent") == 0) {
    //         agent = chrono_types::make_shared<SynTrackedVehicleAgent>(source_id, description->json);
    //     } else {
    //         std::string message = "SynAgentFactory::CreateAgent: Agent type \"" + type + "\" not recognized.";
    //         throw ChException(message);
    //     }
    // } else
    if (auto wv_description = std::dynamic_pointer_cast<SynWheeledVehicleDescriptionMessage>(description)) {
        auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>();
        vehicle_agent->SetID(source_id);
        vehicle_agent->SetZombieVisualizationFiles(wv_description->chassis_vis_file,  //
                                                   wv_description->wheel_vis_file,    //
                                                   wv_description->tire_vis_file);    //
        vehicle_agent->SetNumWheels(wv_description->num_wheels);

        agent = vehicle_agent;
    } else if (auto tv_description = std::dynamic_pointer_cast<SynTrackedVehicleDescriptionMessage>(description)) {
        auto vehicle_agent = chrono_types::make_shared<SynTrackedVehicleAgent>();
        vehicle_agent->SetID(source_id);
        vehicle_agent->SetZombieVisualizationFiles(tv_description->chassis_vis_file,            //
                                                   tv_description->track_shoe_vis_file,         //
                                                   tv_description->left_sprocket_vis_file,      //
                                                   tv_description->right_sprocket_vis_file,     //
                                                   tv_description->left_idler_vis_file,         //
                                                   tv_description->right_idler_vis_file,        //
                                                   tv_description->left_road_wheel_vis_file,    //
                                                   tv_description->right_road_wheel_vis_file);  //

        vehicle_agent->SetNumAssemblyComponents(tv_description->num_track_shoes,   //
                                                tv_description->num_sprockets,     //
                                                tv_description->num_idlers,        //
                                                tv_description->num_road_wheels);  //

        agent = vehicle_agent;
    } else if (auto copter_description = std::dynamic_pointer_cast<SynCopterDescriptionMessage>(description)) {
        auto copter_agent = chrono_types::make_shared<SynCopterAgent>();
        copter_agent->SetID(source_id);
        copter_agent->SetZombieVisualizationFiles(copter_description->chassis_vis_file,  //
                                                  copter_description->propeller_vis_file);  //

        copter_agent->SetNumProps(copter_description->GetNumProps());
		agent = copter_agent;
    } else if (auto terrain_message = std::dynamic_pointer_cast<SynSCMMessage>(description)) {
        auto terrain_agent = chrono_types::make_shared<SynSCMTerrainAgent>();

        agent = terrain_agent;
    } else if (auto env_message = std::dynamic_pointer_cast<SynEnvironmentMessage>(description)) {
        agent = chrono_types::make_shared<SynEnvironmentAgent>(nullptr);
        agent->SetID(source_id);
    } else {
        std::string message = "SynAgentFactory::CreateAgent: Passed SynAgentDescription is not supported.";
        throw ChException(message);
    }

    return agent;
}

}  // namespace synchrono
}  // namespace chrono
