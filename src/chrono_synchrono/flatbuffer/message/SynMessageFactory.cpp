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
// Called by CommunicationManager to transform an incoming
// SynFlatBuffers::Message into a SynMessage with state information inside of it
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynMessageFactory.h"

#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSCMMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynTrackedVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynCopterMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSimulationMessage.h"

namespace chrono {
namespace synchrono {

std::shared_ptr<SynMessage> SynMessageFactory::GenerateMessage(const SynFlatBuffers::Message* incoming_message) {
    // Create message
    std::shared_ptr<SynMessage> message;

    // Get rank
    unsigned int source_id = incoming_message->source_id();
    unsigned int destination_id = incoming_message->destination_id();

    // Create the correct message
    if (incoming_message->message_type() == SynFlatBuffers::Type_Terrain_State) {
        auto terrain_state = incoming_message->message_as_Terrain_State();
        if (terrain_state->message_type() == SynFlatBuffers::Terrain::Type_SCM_State) {
            message = chrono_types::make_shared<SynSCMMessage>(source_id, destination_id);
        } else {
            throw ChException("SynMessageFactory::GenerateMessage: Unknown TERRAIN type.");
        }
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_Agent_State) {
        const SynFlatBuffers::Agent::State* agent_state = incoming_message->message_as_Agent_State();

        if (agent_state->message_type() == SynFlatBuffers::Agent::Type_WheeledVehicle_State) {
            message = chrono_types::make_shared<SynWheeledVehicleStateMessage>(source_id, destination_id);
        } else if (agent_state->message_type() == SynFlatBuffers::Agent::Type_TrackedVehicle_State) {
            message = chrono_types::make_shared<SynTrackedVehicleStateMessage>(source_id, destination_id);
        } else if (agent_state->message_type() == SynFlatBuffers::Agent::Type_Environment_State) {
            message = chrono_types::make_shared<SynEnvironmentMessage>(source_id, destination_id);
        } else if (agent_state->message_type() == SynFlatBuffers::Agent::Type_Copter_State) {
            message = chrono_types::make_shared<SynCopterStateMessage>(source_id, destination_id);
        } else {
            throw ChException("SynMessageFactory::GenerateMessage: Unknown AGENT STATE type.");
        }
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_Agent_Description) {
        auto agent_description = incoming_message->message_as_Agent_Description();

        if (agent_description->description_type() == SynFlatBuffers::Agent::Type_WheeledVehicle_Description) {
            message = chrono_types::make_shared<SynWheeledVehicleDescriptionMessage>(source_id, destination_id);
        } else if (agent_description->description_type() == SynFlatBuffers::Agent::Type_TrackedVehicle_Description) {
            message = chrono_types::make_shared<SynTrackedVehicleDescriptionMessage>(source_id, destination_id);
        } else if (agent_description->description_type() == SynFlatBuffers::Agent::Type_Environment_Description) {
            message = chrono_types::make_shared<SynEnvironmentMessage>(source_id, destination_id);
        } else if (agent_description->description_type() == SynFlatBuffers::Agent::Type_Copter_Description) {
            message = chrono_types::make_shared<SynCopterDescriptionMessage>(source_id, destination_id);
        } else {
            throw ChException("SynMessageFactory::GenerateMessage: Unknown AGENT DESCRIPTION type.");
        }
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_Simulation_State) {
        message = chrono_types::make_shared<SynSimulationMessage>(source_id, destination_id);
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_SPAT_State) {
        message = chrono_types::make_shared<SynSPATMessage>(source_id, destination_id);
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_MAP_State) {
        message = chrono_types::make_shared<SynMAPMessage>(source_id, destination_id);
    } else {
        throw ChException("SynMessageFactory::GenerateMessage: Unknown type.");
    }

    message->SetMessageType(incoming_message->message_type());
    message->ConvertFromFlatBuffers(incoming_message);

    return message;
}

}  // namespace synchrono
}  // namespace chrono
