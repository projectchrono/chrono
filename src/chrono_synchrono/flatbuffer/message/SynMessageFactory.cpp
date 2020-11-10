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

#include "chrono_synchrono/flatbuffer/message/SynSensorMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSCMMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynTrackedVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

#include "chrono/core/ChLog.h"

namespace chrono {
namespace synchrono {

SynMessage* SynMessageFactory::GenerateMessage(const SynFlatBuffers::Message* incoming_message) {
    // Create message
    SynMessage* message;

    // Get rank
    uint32_t rank = incoming_message->rank();

    // Create the correct message
    if (incoming_message->message_type() == SynFlatBuffers::Type_Terrain_State) {
        auto terrain_state = incoming_message->message_as_Terrain_State();
        if (terrain_state->message_type() == SynFlatBuffers::Terrain::Type_SCM_State) {
            message = new SynSCMMessage(rank);
        } else {
            throw ChException("SynMessageFactory::GenerateMessage: Unknown TERRAIN type");
        }
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_Agent_State) {
        const SynFlatBuffers::Agent::State* agent_state = incoming_message->message_as_Agent_State();

        if (agent_state->message_type() == SynFlatBuffers::Agent::Type_WheeledVehicle_State) {
            message = new SynWheeledVehicleMessage(rank);
        } else if (agent_state->message_type() == SynFlatBuffers::Agent::Type_TrackedVehicle_State) {
            message = new SynTrackedVehicleMessage(rank);
        } else if (agent_state->message_type() == SynFlatBuffers::Agent::Type_Environment_State) {
            message = new SynEnvironmentMessage(rank);
        } else {
            throw ChException("SynMessageFactory::GenerateMessage: Unknown AGENT STATE type. Exitting...");
        }
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_Agent_Description) {
        auto agent_description = incoming_message->message_as_Agent_Description();

        if (agent_description->description_type() == SynFlatBuffers::Agent::Type_WheeledVehicle_Description) {
            message = new SynWheeledVehicleMessage(rank);
        } else if (agent_description->description_type() == SynFlatBuffers::Agent::Type_TrackedVehicle_Description) {
            message = new SynTrackedVehicleMessage(rank);
        } else if (agent_description->description_type() == SynFlatBuffers::Agent::Type_Environment_Description) {
            message = new SynEnvironmentMessage(rank);
        } else {
            throw ChException("SynMessageFactory::GenerateMessage: Unknown AGENT DESCRIPTION type. Exitting...");
        }
        ((SynAgentMessage*)message)->DescriptionFromMessage(incoming_message);
        return message;
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_SPAT_State) {
        message = new SynSPATMessage(rank);
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_MAP_State) {
        message = new SynMAPMessage(rank);
    } else if (incoming_message->message_type() == SynFlatBuffers::Type_Sensor_State) {
        message = new SynSensorMessage(rank);
    } else {
        throw ChException("SynMessageFactory::GenerateMessage: Unknown type. Exitting...");
    }

    message->StateFromMessage(incoming_message);
    return message;
}

}  // namespace synchrono
}  // namespace chrono
