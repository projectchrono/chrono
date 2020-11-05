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
// Wraps data received from a flatbuffer sensor message into a corresponding C++
// class.
// See also flatbuffer/fbs/Sensor.fbs
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynSensorMessage.h"

namespace chrono {
namespace synchrono {

namespace Sensor = SynFlatBuffers::Sensor;

/// Constructors
SynSensorMessage::SynSensorMessage(int rank) : SynMessage(rank, SynMessageType::SENSOR) {}

/// Generate state from FlatBuffers message
void SynSensorMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Sensor::SensorBuffer
    if (message->message_type() != SynFlatBuffers::Type_Sensor_State)
        return;

    auto state = message->message_as_Sensor_State();
    auto buffer = state->buffer();

    auto sensor_buffer =
        chrono_types::make_shared<SynSensorMessageState::SensorBuffer>(buffer->Width(), buffer->Height());

    // Buffer() returns flatbuffers::Vector, so Data() returns uint8_t*
    sensor_buffer->Buffer =
        std::vector<uint8_t>(buffer->Buffer()->Data(), buffer->Buffer()->Data() + buffer->Buffer()->size());

    m_state->buffer = sensor_buffer;
    m_state->type = static_cast<SynSensorMessageState::Type>(state->type());
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynSensorMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    auto sensor_type = static_cast<Sensor::Type>(m_state->type);
    auto sensor_buffer = Sensor::CreateSensorBufferDirect(builder,                    //
                                                          m_state->buffer->Width,     //
                                                          m_state->buffer->Height,    //
                                                          &m_state->buffer->Buffer);  //

    auto flatbuffer_state = Sensor::CreateState(builder, sensor_buffer, sensor_type);
    auto flatbuffer_message = SynFlatBuffers::CreateMessage(builder,                            //
                                                            SynFlatBuffers::Type_Sensor_State,  //
                                                            flatbuffer_state.Union(),           //
                                                            m_rank);                            //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

}  // namespace synchrono
}  // namespace chrono
