#include "chrono_synchrono/flatbuffer/message/SynSimulationMessage.h"

namespace chrono {
namespace synchrono {

namespace Simulation = SynFlatBuffers::Simulation;

/// Constructors
SynSimulationMessage::SynSimulationMessage(AgentKey source_key, AgentKey destination_key, bool quit_sim)
    : SynMessage(source_key, destination_key), m_quit_sim(quit_sim) {}

SynSimulationMessage::~SynSimulationMessage() {}

void SynSimulationMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Terrain::SCM::State
    if (message->message_type() != SynFlatBuffers::Type_Simulation_State)
        return;

    m_source_key = AgentKey(message->source_key());
    m_destination_key = AgentKey(message->destination_key());

    m_quit_sim = message->message_as_Simulation_State()->quit_sim();
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynSimulationMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto flatbuffer_state = Simulation::CreateState(builder, m_quit_sim);
    auto flatbuffer_message =
        SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Simulation_State, flatbuffer_state.Union(),
                                      m_source_key.GetFlatbuffersKey(), m_destination_key.GetFlatbuffersKey());  //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

}  // namespace synchrono
}  // namespace chrono
