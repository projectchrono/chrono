#include "chrono_synchrono/flatbuffer/message/SynSimulationMessage.h"

namespace chrono {
namespace synchrono {

namespace Simulation = SynFlatBuffers::Simulation;

/// Constructors
SynSimulationMessage::SynSimulationMessage(unsigned int source_id, unsigned int destination_id, bool quit_sim)
    : SynMessage(source_id, destination_id), m_quit_sim(quit_sim) {}

SynSimulationMessage::~SynSimulationMessage() {}

void SynSimulationMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Terrain::SCM::State
    if (message->message_type() != SynFlatBuffers::Type_Simulation_State)
        return;

    m_source_id = message->source_id();
    m_destination_id = message->destination_id();

    m_quit_sim = message->message_as_Simulation_State()->quit_sim();
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynSimulationMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto flatbuffer_state = Simulation::CreateState(builder, m_quit_sim);
    auto flatbuffer_message = SynFlatBuffers::CreateMessage(builder,                                //
                                                            SynFlatBuffers::Type_Simulation_State,  //
                                                            flatbuffer_state.Union(),               //
                                                            m_source_id, m_destination_id);         //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

}  // namespace synchrono
}  // namespace chrono
