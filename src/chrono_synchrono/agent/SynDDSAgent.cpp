#include "chrono_synchrono/agent/SynDDSAgent.h"

#include "chrono_synchrono/communication/dds/SynDDSParticipant.h"

#include "chrono_synchrono/agent/SynAgentFactory.h"
#include "chrono_synchrono/flatbuffer/message/SynMessageFactory.h"

namespace chrono {
namespace synchrono {

// Constructor
SynDDSAgent::SynDDSAgent(std::shared_ptr<SynAgent> agent, bool zombie)
    : SynAgent(agent->GetRank(), agent->GetType(), agent->GetSystem()),
      m_participant(new SynDDSParticipant(std::to_string(m_rank) + std::to_string(zombie))),
      m_agent(agent),
      m_zombie(zombie),
      m_ok(true) {}

// Destructor
SynDDSAgent::~SynDDSAgent() {
    delete m_participant;
}

bool SynDDSAgent::Initialize() {
    m_participant->Initialize();

    // Create a publisher
    // This publisher will be responsible for publishing state information of the agent
    auto publisher =
        new SynDDSPublisher(m_participant->GetParticipant(), std::to_string(m_rank) + std::to_string(m_zombie));
    if (!publisher->Initialize())
        return false;
    m_participant->PushPublisher(publisher);
    m_publisher = publisher;

    m_participant->WaitForMatches();

    // Generate agent description
    if (m_agent) {
        auto msg = m_agent->GetMessage();
        if (!msg)
            return false;
        auto description = msg->MessageFromDescription(m_flatbuffers_manager.GetBuilder());
        m_flatbuffers_manager.AddMessage(description);
    }

    // Complete the FlatBuffer buffer with size included as the first 4 bytes
    m_flatbuffers_manager.FinishSizePrefixed();

    Publish();
    Listen(true);

    m_initialized = true;

    return true;
}

// Synchronize the state of the zombie
// TODO: Send more information than just state
void SynDDSAgent::Advance(double time_of_next_sync) {
    if (!m_participant->IsOk()) {
        m_ok = false;
        return;
    }

    if (!m_zombie) {
        m_agent->Advance(time_of_next_sync);
        m_agent->GenerateMessagesToSend(m_messages);
        for (auto msg : m_messages) {
            m_flatbuffers_manager.AddMessage(msg);
            delete msg;
        }
    }
    m_messages.clear();

    Publish();
    Listen(true);
}

void SynDDSAgent::GenerateMessagesToSend(std::vector<SynMessage*>& messages) {
    if (!m_zombie) {
        m_agent->GenerateMessagesToSend(m_messages);
        for (auto msg : m_messages) {
            m_flatbuffers_manager.AddMessage(msg);
            delete msg;
        }
        m_messages.clear();
    } else {
        for (auto msg : m_messages)
            messages.push_back(msg);
    }
}

void SynDDSAgent::Publish() {
    m_flatbuffers_manager.FinishSizePrefixed();

    SynDDSMessage msg;
    msg.data(m_flatbuffers_manager.ToMessageBuffer());
    msg.rank(m_rank);

    m_publisher->Publish(msg);

    m_flatbuffers_manager.Reset();
}

bool SynDDSAgent::Listen(bool synchronous) {
    bool message_received = false;

    for (auto subscriber : m_participant->GetSubscribers()) {
        // Block if in synchronous mode
        while (synchronous && !subscriber->HasData())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (subscriber->HasData()) {
            message_received = true;

            // Get a buffer pointer from the raw bytes of the flatbuffer message
            auto data = subscriber->GetData();
            auto buffer = flatbuffers::GetSizePrefixedRoot<SynFlatBuffers::Buffer>(data.data());
            ProcessMessageBuffer(buffer);
            subscriber->PopData();
        }
    }

    return message_received;
}

void SynDDSAgent::ProcessMessageBuffer(const SynFlatBuffers::Buffer* buffer) {
    if (!m_initialized && m_zombie) {
        auto agent = AgentFromDescription(buffer);
        if (agent)
            m_agent = agent;
        return;
    }

    for (auto message : (*buffer->buffer())) {
        SynMessage* msg = SynMessageFactory::GenerateMessage(message);
        if (!msg)
            continue;

        if (m_zombie) {
            if (!m_initialized)
                continue;
            m_messages.push_back(msg);
        } else {
            int rank = msg->GetRank();

            // Ignore message if sent by this rank or this rank doesn't have an agent
            if (m_rank == rank)
                continue;

            SynchronizeZombie(msg);

            // Clean up
            delete msg;
        }
    }
}

std::shared_ptr<SynAgent> SynDDSAgent::AgentFromDescription(const SynFlatBuffers::Buffer* buffer) {
    std::shared_ptr<SynAgent> agent = nullptr;

    for (auto message : (*buffer->buffer())) {
        SynMessage* msg = SynMessageFactory::GenerateMessage(message);
        if (!msg)
            return agent;

        // TODO: Safer cast
        auto agent_msg = (SynAgentMessage*)msg;

        // Create and add the agent
        agent = SynAgentFactory::CreateAgent(agent_msg);

        // Clean up
        delete msg;

        // Initialize the underlying zombie of the agent
        if (m_zombie)
            agent->InitializeZombie(m_system);

        return agent;
    }

    return agent;
}

}  // namespace synchrono
}  // namespace chrono
