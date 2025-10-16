#include "chrono_synchrono/SynChronoManager.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/agent/SynAgentFactory.h"
#include "chrono_synchrono/flatbuffer/message/SynSimulationMessage.h"

#ifdef CHRONO_SYNCHRONO_USE_FASTDDS
    #undef ALIVE

    #include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
    #include "chrono_synchrono/communication/dds/SynDDSListener.h"
    #include "chrono_synchrono/communication/dds/idl/SynDDSMessage.h"
    #include "chrono_synchrono/communication/dds/idl/SynDDSMessagePubSubTypes.h"
    #include "chrono_synchrono/communication/dds/SynDDSTopic.h"
#endif

namespace chrono {
namespace synchrono {

#ifdef CHRONO_SYNCHRONO_USE_FASTDDS

void ProcessMessage(std::shared_ptr<SynCommunicator> communicator, void* message) {
    communicator->ProcessBuffer(((SynDDSMessage*)message)->data());
}

void RegisterParticipant(std::shared_ptr<SynCommunicator> communicator, const std::string& participant_name) {
    if (auto dds_communicator = std::dynamic_pointer_cast<SynDDSCommunicator>(communicator)) {
        std::string prefix = dds_communicator->m_prefix;

        std::size_t found = participant_name.find(prefix);
        if (found == std::string::npos)
            return;  // prefix not found

        SynLog() << "Adding Participant: " << participant_name << "\n";

        auto callback = std::bind(&ProcessMessage, communicator, std::placeholders::_1);
        dds_communicator->CreateSubscriber(SynDDSTopic::RemovePrefix(participant_name, dds_communicator->m_prefix),
                                           new SynDDSMessagePubSubType(), callback, new SynDDSMessage(), true, true);
    }
}

#endif

SynChronoManager::SynChronoManager(int node_id, int num_nodes, std::shared_ptr<SynCommunicator> communicator)
    : m_is_ok(true),
      m_initialized(false),
      m_node_id(node_id),
      m_num_nodes(num_nodes),
      m_node_key(node_id, 0),
      m_heartbeat(1e-2),
      m_next_sync(0.0),
      m_time_update(0),
      m_time_msg_gather(0),
      m_time_communication(0),
      m_time_msg_process(0) {
    if (communicator)
        SetCommunicator(communicator);

    // Set logger for logging prefix
    SetLogNodeID(node_id);
}

SynChronoManager::~SynChronoManager() {}

// Set the agent at the specified node
bool SynChronoManager::AddAgent(std::shared_ptr<SynAgent> agent) {
    // Because it is assumed a handshake is done when the Initialization function is called,
    // it is not allowed to add a new agent after this process to ensure each node/agent knows
    // about each other
    if (m_initialized) {
        SynLog() << "WARNING: SynChronoManager has been initialized. All agents should be added prior to "
                    "initializing the manager. Ignoring this new agent.\n";
        return false;
    }

    int new_agent_num = ++m_num_managed_agents;

    // concatenate the node id and the agent id
    AgentKey agent_key = AgentKey(m_node_id, new_agent_num);
    m_agents[agent_key] = agent;

    agent->SetKey(agent_key);

#ifdef CHRONO_SYNCHRONO_USE_FASTDDS
    if (auto dds_communicator = std::dynamic_pointer_cast<SynDDSCommunicator>(m_communicator)) {
        // Create the topic that state information will be passed on
        // and add the topic to the communicator
        auto topic_name = m_node_key.GetKeyString();
        dds_communicator->CreatePublisher(topic_name, new SynDDSMessagePubSubType(), true);
    }
#endif

    return true;
}

bool SynChronoManager::AddZombie(std::shared_ptr<SynAgent> zombie, AgentKey agent_key) {
    // Because it is assumed a handshake is done when the Initialization function is called,
    // it is not allowed to add a new zombie after this process to ensure each node/agent knows
    // about each other
    if (m_initialized) {
        SynLog() << "WARNING: SynChronoManager has been initialized. All zombies should be added prior to "
                    "initializing the manager. Ignoring this new zombie.\n";
        return false;
    }

    m_zombies[agent_key] = zombie;
    zombie->SetKey(agent_key);

    return true;
}

bool SynChronoManager::SetCommunicator(std::shared_ptr<SynCommunicator> communicator) {
    // Because it is assumed a handshake is done when the Initialization function is called,
    // it is not allowed to set the communicator after this process to ensure each node/agent knows
    // about each other
    if (m_initialized) {
        SynLog() << "WARNING: SynChronoManager has been initialized. The communicator should be set prior to "
                    "initializing the manager. Ignoring the communicator.\n";
        return false;
    }

    m_communicator = communicator;

    return true;
}

bool SynChronoManager::Initialize(ChSystem* system) {
    if (!m_communicator) {
        SynLog() << "WARNING: A Communicator has not been attached.\n";
        return false;
    }

    // Initialize the communicator
    m_communicator->Initialize();

#ifdef CHRONO_SYNCHRONO_USE_FASTDDS
    // If the communicator uses DDS, we want to create subscribers that will listen to state information
    // coming from the other nodes. This is done by setting the name of each governing participant to
    // common names to be parsed. RegisterParticipant will parse these names and create Subscribers
    // listening to incoming state data.
    if (auto dds_communicator = std::dynamic_pointer_cast<SynDDSCommunicator>(m_communicator)) {
        dds_communicator->Barrier(m_num_nodes - 1);

        for (const std::string& participant_name : dds_communicator->GetMatchedParticipantNames())
            RegisterParticipant(m_communicator, participant_name);
    }
#endif

    // Gather all of the underlying messages and add those to the communicator
    auto descriptions = GatherDescriptionMessages();
    m_communicator->AddOutgoingMessages(descriptions);

    // Send the description messages out to each node and receive any other messages
    m_communicator->Synchronize();

    // Process any received data
    // If first pass, data will typically contain description messages that describe new agents
    // Otherwise, will contain state or general purpose messages
    ProcessReceivedMessages();

    // Create agents from received descriptions
    CreateAgentsFromDescriptions();

    // Initialize each zombie with the passed system
    for (auto& zombie_pair : m_zombies)
        zombie_pair.second->InitializeZombie(system);

    m_initialized = true;

    return true;
}

void SynChronoManager::Synchronize(double time) {
    if (!m_communicator)
        return;

    // If time to next sync is in the future, do nothing
    if (time < m_next_sync)
        return;

    // Reset timers
    m_timer_update.reset();
    m_timer_msg_gather.reset();
    m_timer_communication.reset();
    m_timer_msg_process.reset();

    // Call update for each underlying agent
    m_timer_update.start();
    UpdateAgents();
    m_timer_update.stop();

    // Gather messages from each node and add those to the communicator
    // Only add the messages to the communicator which is responsible for commuticating with that node
    m_timer_msg_gather.start();
    SynMessageList messages = GatherMessages();
    m_communicator->AddOutgoingMessages(messages);
    m_timer_msg_gather.stop();

    // Send the messages out to each node and receive any other messages
    m_timer_communication.start();
    m_communicator->Synchronize();
    m_timer_communication.stop();

    // Process any received data
    // Will most likely contain state or general purpose messages
    // Distribute the organized messages
    m_timer_msg_process.start();
    ProcessReceivedMessages();
    DistributeMessages();
    m_timer_msg_process.stop();

    // Accumulate timers
    m_time_update += m_timer_update();
    m_time_msg_gather += m_timer_msg_gather();
    m_time_communication += m_timer_communication();
    m_time_msg_process += m_timer_msg_process();

    // Reset
    m_communicator->Reset();     // Reset the communicator
    m_messages.clear();          // clean the message map
    m_next_sync += m_heartbeat;  // Set next sync to a point in the future
}

void SynChronoManager::UpdateAgents() {
    for (auto& agent_pair : m_agents)
        agent_pair.second->Update();
}

void SynChronoManager::QuitSimulation() {
    if (m_is_ok) {
        m_communicator->AddQuitMessage();
        m_communicator->Synchronize();
        m_is_ok = false;
    }
}

void SynChronoManager::PrintStepStatistics(std::ostream& os) const {
    os << " Timers (ms [s]):" << std::endl;
    os << "   Agent update:    " << 1e3 * m_timer_update() << "  [" << m_time_update << "]" << std::endl;
    os << "   Msg. generation: " << 1e3 * m_timer_msg_gather() << "  [" << m_time_msg_gather << "]" << std::endl;
    os << "   Communication:   " << 1e3 * m_timer_communication() << "  [" << m_time_communication << "]" << std::endl;
    os << "   Msg. processing: " << 1e3 * m_timer_msg_process() << "  [" << m_time_msg_process << "]" << std::endl;
}

// --------------------------------------------------------------------------------------------------------------

SynMessageList SynChronoManager::GatherMessages() {
    SynMessageList messages;

    // Gather messages from each agent on this node
    for (auto& agent_pair : m_agents)
        agent_pair.second->GatherMessages(messages);

    return messages;
}

SynMessageList SynChronoManager::GatherDescriptionMessages() {
    SynMessageList messages;

    // Gather the description messages from each agent on this node
    for (auto& agent_pair : m_agents)
        agent_pair.second->GatherDescriptionMessages(messages);

    return messages;
}

void SynChronoManager::ProcessReceivedMessages() {
    // get the message buffer from the underlying communicator
    SynMessageList messages = m_communicator->GetMessages();

    for (auto& message : messages) {
        if (message->GetMessageType() == SynFlatBuffers::Type_Simulation_State) {
            auto sim_msg = std::dynamic_pointer_cast<SynSimulationMessage>(message);
            m_is_ok = !(sim_msg->m_quit_sim);
        } else {
            for (const auto& agent_pair : m_agents)
                m_messages[agent_pair.second].push_back(message);
        }
    }
}

void SynChronoManager::DistributeMessages() {
    for (auto& message_agent_pair : m_messages) {
        // For readibility
        auto to_agent = message_agent_pair.first;
        auto messages = message_agent_pair.second;

        for (auto message : messages) {
            auto from_zombie = m_zombies[message->GetSourceKey()];

            // std::map will return a nullptr if zombie does not exist (i.e. has never been added)
            if (!from_zombie) {
                std::string print_str("The intended agent (" + message->GetSourceKey().GetKeyString() +
                                      ") is not on this node!\n");
                SynLog() << print_str;
                continue;
            }

            from_zombie->SynchronizeZombie(message);
            to_agent->ProcessMessage(message);
        }
    }
}

void SynChronoManager::CreateAgentsFromDescriptions() {
    for (auto& message_agent_pair : m_messages) {
        // For readibility
        auto to_agent = message_agent_pair.first;
        auto messages = message_agent_pair.second;

        // Will remove the message if used
        for (SynMessageList::iterator it = messages.begin(); it != messages.end();) {
            // Try to create a zombie from the passed message
            auto message = *it;
            try {
                /// TODO: In the future, this shouldn't be necessary since destination and source will be used
                if (auto zombie = m_zombies[message->GetSourceKey()]) {
                    to_agent->RegisterZombie(zombie);
                } else {
                    zombie = SynAgentFactory::CreateAgent(message);
                    AddZombie(zombie, message->GetSourceKey());
                    to_agent->RegisterZombie(zombie);

                    // SynLog() << "Added agent with ID " << message->GetSourceKey() << "\n";
                }
                it = messages.erase(it);

            } catch (const std::exception&) {
                ++it;
            }
        }
    }
}

}  // namespace synchrono
}  // namespace chrono
