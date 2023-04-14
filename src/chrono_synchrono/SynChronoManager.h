#ifndef SYN_CHRONO_MANAGER
#define SYN_CHRONO_MANAGER

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/communication/SynCommunicator.h"

#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_core
/// @{

/// Base class responsible for handling agents and synchronizing states between nodes
class SYN_API SynChronoManager {
  public:
    /// Class constructor
    SynChronoManager(int node_id, int num_nodes, std::shared_ptr<SynCommunicator> communicator = nullptr);

    /// Class destructor
    ~SynChronoManager();

    /// Add an agent to the SynChrono manager
    /// Cannot be called after Initialize is called
    ///
    /// @param agent The agent that should be added to the SynChrono world
    bool AddAgent(std::shared_ptr<SynAgent> agent);

    /// Add a zombie to the SynChrono manager with the passed id
    /// Cannot be called after Initialize is called
    ///
    /// @param zombie The zombie that should be added to the SynChrono world
    /// @param agent_key The id of the added zombie
    bool AddZombie(std::shared_ptr<SynAgent> zombie, AgentKey agent_key);

    ///@brief Sets the underlying communicator
    ///
    ///@param communicator the communicator to use for internode communication
    bool SetCommunicator(std::shared_ptr<SynCommunicator> communicator);

    ///@brief Initialize all the added agents.
    /// Communicator must be added before calling this function.
    /// Locks the agent list so new agents cannot be added.
    ///
    /// @param system The chrono system with which the sensor manager is associated. Used for zombie initialization.
    bool Initialize(ChSystem* system);

    ///@brief Synchronize data across nodes. This method essentially calls
    /// the underlying communicator to perform the work. The communicator will
    /// send and block until all messages are received. The manager is then responsible
    /// for distributing messages to each agent.
    ///
    /// A manager is responsible for maintaining time and space coherence,
    /// so each node and it's agent should be at the same time within the simulation
    ///
    ///@param time timestamp to synchronize each node at
    void Synchronize(double time);

    /// @brief Update the underlying agents
    /// Agents typically will update their state messages
    ///
    void UpdateAgents();

    /// @brief If our simulation is still running, sends a quit message to everyone else to exit as well
    ///     Needed specifically for MPI where we don't wait for timeouts
    void QuitSimulation();

    // ------------------------------------------------------------------------

    ///@brief Get the agents list
    ///
    std::map<AgentKey, std::shared_ptr<SynAgent>>& GetAgents() { return m_agents; }

    ///@brief Set the heartbeat for the rate at which SynChrono synchronization
    /// occurs
    ///
    void SetHeartbeat(double heartbeat) { m_heartbeat = heartbeat; }

    /// @brief Should the simulation still be running?
    bool IsOk() { return m_is_ok; }

    /// @brief Print timing information (over last step and cumulative)
    void PrintStepStatistics(std::ostream& os) const;

  private:
    // These methods are only available to derived classes.
    // This decision was made to ensure agents are responsible for message generation,
    // not users. If users were allowed such a liberty, synchronization data would not
    // be agent specific, as in the receiving agent won't know where the information was sent.
    // A SynCommunicator should be used for such applications.

    /// @brief Gather all messages from the attached nodes
    /// This method will ask each agent for any messages to send and then update the underlying communicator
    /// with those new messages
    ///
    SynMessageList GatherMessages();

    /// @brief Gather all description messages from the attached nodes
    /// A description message essentially describes how a zombie agent should be visualized.
    /// A description message will contain visual assets and initial positions.
    /// Should be called either during the initialization/handshake phase or on the first pass of info
    ///
    SynMessageList GatherDescriptionMessages();

    ///@brief Process the messages that have just been received.
    /// Will parse through received buffer and organize messages to pass to correct agents.
    ///
    /// For each message in the received buffer
    /// 1. check the intended node/num
    /// 2. organize the messages into a nested map for each agent on each node
    /// 3. if desired, pass those messages to the intended node
    ///
    void ProcessReceivedMessages();

    ///@brief This method passes out each received message to it's intended agent
    ///

    ///@brief Distribute each received message to it's intended agent
    /// This message uses the sorted messages stored in m_messages, so call ProcessReceivedMessages
    /// prior to this method in order to properly organize the received messages
    ///
    void DistributeMessages();

    ///@brief Create a agents from description messages
    /// During the initialization phase, description messages of each agent in the simulation is passed.
    /// This method uses those descriptions to create new agents and store them in the proper location
    /// in the m_zombies class variable
    ///
    void CreateAgentsFromDescriptions();

    // --------------------------------------------------------------------------------------------------------------

    bool m_is_ok;
    bool m_initialized;       ///< Has the Initialize function been called?
    int m_node_id;            ///< The node id assigned to this manager (provided by user code)
    int m_num_nodes;          ///< The number of nodes in the SynChrono world (provided by user code)
    AgentKey m_node_key;

    double m_heartbeat;  ///< Rate at which synchronization between nodes occurs
    double m_next_sync;  ///< Time at which next synchronization between nodes should occur

    ChTimer m_timer_update;         ///< timer for agent updates
    ChTimer m_timer_msg_gather;     ///< timer for generating outgoing messages
    ChTimer m_timer_communication;  ///< timer for communication
    ChTimer m_timer_msg_process;    ///< timer for processing received messages

    double m_time_update;         ///< cumulative time for agent updates
    double m_time_msg_gather;     ///< cumulative time for generating outgoing messages
    double m_time_communication;  ///< cummulative time for communication
    double m_time_msg_process;    ///< cumulative time for processing received messages

    int m_num_managed_agents = 0;  ///< Number of agents managed by this node
    std::map<AgentKey, std::shared_ptr<SynAgent>> m_agents;          ///< Agents in the SynChrono world on this node
    std::map<AgentKey, std::shared_ptr<SynAgent>> m_zombies;         ///< Agents in the SynChrono world not on this node
    std::map<std::shared_ptr<SynAgent>, SynMessageList> m_messages;  ///< Messages associated with each agent

    std::shared_ptr<SynCommunicator> m_communicator;  ///< Underlying communicator used for inter-node comm
};

/// @} synchrono_core

}  // namespace synchrono
}  // namespace chrono

#endif
