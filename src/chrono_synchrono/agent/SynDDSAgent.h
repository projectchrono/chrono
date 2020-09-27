#ifndef SYN_DDS_AGENT_H
#define SYN_DDS_AGENT_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynAgent.h"

#include "chrono_synchrono/flatbuffer/SynFlatBuffersManager.h"

namespace chrono {
namespace synchrono {

// Forward declaration
class SynDDSParticipant;
class SynDDSPublisher;

/// TODO Allow for more than 1 subscriber and 1 publisher
class SYN_API SynDDSAgent : public SynAgent {
  public:
    /// Construct a dds manager object
    SynDDSAgent(std::shared_ptr<SynAgent> agent = nullptr, bool zombie = false);

    /// Destructor
    virtual ~SynDDSAgent();

    /// @brief Initialize the DDS portion of this agent
    bool Initialize();

    /// Advance the state of this agent until agent time syncs with passed time.
    virtual void Advance(double time_of_next_sync) override;

    /// Initialize this agents zombie representation.
    virtual void InitializeZombie(ChSystem* system = 0) override { m_agent->InitializeZombie(system); }

    /// Synchronoize this agents zombie with the rest of the simulation.
    /// Updates agent based on specified message.
    virtual void SynchronizeZombie(SynMessage* message) override {
        m_flatbuffers_manager.AddMessage(message);
    }

    /// Get agent state
    virtual std::shared_ptr<SynMessageState> GetState() override { return m_agent->GetState(); }

    /// Get agent message
    virtual std::shared_ptr<SynAgentMessage> GetMessage() override { return m_agent->GetMessage(); }

    /// Process incoming message. Forwards message to underlying agent brain.
    virtual void ProcessMessage(SynMessage* msg) override {}

    /// Generate vector of SynMessages to send
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) override;

    bool IsOk() { return m_ok; }

  private:
    /// @brief Publish relevant data
    void Publish();

    /// @brief Listen for data
    bool Listen(bool synchronous = false);

    /// @brief Processes all messages inside a single buffer. Add agents from their description messages
    ///
    /// @param buffer A buffer that contains messages received from a particular rank. e.g. state data + other messages
    virtual std::shared_ptr<SynAgent> AgentFromDescription(const SynFlatBuffers::Buffer* buffer) final;

    /// @brief Processes all messages inside a single buffer. Updates zombie agents and allows agents additional
    /// processing.
    ///
    /// @param buffer A buffer that contains messages received from a particular rank. e.g. state data + other messages
    virtual void ProcessMessageBuffer(const SynFlatBuffers::Buffer* buffer) final;

  private:
    bool m_zombie;

    SynDDSParticipant* m_participant;  ///< FastDDS DomainParticipant wrapped for use in SynChrono
    SynDDSPublisher* m_publisher;      ///< FastDDS Publisher responsible for sending rank state

    bool m_initialized;  ///< has the manager been initialized
    bool m_ok;           ///< Is the underlying DDS interface still alive

    SynFlatBuffersManager m_flatbuffers_manager;  ///< flatbuffer manager for this rank

    std::vector<SynMessage*> m_messages;  ///< Messages to send interfaced rank

    std::shared_ptr<SynAgent> m_agent;
};

}  // namespace synchrono
}  // namespace chrono

#endif