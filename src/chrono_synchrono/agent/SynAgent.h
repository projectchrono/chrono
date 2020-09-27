#ifndef SYN_AGENT_H
#define SYN_AGENT_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/simulation/SynSimulationConfig.h"

#include "chrono_synchrono/brain/SynBrain.h"
#include "chrono_synchrono/visualization/SynVisualizationManager.h"
#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"
#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono/physics/ChSystem.h"

using namespace chrono;

namespace chrono {
namespace synchrono {

/// Agent types
enum class SynAgentType { VEHICLE, ENVIRONMENT };

/// Base class for SynChrono agents.
class SYN_API SynAgent {
  public:
    /// Construct a agent with the specified rank and type
    SynAgent(unsigned int rank, SynAgentType type, ChSystem* system = nullptr)
        : m_rank(rank),
          m_type(type),
          m_system(system),
          m_vis_manager(chrono_types::make_shared<SynVisualizationManager>()) {}

    /// Destructor.
    virtual ~SynAgent() {}

    /// Advance the state of this agent until agent time syncs with passed time.
    virtual void Advance(double time_of_next_sync) = 0;

    /// Initialize this agents zombie representation.
    virtual void InitializeZombie(ChSystem* system = 0) = 0;

    /// Synchronoize this agents zombie with the rest of the simulation.
    /// Updates agent based on specified message.
    virtual void SynchronizeZombie(SynMessage* message) = 0;

    /// Get agent state
    virtual std::shared_ptr<SynMessageState> GetState() = 0;

    virtual std::shared_ptr<SynAgentMessage> GetMessage() = 0;

    /// Generate vector of SynMessages to send
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) = 0;

    // Attach a visualization to this agent.
    void AttachVisualizationManager(std::shared_ptr<SynVisualizationManager> vis_manager) {
        m_vis_manager = vis_manager;
    }

    /// Process incoming message. Forwards message to underlying agent brain.
    virtual void ProcessMessage(SynMessage* msg) { m_brain->ProcessMessage(msg); }

    /// Set this agent's brain
    void SetBrain(std::shared_ptr<SynBrain> brain) { m_brain = brain; }

    /// Set this agent's rank
    void SetRank(unsigned int rank) { m_rank = rank; }

    /// Get the handle to this agent's brain
    std::shared_ptr<SynBrain> GetBrain() { return m_brain; }

    /// Get the type of this agent
    SynAgentType GetType() const { return m_type; }

    /// Get this agent's rank
    unsigned int GetRank() { return m_rank; }

    /// Get the Chrono system associated with this agent
    ChSystem* GetSystem() { return m_system; }

    /// Set the Chrono system associated with this agent
    void SetSystem(ChSystem* system) { m_system = system; }

    // Parse an agent json specification file
    static rapidjson::Document ParseAgentFileJSON(const std::string& filename);

  protected:
    unsigned int m_rank;  ///< agent rank
    SynAgentType m_type;  ///< agent type

    std::shared_ptr<SynBrain> m_brain;                       ///< handle to this agent's brain
    std::shared_ptr<SynVisualizationManager> m_vis_manager;  ///< handle to this agent's visualization manager

    ChSystem* m_system;  ///< pointer to the Chrono system
};

typedef std::vector<std::shared_ptr<SynAgent>> SynAgentList;

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_AGENT_H
