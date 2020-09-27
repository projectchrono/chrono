#ifndef SYN_VEHICLE_AGENT_H
#define SYN_VEHICLE_AGENT_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynAgent.h"

#include "chrono_synchrono/terrain/SynTerrain.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/vehicle/SynVehicle.h"

#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

#include "chrono_synchrono/utils/SynUtilsJSON.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

class SYN_API SynVehicleAgent : public SynAgent {
  public:
    /// Construct a vehicle agent with the specified rank and system
    /// Underlying agent is set to a vehicle type automatically
    SynVehicleAgent(unsigned int rank, ChSystem* system = 0);

    /// Destructor
    virtual ~SynVehicleAgent() {}

    /// Initialze this agent
    /// Really only used to initialize the position of the underlying vehicle
    void Initialize(ChCoordsys<> coord_sys) { GetVehicle()->Initialize(coord_sys); }

    // Update the state of this vehicle at the current time.
    virtual void Synchronize(double time, ChDriver::Inputs driver_inputs) = 0;

    /// Initialize this agents zombie representation.
    /// Will use the passed system if agent system is null.
    virtual void InitializeZombie(ChSystem* system = 0) override { GetVehicle()->InitializeZombie(system); }

    /// Synchronoize this agents zombie with the rest of the simulation.
    /// Updates agent based on specified message.
    virtual void SynchronizeZombie(SynMessage* message) override { GetVehicle()->SynchronizeZombie(message); }

    /// Advance the state of this vehicle agent until agent time syncs with passed time.
    virtual void Advance(double time_of_next_sync) override;

    /// Process incoming message. Forwards message to underlying agent brain.
    virtual void ProcessMessage(SynMessage* msg) override;

    // --------------------------------------------------------------------------------------------------------------

    /// Set the terrain for this agent
    void SetTerrain(std::shared_ptr<SynTerrain> terrain) { m_terrain = terrain; }

    /// Set the underlying brain for this agent
    void SetBrain(std::shared_ptr<SynVehicleBrain> brain) { m_brain = brain; }

    /// Get this agent's vehicle
    virtual std::shared_ptr<SynVehicle> GetVehicle() = 0;

    /// Get handle to this agents terrain
    std::shared_ptr<SynTerrain> GetTerrain() { return m_terrain; }

    /// Get the underlying brain from this again
    std::shared_ptr<SynVehicleBrain> GetBrain() { return m_brain; }

    /// Get Chrono driver from brain
    std::shared_ptr<ChDriver> GetDriver() { return m_brain->GetDriver(); }

    /// Get the ChVehicle from the SynVehicle
    ChVehicle& GetChVehicle() { return GetVehicle()->GetVehicle(); }

  protected:
    // Parse a vehicle agent json specification file
    rapidjson::Document ParseVehicleAgentFileJSON(const std::string& filename);

    // Construct a VehicleAgent from the document
    // Will assume the SynVehicle has already been made
    void VehicleAgentFromJSON(rapidjson::Document& d);

    std::shared_ptr<SynTerrain> m_terrain;     ///< handle to this agent's terrain
    std::shared_ptr<SynVehicleBrain> m_brain;  ///< handle to this agent's brain
};

}  // namespace synchrono
}  // namespace chrono

#endif
