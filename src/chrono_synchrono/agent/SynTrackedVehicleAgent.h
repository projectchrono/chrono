#ifndef SYN_TRACKED_VEHICLE_AGENT_H
#define SYN_TRACKED_VEHICLE_AGENT_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynVehicleAgent.h"

#include "chrono_synchrono/vehicle/SynTrackedVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

class SYN_API SynTrackedVehicleAgent : public SynVehicleAgent {
  public:
    /// Construct a vehicle agent with the specified rank and system
    /// Underlying agent is set to a vehicle type automatically
    SynTrackedVehicleAgent(unsigned int rank, ChSystem* system = 0);

    /// Construct a vehicle agent from the specified json configuration and the ChSystem
    SynTrackedVehicleAgent(unsigned int rank, const std::string& filename, ChSystem* system);

    /// Construct a vehicle agent from the specified json configuration and contact method
    SynTrackedVehicleAgent(unsigned int rank, const std::string& filename, ChContactMethod contact_method);

    /// Construct a zombie vehicle agent from the specified json configuration and the ChSystem
    SynTrackedVehicleAgent(unsigned int rank, const std::string& filename);

    /// Destructor
    virtual ~SynTrackedVehicleAgent() {}

    // Update the state of this vehicle at the current time.
    virtual void Synchronize(double step, ChDriver::Inputs driver_inputs) override;

    /// Get agent state
    virtual std::shared_ptr<SynMessageState> GetState() override;
    virtual std::shared_ptr<SynAgentMessage> GetMessage() override { return m_msg; }

    /// Get this agent's vehicle
    virtual std::shared_ptr<SynVehicle> GetVehicle() override { return m_tracked_vehicle; }
    std::shared_ptr<SynTrackedVehicle> GetTrackedVehicle() { return m_tracked_vehicle; }

    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) override;

    void SetVehicle(std::shared_ptr<SynTrackedVehicle> tracked_vehicle);

  private:
    // Construct a zombie VehicleAgent
    void VehicleAgentFromJSON(const std::string& filename);

    // Construct a VehicleAgent with a ChSystem
    void VehicleAgentFromJSON(const std::string& filename, ChSystem* system);

    // Construct a VehicleAgent with a ChContactMethod
    void VehicleAgentFromJSON(const std::string& filename, ChContactMethod contact_method);

  private:
    std::shared_ptr<SynTrackedVehicle> m_tracked_vehicle;  ///< handle to this agent's vehicle

    std::shared_ptr<SynTrackedVehicleMessage> m_msg;
};

}  // namespace synchrono
}  // namespace chrono

#endif
