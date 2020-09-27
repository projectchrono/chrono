#ifndef SYN_WHEELED_VEHICLE_AGENT_H
#define SYN_WHEELED_VEHICLE_AGENT_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynVehicleAgent.h"

#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

class SYN_API SynWheeledVehicleAgent : public SynVehicleAgent {
  public:
    /// Construct a vehicle agent with the specified rank and system
    /// Underlying agent is set to a vehicle type automatically
    SynWheeledVehicleAgent(unsigned int rank, ChSystem* system = 0);

    /// Construct a vehicle agent from the specified json configuration and the ChSystem
    SynWheeledVehicleAgent(unsigned int rank, const std::string& filename, ChSystem* system);

    /// Construct a vehicle agent from the specified json configuration and contact method
    SynWheeledVehicleAgent(unsigned int rank, const std::string& filename, ChContactMethod contact_method);

    /// Construct a zombie vehicle agent from the specified json configuration and the ChSystem
    SynWheeledVehicleAgent(unsigned int rank, const std::string& filename);

    /// Destructor
    virtual ~SynWheeledVehicleAgent() {}

    // Update the state of this vehicle at the current time.
    virtual void Synchronize(double step, ChDriver::Inputs driver_inputs) override;

    /// Get agent state
    virtual std::shared_ptr<SynMessageState> GetState() override;
    virtual std::shared_ptr<SynAgentMessage> GetMessage() override { return m_msg; }

    /// Get this agent's vehicle
    virtual std::shared_ptr<SynVehicle> GetVehicle() override { return m_wheeled_vehicle; }
    std::shared_ptr<SynWheeledVehicle> GetWheeledVehicle() { return m_wheeled_vehicle; }

    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) override;

    void SetVehicle(std::shared_ptr<SynWheeledVehicle> wheeled_vehicle);

  private:
    // Construct a zombie VehicleAgent
    void VehicleAgentFromJSON(const std::string& filename);

    // Construct a VehicleAgent with a ChSystem
    void VehicleAgentFromJSON(const std::string& filename, ChSystem* system);

    // Construct a VehicleAgent with a ChContactMethod
    void VehicleAgentFromJSON(const std::string& filename, ChContactMethod contact_method);

    std::shared_ptr<SynWheeledVehicle> m_wheeled_vehicle;  ///< handle to this agent's vehicle

    std::shared_ptr<SynWheeledVehicleMessage> m_msg;
};

}  // namespace synchrono
}  // namespace chrono

#endif
