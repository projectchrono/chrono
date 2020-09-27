#ifndef SYN_VEHICLE_BRAIN_H
#define SYN_VEHICLE_BRAIN_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/brain/SynBrain.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/vehicle/SynVehicle.h"
#include "chrono_vehicle/ChDriver.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

class SYN_API SynVehicleBrain : public SynBrain {
  public:
    /// Constructor which takes and sets this brains ChDriver and ChVehicle
    SynVehicleBrain(int rank, std::shared_ptr<ChDriver> driver, ChVehicle& vehicle)
        : SynBrain(rank), m_driver(driver), m_vehicle(vehicle) {}

    /// Destructor
    ~SynVehicleBrain() {}

    /// Advance the state of this brain until brain time syncs with passed time
    virtual void Advance(double step) override;

    /// Synchronize this brain to the specified time
    virtual void Synchronize(double time) override;

    virtual void ProcessMessage(SynMessage* msg) override{};
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) override{};

    // --------------------------------------------------------------------------------------------------------------

    /// Get the driver inputs from the attached driver
    ChDriver::Inputs GetDriverInputs() { return m_driver->GetInputs(); }

    /// Get the attached driver
    std::shared_ptr<ChDriver> GetDriver() { return m_driver; }

    /// Set the attached driver
    void SetDriver(std::shared_ptr<ChDriver> driver) { m_driver = driver; }

    ChVehicle& GetVehicle() { return m_vehicle; }

  protected:
    std::shared_ptr<ChDriver> m_driver;  ///< handle to the ChDriver
    ChVehicle& m_vehicle;
};

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_VEHICLE_BRAIN_H
