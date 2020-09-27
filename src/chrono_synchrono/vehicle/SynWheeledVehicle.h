#ifndef SYN_WHEELED_VEHICLE_H
#define SYN_WHEELED_VEHICLE_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/vehicle/SynVehicle.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

class SYN_API SynWheeledVehicle : public SynVehicle {
  public:
    // Constructor for zombie vehicle
    SynWheeledVehicle();

    // Constructor for vehicle
    SynWheeledVehicle(ChWheeledVehicle* vehicle);

    // Constructor for vehicles specified through json
    // Vehicle will create a system itself
    SynWheeledVehicle(const std::string& filename, ChContactMethod contact_method);

    // Constructor for vehicles specified through json
    // Vehicle will use the passed system
    SynWheeledVehicle(const std::string& filename, ChSystem* system);

    // Constructor for zombie vehicles specified through json
    SynWheeledVehicle(const std::string& filename);

    // Destructor
    virtual ~SynWheeledVehicle() {
        if (m_wheeled_vehicle && m_owns_vehicle) {
            delete m_wheeled_vehicle;

            if (m_system)
                delete m_system;
        }
    }

    // Initialize the underlying vehicle
    virtual void Initialize(ChCoordsys<> coord_sys) override;

    // Initialize the zombie vehicle
    virtual void InitializeZombie(ChSystem* system) override;

    // Synchronize zombie with other ranks
    virtual void SynchronizeZombie(SynMessage* message) override;

    // Update the current state of this vehicle
    virtual void Update() override;

    // ------------------------------------------------------------------------

    // Setters for the visualization files
    void SetZombieVisualizationFiles(std::string chassis_vis_file,
                                     std::string wheel_vis_file,
                                     std::string tire_vis_file) {
        m_description->m_chassis_vis_file = chassis_vis_file;
        m_description->m_wheel_vis_file = wheel_vis_file;
        m_description->m_tire_vis_file = tire_vis_file;
    }

    // Set the number of wheels this vehicle has
    void SetNumWheels(int num_wheels) { m_description->m_num_wheels = num_wheels; }

    // ------------------------------------------------------------------------

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    // Update the state of this vehicle at the current time.
    void Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain) {
        m_wheeled_vehicle->Synchronize(time, driver_inputs, terrain);
    }

    // Get the underlying vehicle
    virtual ChVehicle& GetVehicle() override { return *m_wheeled_vehicle; }

  protected:
    virtual void ParseVehicleFileJSON(const std::string& filename) override;

    virtual void CreateVehicle(const std::string& filename, ChSystem* system) override;

    virtual void CreateZombie(const std::string& filename) override;

  private:
    ChWheeledVehicle* m_wheeled_vehicle;

    std::shared_ptr<SynWheeledVehicleState> m_state;
    std::shared_ptr<SynWheeledVehicleDescription> m_description;

    std::vector<std::shared_ptr<ChBodyAuxRef>> m_wheel_list;  ///< vector of this agent's zombie wheels

    friend class SynWheeledVehicleAgent;
};

// A class which wraps a vehicle model that contains a ChWheeledVehicle
template <class V>
class SYN_API SynCustomWheeledVehicle : public SynWheeledVehicle {
  public:
    // Constructor for non-zombie vehicle
    SynCustomWheeledVehicle(std::shared_ptr<V> vehicle_model) : SynWheeledVehicle(&vehicle_model->GetVehicle()) {
        m_vehicle_model = vehicle_model;
        m_system = vehicle_model->GetSystem();
    }

  private:
    std::shared_ptr<V> m_vehicle_model;
};

}  // namespace synchrono
}  // namespace chrono

#endif