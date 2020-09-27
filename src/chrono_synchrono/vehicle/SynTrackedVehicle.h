#ifndef SYN_TRACKED_VEHICLE_H
#define SYN_TRACKED_VEHICLE_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/vehicle/SynVehicle.h"
#include "chrono_synchrono/flatbuffer/message/SynTrackedVehicleMessage.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_models/vehicle/m113/M113.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

// ALWAYS owns system
class SYN_API SynTrackedVehicle : public SynVehicle {
  public:
    // Constructor for zombie vehicle
    SynTrackedVehicle();

    // Constructor for vehicle
    SynTrackedVehicle(ChTrackedVehicle* vehicle);

    // Constructor for vehicles specified through json
    // Vehicle will create a system itself
    SynTrackedVehicle(const std::string& filename, ChContactMethod contact_method);

    // Constructor for vehicles specified through json
    // Vehicle will use the passed system
    SynTrackedVehicle(const std::string& filename, ChSystem* system);

    // Constructor for zombie vehicles specified through json
    SynTrackedVehicle(const std::string& filename);

    // Destructor
    virtual ~SynTrackedVehicle() {
        if (m_tracked_vehicle && m_owns_vehicle) {
            delete m_tracked_vehicle;

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

    void SetZombieVisualizationFiles(std::string chassis_vis_file,
                                     std::string track_shoe_vis_file,
                                     std::string left_sprocket_vis_file,
                                     std::string right_sprocket_vis_file,
                                     std::string left_idler_vis_file,
                                     std::string right_idler_vis_file,
                                     std::string left_road_wheel_vis_file,
                                     std::string right_road_wheel_vis_file) {
        m_description->m_chassis_vis_file = chassis_vis_file;
        m_description->m_track_shoe_vis_file = track_shoe_vis_file;
        m_description->m_left_sprocket_vis_file = left_sprocket_vis_file;
        m_description->m_right_sprocket_vis_file = right_sprocket_vis_file;
        m_description->m_left_idler_vis_file = left_idler_vis_file;
        m_description->m_right_idler_vis_file = right_idler_vis_file;
        m_description->m_left_road_wheel_vis_file = left_road_wheel_vis_file;
        m_description->m_right_road_wheel_vis_file = right_road_wheel_vis_file;
    }

    void SetNumAssemblyComponents(int num_track_shoes, int num_sprockets, int num_idlers, int num_road_wheels) {
        m_description->m_num_track_shoes = num_track_shoes;
        m_description->m_num_sprockets = num_sprockets;
        m_description->m_num_idlers = num_idlers;
        m_description->m_num_road_wheels = num_road_wheels;
    }

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    // Update the state of this vehicle at the current time.
    void Synchronize(double time, const ChDriver::Inputs& driver_inputs) {
        m_tracked_vehicle->Synchronize(time, driver_inputs, m_shoe_forces_left, m_shoe_forces_right);
    }

    // Get the underlying vehicle
    virtual ChVehicle& GetVehicle() override { return *m_tracked_vehicle; }

  private:
    virtual void ParseVehicleFileJSON(const std::string& filename) override;

    virtual void CreateVehicle(const std::string& filename, ChSystem* system) override;

    virtual void CreateZombie(const std::string& filename) override;

    void AddMeshToVector(std::shared_ptr<ChTriangleMeshShape> trimesh,
                         std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                         ChSystem* system);

    void AddMeshToVector(std::shared_ptr<ChTriangleMeshShape> left,
                         std::shared_ptr<ChTriangleMeshShape> right,
                         std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                         ChSystem* system);

  protected:
    ChTrackedVehicle* m_tracked_vehicle;  ///< pointer to the chrono vehicle

    std::shared_ptr<SynTrackedVehicleState> m_state;
    std::shared_ptr<SynTrackedVehicleDescription> m_description;

    TerrainForces m_shoe_forces_left;
    TerrainForces m_shoe_forces_right;

    std::vector<std::shared_ptr<ChBodyAuxRef>> m_track_shoe_list;  ///< vector of this agent's zombie track shoes
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_sprocket_list;    ///< vector of this agent's zombie sprockets
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_idler_list;       ///< vector of this agent's zombie idlers
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_road_wheel_list;  ///< vector of this agent's zombie road wheels

    friend class SynTrackedVehicleAgent;
};

// A class which wraps a vehicle model that contains a ChWheeledVehicle
template <class V>
class SYN_API SynCustomTrackedVehicle : public SynTrackedVehicle {
  public:
    // Constructor for non-zombie vehicle
    SynCustomTrackedVehicle(std::shared_ptr<V> vehicle_model) : SynTrackedVehicle(&vehicle_model->GetVehicle()) {
        m_vehicle_model = vehicle_model;
        m_system = vehicle_model->GetSystem();
    }

  private:
    std::shared_ptr<V> m_vehicle_model;
};

}  // namespace synchrono
}  // namespace chrono
#endif
