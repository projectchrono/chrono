#ifndef SYN_VEHICLE_H
#define SYN_VEHICLE_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/simulation/SynSimulationConfig.h"

#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"

#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChDriver.h"

#include "chrono/assets/ChTriangleMeshShape.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;

namespace chrono {
namespace synchrono {

// ALWAYS owns system
class SYN_API SynVehicle {
  public:
    // Default constructor
    SynVehicle(bool is_zombie = true);

    // Constructor for vehicles specified through json
    // Vehicle will create a system itself
    SynVehicle(const std::string& filename, ChContactMethod contact_method);

    // Constructor for vehicles specified through json
    // Vehicle will use the passed system
    SynVehicle(const std::string& filename, ChSystem* system);

    // Constructor for zombie vehicles specified through json
    SynVehicle(const std::string& filename);

    // Destructor
    virtual ~SynVehicle() {
        if (m_owns_vehicle && m_system)
            delete m_system;
    }

    // Initialize the underlying vehicle
    virtual void Initialize(ChCoordsys<> coord_sys) = 0;

    // Initialize the zombie vehicle
    virtual void InitializeZombie(ChSystem* system) = 0;

    // Synchronize zombie with other ranks
    virtual void SynchronizeZombie(SynMessage* message) = 0;

    // Update the current state of this vehicle
    virtual void Update() = 0;

    // ------------------------------------------------------------------------

    // Is vehicle a zombie
    bool IsZombie() { return m_is_zombie; }

    // ------------------------------------------------------------------------

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    // Advance the state of this vehicle agent until agent time syncs with passed time.
    virtual void Advance(double step) {
        GetVehicle().Advance(step);
        m_system->DoStepDynamics(step);  // Will ALWAYS own system
    }

    // Get the underlying vehicle
    virtual ChVehicle& GetVehicle() = 0;

    // Get the ChSystem
    ChSystem* GetSystem() { return m_system; }

  protected:
    virtual void ParseVehicleFileJSON(const std::string& filename);

    virtual void CreateVehicle(const std::string& filename, ChSystem* system) = 0;

    virtual void CreateZombie(const std::string& filename) = 0;

    std::shared_ptr<ChTriangleMeshShape> CreateMeshZombieComponent(const std::string& filename);

    void CreateChassisZombieBody(const std::string& filename, ChSystem* system);

  protected:
    bool m_owns_vehicle;  ///< has the vehicle been created by this object or passed by user
    bool m_is_json;       ///< has this object been constructed through a json specification file
    bool m_is_zombie;     ///< is this vehicle a zombie

    ChSystem* m_system;  ///< pointer to the chrono system

    std::shared_ptr<ChBodyAuxRef> m_zombie_body;  ///< agent's zombie body reference

    rapidjson::Document d;
};

}  // namespace synchrono
}  // namespace chrono
#endif
