#ifndef SYN_RIGID_TERRAIN_H
#define SYN_RIGID_TERRAIN_H

#include "chrono_synchrono/terrain/SynTerrain.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"

namespace chrono {
namespace synchrono {

class SYN_API SynRigidTerrain : public SynTerrain {
  public:
    // Default Constructor
    SynRigidTerrain(std::shared_ptr<RigidTerrain> terrain = nullptr) { SetTerrain(terrain); }

    // Construct the underlying rigid terrain from the specified JSON file
    SynRigidTerrain(ChSystem* system, const std::string& filename);

    // Destructor
    ~SynRigidTerrain() {}

    /// Processes the incoming message
    virtual void ProcessMessage(SynMessage* message) override {}

    /// Generate SynMessage to send
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages, int rank) override {}

    /// Set the terrain
    void SetTerrain(std::shared_ptr<RigidTerrain> terrain) { m_rigid_terrain = terrain; }

    /// Get the terrain
    virtual std::shared_ptr<ChTerrain> GetTerrain() override { return m_rigid_terrain; }
    std::shared_ptr<RigidTerrain> GetRigidTerrain() { return m_rigid_terrain; }

  private:
    void AddVisualizationAssetsJSON(const rapidjson::Value& a);

    std::shared_ptr<RigidTerrain> m_rigid_terrain;
};

}  // namespace synchrono
}  // namespace chrono

#endif
