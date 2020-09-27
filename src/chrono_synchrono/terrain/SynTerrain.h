#ifndef SYN_TERRAIN_H
#define SYN_TERRAIN_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/flatbuffer/SynFlatBuffersManager.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

class SYN_API SynTerrain {
  public:
    // Constructor
    SynTerrain() {}

    // Destructor
    ~SynTerrain() {}

    /// Processes the incoming message
    virtual void ProcessMessage(SynMessage* message) = 0;

    /// Generate SynMessage to send
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages, int rank) = 0;

    /// Get the terrain
    virtual std::shared_ptr<ChTerrain> GetTerrain() = 0;

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    /// Update the state of the terrain system at the specified time.
    virtual void Advance(double step) { GetTerrain()->Advance(step); }

    /// Advance the state of the terrain system by the specified duration.
    virtual void Synchronize(double time) { GetTerrain()->Synchronize(time); }

    /// Get the terrain height below the specified location.
    double GetHeight(const ChVector<>& loc) { return GetTerrain()->GetHeight(loc); }

    static rapidjson::Document ParseTerrainFileJSON(const std::string& filename);
};

}  // namespace synchrono
}  // namespace chrono

#endif  // SYNTERRAIN_H
