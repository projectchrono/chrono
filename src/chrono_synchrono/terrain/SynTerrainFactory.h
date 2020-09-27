#ifndef SYN_TERRAIN_FACTORY_H
#define SYN_TERRAIN_FACTORY_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/terrain/SynTerrain.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"
#include "chrono_synchrono/terrain/SynSCMTerrain.h"

namespace chrono {
namespace synchrono {

/// Generates SynTerrain's from JSON files
/// Used to improve generality in Agent classes
class SYN_API SynTerrainFactory {
  public:
    /// Generate the corresponding SynTerrain from a JSON specification file
    static std::shared_ptr<SynTerrain> CreateTerrain(ChSystem* system, const std::string& filename);
};

}  // namespace synchrono
}  // namespace chrono

#endif
