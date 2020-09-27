#include "chrono_synchrono/terrain/SynTerrainFactory.h"

#include "chrono_synchrono/utils/SynUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

std::shared_ptr<SynTerrain> SynTerrainFactory::CreateTerrain(ChSystem* system, const std::string& filename) {
    // Create the terrain
    std::shared_ptr<SynTerrain> terrain;

    // Parse JSON file to get the terrain type
    Document d = SynTerrain::ParseTerrainFileJSON(filename);
    std::string type = d["Template"].GetString();

    if (type.compare("RigidTerrain") == 0) {
        terrain = chrono_types::make_shared<SynRigidTerrain>(system, filename);
    } else if (type.compare("SCMDeformableTerrain") == 0) {
        terrain = chrono_types::make_shared<SynSCMTerrain>(system, filename);
    } else {
        std::string message = "Terrain type \"" + type + "\" not recognized.";
        throw ChException(message);
    }

    return terrain;
}

}  // namespace synchrono
}  // namespace chrono
