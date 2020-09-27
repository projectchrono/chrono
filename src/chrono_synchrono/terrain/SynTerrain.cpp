#include "chrono_synchrono/terrain/SynTerrain.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

Document SynTerrain::ParseTerrainFileJSON(const std::string& filename) {
    // Open and parse the input file
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        throw ChException("Vehicle file not read properly in ParseTerrainFileJSON.");

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));

    std::string type = d["Type"].GetString();
    assert(type.compare("Terrain") == 0);

    return d;
}

}  // namespace synchrono
}  // namespace chrono
