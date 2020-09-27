#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

namespace chrono {
namespace synchrono {

void SetChronoDataPaths(const std::string& chrono_main, const std::string& chrono_vehicle) {
    SetChronoDataPath(chrono_main);
    vehicle::SetDataPath(chrono_vehicle);
}

// -----------------------------------------------------------------------------
// Functions for manipulating the SynChrono data directory
// -----------------------------------------------------------------------------

static std::string synchrono_data_path("../data/synchrono/");

// Set the path to the SynChrono data directory (ATTENTION: not thread safe)
void SetSynChronoDataPath(const std::string& path) {
    synchrono_data_path = path;
}

// Obtain the current path to the SynChrono data directory (thread safe)
const std::string& GetSynChronoDataPath() {
    return synchrono_data_path;
}

// Obtain the complete path to the specified filename, given relative to the
// SynChrono data directory (thread safe)
std::string GetSynDataFile(const std::string& filename) {
    return synchrono_data_path + filename;
}

}  // namespace synchrono
}  // namespace chrono
