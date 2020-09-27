#ifndef SYN_JSON_UTILS_H
#define SYN_JSON_UTILS_H

#include "chrono_synchrono/SynApi.h"

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChCoordsys.h"

#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_thirdparty/rapidjson/document.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

// -----------------------------------------------------------------------------

/// Load and return a RapidJSON document from the specified file.
/// A Null document is returned if the file cannot be opened.
SYN_API rapidjson::Document ReadFileJSON(const std::string& filename);

// -----------------------------------------------------------------------------

/// Load and return a ChVector from the specified JSON array
SYN_API ChVector<> ReadVectorJSON(const rapidjson::Value& a);

/// Load and return a ChQuaternion from the specified JSON array
SYN_API ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a);

/// Load and return a ChCoordsys from the specified JSON values
SYN_API ChCoordsys<> ReadCoordsysJSON(const rapidjson::Value& a, const rapidjson::Value& b);

// -----------------------------------------------------------------------------

///  Load and return the visualization type from the specified JSON file.
SYN_API VisualizationType ReadVisualizationTypeJSON(const std::string& type);

// -----------------------------------------------------------------------------

// /// Change value in JSON file
// SYN_API void ChangeValueJSON(const std::string& filename,
//                              std::vector<std::string> entries,
//                              const rapidjson::Value new_value);

// -----------------------------------------------------------------------------

}  // namespace synchrono
}  // namespace chrono

#endif
