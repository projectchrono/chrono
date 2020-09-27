#include <fstream>

#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono/core/ChLog.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace chrono;
using namespace rapidjson;

namespace chrono {
namespace synchrono {

// -----------------------------------------------------------------------------

Document ReadFileJSON(const std::string& filename) {
    Document d;
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        GetLog() << "ERROR: Could not open JSON file: " << filename << "\n";
    } else {
        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            GetLog() << "ERROR: Invalid JSON file: " << filename << "\n";
        }
    }
    return d;
}

// -----------------------------------------------------------------------------

ChVector<> ReadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

ChQuaternion<> ReadQuaternionJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

ChCoordsys<> ReadCoordsysJSON(const Value& a, const Value& b) {
    return ChCoordsys<>(ReadVectorJSON(a), ReadQuaternionJSON(b));
}

// -----------------------------------------------------------------------------

VisualizationType ReadVisualizationTypeJSON(const std::string& type) {
    // Determine visualization type.
    VisualizationType visualization_type;
    if (type.compare("None") == 0) {
        visualization_type = VisualizationType::NONE;
    } else if (type.compare("Primitives") == 0) {
        visualization_type = VisualizationType::PRIMITIVES;
    } else if (type.compare("Mesh") == 0) {
        visualization_type = VisualizationType::MESH;
    } else {
        throw ChException("Visualization type \"" + type + "\" is not a supported type in ReadVisualizationTypeJSON.");
    }
    return visualization_type;
}

// -----------------------------------------------------------------------------

// void ChangeValueJSON(const std::string& filename, std::vector<std::string> entries, const rapidjson::Value new_value) {
//     Document d = ReadFileJSON(filename);

//     for (auto& entry : entries) {
//         if (d.HasMember(entry))
//             d = d[entry];
//         else
//             throw ChException(entry " not found correctly.");
//     }

//     rapidjson::Document jsonDocument;
//     jsonDocument.SetObject();
// }

}  // namespace synchrono
}  // namespace chrono
