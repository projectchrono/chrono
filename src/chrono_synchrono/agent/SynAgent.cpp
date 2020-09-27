#include "chrono_synchrono/agent/SynAgent.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

Document SynAgent::ParseAgentFileJSON(const std::string& filename) {
    // Open and parse the input file
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        throw ChException("Agent file not read properly in ParseAgentFileJSON.");

    // Read top-level data
    assert(d.HasMember("Name"));
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    std::string suptype = d["Template"].GetString();
    assert(type.compare("Agent") == 0);

    return d;
}

}  // namespace synchrono
}  // namespace chrono
