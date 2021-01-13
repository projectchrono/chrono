#include "chrono_synchrono/utils/SynLog.h"

namespace chrono {
namespace synchrono {

// Global node id
static SynNodeID* GlobalNodeID = NULL;

void SetLogNodeID(SynNodeID node_id) {
    GlobalNodeID = new SynNodeID(node_id);
}

std::string GetAppendingString(SynNodeID nid);
ChStreamOutAscii& AppendLog(SynNodeID);

ChStreamOutAscii& SynLog() {
    return GlobalNodeID != NULL ? AppendLog((*GlobalNodeID)) : GetLog();
}

std::string GetAppendingString(SynNodeID nid) {
    return "[" + std::to_string(nid) + "]:\t";
}

ChStreamOutAscii& AppendLog(SynNodeID nid) {
    return GetLog() << GetAppendingString(nid).c_str();
}

}  // namespace synchrono
}  // namespace chrono