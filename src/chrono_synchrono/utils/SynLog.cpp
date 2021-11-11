#include "chrono_synchrono/utils/SynLog.h"

namespace chrono {
namespace synchrono {

// Global node id
static int* GlobalNodeID = NULL;

void SetLogNodeID(int node_id) {
    GlobalNodeID = new int(node_id);
}

std::string GetAppendingString(int node_id);
ChStreamOutAscii& AppendLog(int);

ChStreamOutAscii& SynLog() {
    return GlobalNodeID != NULL ? AppendLog((*GlobalNodeID)) : GetLog();
}

std::string GetAppendingString(int node_id) {
    return "[" + std::to_string(node_id) + "]:\t";
}

ChStreamOutAscii& AppendLog(int node_id) {
    return GetLog() << GetAppendingString(node_id).c_str();
}

}  // namespace synchrono
}  // namespace chrono