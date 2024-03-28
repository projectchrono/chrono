#include "chrono_synchrono/utils/SynLog.h"

#include <iostream>
#include <string>

namespace chrono {
namespace synchrono {

// Global node id
static int* GlobalNodeID = NULL;

void SetLogNodeID(int node_id) {
    GlobalNodeID = new int(node_id);
}

std::string GetAppendingString(int node_id);
std::ostream& AppendLog(int);

std::ostream& SynLog() {
    return GlobalNodeID != NULL ? AppendLog((*GlobalNodeID)) : std::cout;
}

std::string GetAppendingString(int node_id) {
    return "[" + std::to_string(node_id) + "]:\t";
}

std::ostream& AppendLog(int node_id) {
    return std::cout << GetAppendingString(node_id);
}

}  // namespace synchrono
}  // namespace chrono