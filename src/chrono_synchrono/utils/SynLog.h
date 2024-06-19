#ifndef SYN_LOG_H
#define SYN_LOG_H

#include "chrono_synchrono/SynApi.h"
#include <ostream>

/// @addtogroup synchrono_utils
/// @{

namespace chrono {
namespace synchrono {

/// Will set the node id for this specific node.
/// That way, the logger will print the correct node id
SYN_API void SetLogNodeID(int node_id);

/// "Overriden" global function to get the current std::cout object
/// Will prepend any output with the global id set from SetLogNodeID
SYN_API std::ostream& SynLog();

}  // namespace synchrono
}  // namespace chrono

/// @} synchrono_utils

#endif