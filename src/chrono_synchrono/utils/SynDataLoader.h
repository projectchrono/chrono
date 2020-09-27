#ifndef SYN_DATA_LOADER_H
#define SYN_DATA_LOADER_H

#include <string>

#include "chrono_synchrono/SynApi.h"

namespace chrono {
namespace synchrono {

/// Set the path to the SynChrono data directory (ATTENTION: not thread safe)
SYN_API void SetSynChronoDataPath(const std::string& path);

/// Obtain the current path to the SynChrono data directory (thread safe)
SYN_API const std::string& GetSynChronoDataPath();

/// Obtain the complete path to the specified filename, given relative to the
/// SynChrono data directory (thread safe)
SYN_API std::string GetSynDataFile(const std::string& filename);

}  // namespace synchrono
}  // namespace chrono

#endif
