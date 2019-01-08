/*
 * ChUtilsJSON.h
 *
 *  Created on: Oct 24, 2018
 *      Author: Milad Rakhsha
 */
#ifndef CHUTILSJSON_H_
#define CHUTILSJSON_H_
#include "chrono_fsi/custom_math.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
using namespace rapidjson;
struct SimParams;

namespace chrono {
namespace fsi {
namespace utils {

CH_FSI_API bool ParseJSON(const char* json_file, SimParams* paramsH, Real3 Domain);
CH_FSI_API Real3 LoadVectorJSON(const Value& a);
CH_FSI_API void InvalidArg(std::string arg);

}  // namespace utils
}  // namespace fsi
}  // namespace chrono

#endif /* CHUTILSJSON_H_ */
