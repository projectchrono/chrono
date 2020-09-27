#ifndef SYN_SIMULATION_CONFIG_H
#define SYN_SIMULATION_CONFIG_H

#include "chrono_synchrono/SynApi.h"

#include "chrono/physics/ChMaterialSurface.h"

using namespace chrono;

namespace chrono {
namespace synchrono {

class SynCLI;

extern SYN_API double STEP_SIZE;
extern SYN_API double END_TIME;
extern SYN_API double HEARTBEAT;
extern SYN_API bool VERBOSE;
extern SYN_API ChContactMethod CONTACT_METHOD;

SYN_API void ConfigFromCLI(SynCLI* cli);

SYN_API void ContactMethodFromString(const std::string& contact_method);
SYN_API std::string ContactMethodToString(ChContactMethod contact_method);

SYN_API std::shared_ptr<ChMaterialSurface> DefaultMaterialSurface();

}  // namespace synchrono
}  // namespace chrono

#endif