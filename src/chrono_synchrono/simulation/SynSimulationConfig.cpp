#include "chrono_synchrono/simulation/SynSimulationConfig.h"

#include "chrono_synchrono/cli/SynCLI.h"

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

namespace chrono {
namespace synchrono {

double STEP_SIZE = 3e-3;
SYN_API double END_TIME = 20.0;
SYN_API double HEARTBEAT = 1e-2;
SYN_API bool VERBOSE = false;
SYN_API ChContactMethod CONTACT_METHOD = ChContactMethod::NSC;

SYN_API void ConfigFromCLI(SynCLI* cli) {
    STEP_SIZE = cli->GetAsType<double>("step_size");
    END_TIME = cli->GetAsType<double>("end_time");
    HEARTBEAT = cli->GetAsType<double>("heartbeat");
    VERBOSE = cli->GetAsType<bool>("verbose");
    ContactMethodFromString(cli->GetAsType<std::string>("contact_method"));
}

SYN_API void ContactMethodFromString(const std::string& contact_method) {
    if (contact_method.compare("NSC") && contact_method.compare("SMC")) {
        std::cout << "\"" << contact_method
                  << "\" is not a viable ChContactMethod. Must be either \"NSC\" or "
                     "\"SMC\". Leaving CONTACT_METHOD unchanged."
                  << std::endl;
        return;
    }
    CONTACT_METHOD = contact_method.compare("NSC") == 0 ? ChContactMethod::NSC : ChContactMethod::SMC;
}

SYN_API std::string ContactMethodToString(ChContactMethod contact_method) {
    std::string out = contact_method == ChContactMethod::NSC ? "NSC" : "SMC";
    return out;
}

SYN_API std::shared_ptr<ChMaterialSurface> DefaultMaterialSurface() {
    switch (CONTACT_METHOD) {
        case ChContactMethod::NSC: {
            auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            mat->SetFriction(0.9f);
            mat->SetRestitution(0.01f);
            return mat;
        }
        case ChContactMethod::SMC: {
            auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            mat->SetFriction(0.9f);
            mat->SetRestitution(0.01f);
            mat->SetYoungModulus(2e7f);
            return mat;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

}  // namespace synchrono
}  // namespace chrono