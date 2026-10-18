%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/ChSensorRenderTypes.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"
#include "chrono_sensor/optix/ChOptixScene.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

using namespace chrono;
using namespace chrono::sensor;

%}

/* ChOptixDefinitions.h must be parsed before the Background struct below, so that the
   BackgroundMode enum it uses is a known (wrapped) type rather than an opaque one. */
%include "chrono_sensor/optix/ChOptixDefinitions.h"

/* chrono::sensor::Background is defined in chrono_sensor/ChSensorRenderTypes.h, which cannot be
   %included here: that header selects between an OptiX and a Vulkan-RT definition of the shared
   render types through preprocessor branches that SWIG does not parse reliably. Declare the
   struct directly instead -- its layout is the same in both branches. Without this, SWIG knows
   the name Background (from ChScene::SetBackground) but generates no constructor for it, so
   Python code cannot build one; see pyutest_SEN_data_access.py. */
namespace chrono {
namespace sensor {

struct Background {
    BackgroundMode mode;
    chrono::ChVector3f color_zenith;
    chrono::ChVector3f color_horizon;
    std::string env_tex;
};

}  // namespace sensor
}  // namespace chrono

/* Parse the header files to generate wrappers */
%include "chrono_sensor/optix/ChOptixScene.h"
%include "chrono_sensor/optix/ChOptixUtils.h"
