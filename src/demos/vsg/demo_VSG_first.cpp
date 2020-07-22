#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vsg/ChVSGApp.h"

using namespace chrono;
using namespace geometry;
using namespace chrono::vsg;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    ChVSGApp app(&sys);

    return 0;
}
