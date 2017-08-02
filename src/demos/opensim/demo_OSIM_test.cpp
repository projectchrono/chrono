#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChParserOpenSim.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

#include <functional>
#include <cassert>
#include <cmath>

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::irrlicht;

using namespace irr;
using namespace rapidxml;

int main(int argc, char* argv[]) {
    // Make a system
    ChSystemSMC my_system;

    // Create parser instance
    ChParserOpenSim parser;

    std::string filename;
    filename = std::string("../../data/opensim/Rajagopal2015-objs.osim");
    // filename = std::string("../../data/opensim/dancing_dude.osim");

    if (argc == 2) {
        filename = std::string(argv[1]);
    }
    // Use MESH for the Rajagopal file, PRIMITIVES for files that don't have the data in data/opensim
    parser.parse(my_system, filename.c_str(), ChParserOpenSim::VisType::MESH);
    // parser.parse(my_system, filename.c_str(), ChParserOpenSim::VisType::PRIMITIVES);

    // Setup Irrlicht
    ChIrrApp application(&my_system, L"ChBodyAuxRef demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, 6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }
    return 0;
}
