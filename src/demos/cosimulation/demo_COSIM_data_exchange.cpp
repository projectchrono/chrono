// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo on how to implement a basic cosimulation framework where data is passed
// back-forth between Chrono and Simulink.
//
// This example needs test_cosimulation.mdl to be load and run in Simulink.
//
// =============================================================================

#include "chrono/utils/ChSocketCommunication.h"

using namespace chrono;
using namespace chrono::utils;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\n"
              << "Chrono version: " << CHRONO_VERSION << std::endl;

    std::cout << "CHRONO demo about cosimulation\n" << std::endl;
    std::cout << "NOTE! This requires that you also run a copy of Simulink.\n" << std::endl;

    try {
        // Create a cosimulation interface and exchange some data with Simulink.
        int PORT_NUMBER = 50009;

        ChVectorDynamic<double> data_in(3);
        ChVectorDynamic<double> data_out(2);
        data_in.setZero();
        data_out.setZero();

        // 1) Add a socket framework object
        ChSocketFramework socket_tools;

        // 2) Create the cosimulation interface:
        ChSocketCommunication cosimul_interface(socket_tools,
                                                3,   // num. input values from Simulink
                                                2);  // num. output values to Simulink

        // 3) Wait client (Simulink) to connect...
        std::cout << " *** Waiting Simulink to start... ***\n"
                  << "(load 'data/cosimulation/test_cosimulation.mdl' in Simulink and press Start...)\n"
                  << std::endl;

        cosimul_interface.WaitConnection(PORT_NUMBER);

        double mytime = 0;
        double histime = 0;

        while (true) {
            // 4) Send data and receive data
            cosimul_interface.SendData(mytime, data_out);     // --> to Simulink
            cosimul_interface.ReceiveData(histime, data_in);  // <-- from Simulink

            mytime = histime;

            // do some example computation to update my vars (data_out)
            data_out(0) = 0.1 * data_in(0) + 0.4 * data_in(1) + 0.1 * data_in(2);
            data_out(1) = 1.0 - data_in(1);

            std::cout << "--- synchronization at time: " << mytime << std::endl << std::endl;
            std::cout << data_in << std::endl;
        }
    } catch (std::exception exception) {
        std::cerr << " ERRROR with socket system:\n" << exception.what() << std::endl;
    }

    return 0;
}
