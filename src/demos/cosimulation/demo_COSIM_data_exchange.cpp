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

#include "chrono/core/ChLog.h"
#include "chrono/core/ChMatrixDynamic.h"

#include "chrono_cosimulation/ChCosimulation.h"
#include "chrono_cosimulation/ChExceptionSocket.h"

using namespace chrono;
using namespace chrono::cosimul;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // To write something to the console, use the chrono::GetLog()

    GetLog() << "CHRONO demo about cosimulation \n\n";
    GetLog() << "NOTE! This requires that you also run a copy of Simulink. \n\n";

    try {
        // Test 1
        // Create a cosimulation interface and exchange some data
        // with Simulink.

        int PORTNUM = 50009;

        ChMatrixDynamic<double> data_in(3, 1);
        ChMatrixDynamic<double> data_out(2, 1);
        data_in.Reset();
        data_out.Reset();

        // 1) Add a socket framework object
        ChSocketFramework socket_tools;

        // 2) Create the cosimulation interface:

        ChCosimulation cosimul_interface(socket_tools,
                                         3,   // n.input values from Simulink
                                         2);  // n.output values to Simulink

        // 3) Wait client (Simulink) to connect...

        GetLog() << " *** Waiting Simulink to start... *** \n     (load 'data/cosimulation/test_cosimulation.mdl' in "
                    "Simulink and press Start...)\n\n";

        cosimul_interface.WaitConnection(PORTNUM);

        double mytime = 0;
        double histime = 0;

        while (true) {
            // 4) Send data and receive data
            cosimul_interface.SendData(mytime, &data_out);     // --> to Simulink
            cosimul_interface.ReceiveData(histime, &data_in);  // <-- from Simulink

            mytime = histime;

            // do some example computation to update my vars (data_out)
            data_out(0) = 0.1 * data_in(0) + 0.4 * data_in(1) + 0.1 * data_in(2);
            data_out(1) = 1.0 - data_in(1);

            GetLog() << "--- synchronization at time: " << mytime << "\n\n";
            GetLog() << data_in << "\n";
        }

    } catch (ChExceptionSocket exception) {
        GetLog() << " ERRROR with socket system: \n" << exception.what() << "\n";
    }

    return 0;
}
