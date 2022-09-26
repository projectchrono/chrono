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
// Demo on how to use low level functions of TCP 'sockets' (these can be used to
// send messages between applications, for co-simulation)
//
// =============================================================================

#include "chrono/core/ChLog.h"

#include "chrono/utils/ChSocket.h"

using namespace chrono;
using namespace chrono::utils;

int main(int argc, char* argv[]) {
    // To write something to the console, use the chrono::GetLog()
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << "CHRONO demo about sockets \n\n";

    try {
        ChSocketFramework* socket_tools = new ChSocketFramework;

        // Test 1
        // Get the name and IP of local host, and display infos

        ChSocketHostInfo local_host;

        GetLog() << " Local host information: \n";
        GetLog() << "      Name:    " << local_host.getHostName() << "\n";
        GetLog() << "      Address: " << local_host.getHostIPAddress() << "\n";

        // Test 2
        // create a server and listen to a client on a port
        // (it is assumed that, when waiting, a client will send
        // something to this port)

        int PORT_NUMBER = 50009;

        // a server is created, that could listen at a given port
        ChSocketTCP myServer(PORT_NUMBER);

        // bind socket to server
        GetLog() << "Binding to socket... \n";
        myServer.bindSocket();

        // wait for a client to connect (this might put the program in
        // a long waiting state... a timeout can be useful then)
        GetLog() << "Listening for connection... \n";
        GetLog() << "(load 'data/cosimulation/test_socket.mdl' in Simulink and press Start...)\n\n";
        myServer.listenToClient(1);

        ChSocketTCP* client;
        std::string clientHostName;
        client = myServer.acceptClient(clientHostName);  // pick up the call!

        if (!client)
            throw(ChExceptionSocket(0, "Server failed in getting the client socket"));

        GetLog() << "OK! Connected with client: " << clientHostName << "\n";

        double a_out = 0;
        double a, b, c = 0;

        while (true) {
            // Send to the client

            std::vector<char> mbuffer;                     // zero length
            ChStreamOutBinaryVector stream_out(&mbuffer);  // wrap the buffer, for easy formatting

            stream_out << a_out;

            GetLog() << local_host.getHostName() << " will send a buffer of n." << stream_out.GetVector()->size()
                     << " bytes. \n";

            // -----> SEND!!!
            client->SendBuffer(*stream_out.GetVector());

            // Receive from the client
            int nbytes = 8 * 3;
            std::vector<char> rbuffer;
            rbuffer.resize(nbytes);                      // reserve to number of expected bytes
            ChStreamInBinaryVector stream_in(&rbuffer);  // wrap the buffer, for easy formatting

            // -----> RECEIVE!!!
            int numBytes = client->ReceiveBuffer(*stream_in.GetVector(), nbytes);

            GetLog() << "Received " << numBytes << " bytes\n";

            stream_in >> a;
            stream_in >> b;
            stream_in >> c;
            GetLog() << " a = " << a << "\n b = " << b << "\n c = " << c << "\n";

            // change output var just for fun
            a_out = 0.5 * b;
        }

        delete client;

        // Last stuff to do
        delete socket_tools;

    } catch (ChExceptionSocket exception) {
        GetLog() << " ERRROR with socket system: \n" << exception.what() << "\n";
    }

    return 0;
}
