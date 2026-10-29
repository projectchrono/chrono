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

#include "chrono/utils/ChSocket.h"
#include <memory>

using namespace chrono;
using namespace chrono::utils;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n";

    std::cout << "CHRONO demo about sockets\n";

    try {
        // Test 1 ==============================================================
        // Initialize platform-specific socket framework
        ChSocketFramework socket_framework;

        // Get the name and IP of local host, and display infos
        ChSocketHostInfo local_host;
        std::cout << " Local host information:" << "\n";
        std::cout << "    Name:    " << local_host.GetHostName() << "\n";
        std::cout << "    Address: " << local_host.GetHostIPAddress() << "\n";

        //// TODO: the pointer of local_host gets null after some time
        //// thus triggering an exception when asked to get its name
        std::string local_host_name = local_host.GetHostName();


        // Test 2 ==============================================================
        // Create a server and listen to a client on a port
        // It is assumed that, while waiting, a client will send something to this port

        // Create a server that can listen to a given port
        const int port_number = 50009;
        ChSocketTCP server(port_number);

        // Bind socket to server
        std::cout << "Binding server to socket... ";
        server.BindSocket();
        std::cout << "done\n";

        // Wait for a client to connect
        // This might put the program in a long waiting state, so a timeout could be useful
        std::cout << "Listening for connection...\n";
        std::cout << "(load 'data/cosimulation/test_socket.mdl' in Simulink and press Start...)\n";
        server.ListenToClient(1);

        // Accept incoming client connection
        std::string client_host_name;
        auto connection = std::unique_ptr<ChSocketTCP>(server.AcceptClient(client_host_name));

        if (!connection)
            throw std::runtime_error("Server failed in getting the client socket");
        std::cout << "Successfully connected with client: " << client_host_name << "\n";


        // Test 3 ==============================================================
        // Exchange some data between server and client

        double a_out = 0;
        double a = 0, b = 0, c = 0;

        while (true) {
            // Send single variable through templated approach
            int size_sent = connection->SendData(a_out);
            std::cout << local_host_name << " will send a buffer of " << size_sent << " bytes\n";

            // Receive multiple variables at once through templated approach
            int size_recv = connection->ReceiveData(a, b, c);
            std::cout << "Received " << size_recv << " bytes\n";

            std::cout << "a = " << a << "\n" << " b = " << b << "\n" << " c = " << c << "\n";

            // Update sent data
            a_out = 0.5 * b;
        }

    } catch (const std::exception& exc) {
        std::cerr << "Error with socket system:\n" << exc.what() << "\n";
    }

    std::cout << "Done\n";
    return 0;
}
