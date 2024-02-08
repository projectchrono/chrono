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

using namespace chrono;
using namespace chrono::utils;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\n"
              << "Chrono version: " << CHRONO_VERSION << std::endl;

    std::cout << "CHRONO demo about sockets\n" << std::endl;

    try {
        ChSocketFramework* socket_tools = new ChSocketFramework;

        // Test 1
        // Get the name and IP of local host, and display infos

        ChSocketHostInfo local_host;

        std::cout << " Local host information:" << std::endl;
        std::cout << "      Name:    " << local_host.getHostName() << std::endl;
        std::cout << "      Address: " << local_host.getHostIPAddress() << std::endl;

        // Test 2
        // create a server and listen to a client on a port
        // (it is assumed that, when waiting, a client will send
        // something to this port)

        int PORT_NUMBER = 50009;

        // a server is created, that could listen at a given port
        ChSocketTCP myServer(PORT_NUMBER);

        // bind socket to server
        std::cout << "Binding to socket..." << std::endl;
        myServer.bindSocket();

        // wait for a client to connect (this might put the program in
        // a long waiting state... a timeout can be useful then)
        std::cout << "Listening for connection..." << std::endl;
        std::cout << "(load 'data/cosimulation/test_socket.mdl' in Simulink and press Start...)" << std::endl;
        myServer.listenToClient(1);

        ChSocketTCP* client;
        std::string clientHostName;
        client = myServer.acceptClient(clientHostName);  // pick up the call!

        if (!client)
            throw std::runtime_error("Server failed in getting the client socket");

        std::cout << "OK! Connected with client: " << clientHostName << std::endl;

        double a_out = 0;
        double a, b, c = 0;

        auto write_to_charv = [](std::vector<char>& buffer, size_t offset, auto data_in) {
            for (auto s = 0; s < sizeof(data_in); ++s) {
                buffer[offset + s] = reinterpret_cast<char*>(&data_in)[s];
            }
        };

        auto read_from_charv = [](const std::vector<char>& buffer, size_t offset, auto& data_out) {
            for (auto s = 0; s < sizeof(data_out); ++s) {
                reinterpret_cast<char*>(&data_out)[s] = buffer[offset + s];
            }
        };

        //// TODO: the pointer of local_host gets null after some time
        //// thus triggering an exception when asket to get its name
        auto local_host_name = std::string(local_host.getHostName());

        while (true) {
            // Send to the client

            std::vector<char> sbuffer(sizeof(double));
            write_to_charv(sbuffer, 0, a_out);

            std::cout << local_host_name << " will send a buffer of " << sbuffer.size() << " bytes." << std::endl;

            // -----> SEND!!!
            client->SendBuffer(sbuffer);

            // Receive from the client
            int nbytes = sizeof(double) * 3;
            std::vector<char> rbuffer;
            rbuffer.resize(nbytes);  // reserve to number of expected bytes

            // -----> RECEIVE!!!
            int numBytes = client->ReceiveBuffer(rbuffer, nbytes);

            std::cout << "Received " << numBytes << " bytes" << std::endl;

            read_from_charv(rbuffer, 0, a);
            read_from_charv(rbuffer, sizeof(double), b);
            read_from_charv(rbuffer, 2 * sizeof(double), c);

            std::cout << " a = " << a << std::endl << " b = " << b << std::endl << " c = " << c << std::endl;

            // change output var just for fun
            a_out = 0.5 * b;
        }

        delete client;

        // Last stuff to do
        delete socket_tools;

    } catch (std::exception exception) {
        std::cerr << " ERROR with socket system:\n" << exception.what() << std::endl;
    }

    return 0;
}
