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

#include <vector>

#include "chrono/utils/ChSocketCommunication.h"

using namespace chrono::utils;

namespace chrono {
namespace utils {

ChSocketCommunication::ChSocketCommunication(ChSocketFramework& framework,  // socket framework
                                             int n_in_values,               // number of recv scalar variables
                                             int n_out_values               // number of send variables
) {
    myServer = 0;
    myClient = 0;
    in_n = n_in_values;
    out_n = n_out_values;
    nport = 0;
}

ChSocketCommunication::~ChSocketCommunication() {
    delete myServer;
    myServer = nullptr;
    delete myClient;
    myClient = nullptr;
}

bool ChSocketCommunication::WaitConnection(int port) {
    nport = port;

    // a server is created, that could listen at a given port
    myServer = new ChSocketTCP(port);

    // bind socket to server
    myServer->bindSocket();

    // wait for a client to connect (this might put the program in
    // a long waiting state... a timeout can be useful then)
    myServer->listenToClient(1);

    std::string clientHostName;
    myClient = myServer->acceptClient(clientHostName);  // pick up the call!

    if (!myClient)
        throw std::runtime_error("Server failed in getting the client socket");

    return true;
}

bool ChSocketCommunication::SendData(double time, ChVectorConstRef out_data) {
    if (out_data.size() != out_n)
        throw std::runtime_error("ERROR: Sent data must be a vector of size N.");
    if (!myClient)
        throw std::runtime_error("ERROR: Attempted 'SendData' with no connected client.");

    std::vector<char> mbuffer;
    mbuffer.resize((out_data.size() + 1) * sizeof(double));
    // Serialize data (little endian): send time
    for (size_t ds = 0; ds < sizeof(double); ++ds) {
        mbuffer[ds] = (reinterpret_cast<char*>(&time))[ds];
    }

    // variables:
    for (int i = 0; i < out_data.size(); i++) {
        for (size_t ds = 0; ds < sizeof(double); ++ds) {
            mbuffer[(i + 1) * sizeof(double) + ds] =
                reinterpret_cast<char*>(const_cast<double*>(&out_data.data()[i]))[ds];
        }
    }

    // -----> SEND!!!
    myClient->SendBuffer(mbuffer);

    return true;
}

bool ChSocketCommunication::ReceiveData(double& time, ChVectorRef in_data) {
    if (in_data.size() != in_n)
        throw std::runtime_error("ERROR: Received data must be a vector of size N.");
    if (!myClient)
        throw std::runtime_error("ERROR: Attempted 'ReceiveData' with no connected client.");

    // Receive from the client
    int nbytes = sizeof(double) * (in_n + 1);
    std::vector<char> rbuffer;
    rbuffer.resize(nbytes);

    // -----> RECEIVE!!!
    /*int numBytes =*/myClient->ReceiveBuffer(rbuffer, nbytes);

    // Deserialize data (little endian): retrieve time
    for (size_t ds = 0; ds < sizeof(double); ++ds) {
        reinterpret_cast<char*>(&time)[ds] = rbuffer[ds];
    }

    // retrieve variables:
    for (size_t i = 0; i < (size_t)in_data.size(); ++i) {
        for (size_t ds = 0; ds < sizeof(double); ++ds) {
            reinterpret_cast<char*>(&(in_data.data()[i]))[ds] = rbuffer[(i + 1) * sizeof(double) + ds];
        }
    }

    return true;
}

}  // end namespace utils
}  // end namespace chrono
