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
// Authors: Aaron Young
// =============================================================================
//
//
// =============================================================================

#include "chrono_vehicle/driver/ChExternalDriver.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

ChExternalDriver::ChExternalDriver(ChVehicle& vehicle, int port) : ChDriver(vehicle) {
    WaitConnection(port);
}

ChExternalDriver::~ChExternalDriver() {
    if (m_server)
        delete m_server;
    m_server = 0;
    if (m_client)
        delete m_client;
    m_client = 0;
}

void ChExternalDriver::Synchronize(double time) {
    SendData(time);
    ReceiveData();
}

void ChExternalDriver::AddDataGenerator(std::shared_ptr<DataGeneratorFunctor> functor, float updateRate) {
    m_generators.push_back({functor, updateRate, 0, 0, functor->type, functor->name});
};

void ChExternalDriver::AddDataParser(std::shared_ptr<DataParserFunctor> functor) {
    m_parsers[functor->type] = {functor, functor->type};
};

bool ChExternalDriver::WaitConnection(int port) {
    GetLog() << "Waiting for connection on port " << port << "...\n";

    m_port = port;

    // a server is created that can listen at the given port
    m_server = new utils::ChSocketTCP(port);

    // bind the socket
    m_server->bindSocket();

    // Wait for a client to connect
    m_server->listenToClient(1);

    // When a client is available, connect to it
    std::string clientHostName;
    m_client = m_server->acceptClient(clientHostName);

    if (!m_client)
        throw utils::ChExceptionSocket(0, "Server failed in getting the client socket.");

    GetLog() << "Connected to client: (" << clientHostName << ", " << port << ")\n";

    return true;
}

bool ChExternalDriver::SendData(double time) {
    if (!m_client)
        throw utils::ChExceptionSocket(0, "Error. Attempted 'SendData' with no connected client.");

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);

    writer.StartArray();

    for (auto& generator : m_generators) {
        if ((generator.updateRate == -1 || time > generator.numLaunches / generator.updateRate - 1e-7) &&
            generator.functor->HasData()) {
            writer.StartObject();

            // type
            {
                writer.Key("type");
                writer.String(generator.type.c_str());
            }

            // name
            {
                writer.Key("name");
                writer.String(generator.name.c_str());
            }

            // data
            {
                writer.Key("data");
                writer.StartObject();
                generator.functor->Serialize(writer);
                writer.EndObject();
            }

            writer.EndObject();
            generator.numLaunches += 1;
        }
    }

    writer.EndArray();

    // Send the message to the client
    std::string str(buffer.GetString());
    m_client->sendMessage(str);

    return true;
}

bool ChExternalDriver::ReceiveData() {
    if (!m_client)
        throw utils::ChExceptionSocket(0, "Error. Attempted 'ReceiveData' with no connected client.");

    // Receive from the client
    std::string message;
    m_client->receiveMessage(message);

    // Parse the JSON string
    Document d;
    d.Parse(message.c_str());

    for (auto& m : d.GetArray()) {
        auto type = m["type"].GetString();
        if (m_parsers.count(type))
            m_parsers[type].functor->Deserialize(m["data"].GetObject());
    }

    return true;
}

// ------------------------------
// Common Data Generator Functors
// ------------------------------

}  // end namespace vehicle
}  // end namespace chrono
