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
    m_generators.push_back({functor, updateRate, 0, 0, functor->type, functor->id});
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

    ChJSONWriter writer;

    for (auto& generator : m_generators) {
        if ((generator.updateRate == -1 || time > generator.numLaunches / generator.updateRate - 1e-7) &&
            generator.functor->HasData()) {
            {
                writer.StartObject(generator.type, generator.id);
                generator.functor->Serialize(writer);
                writer.EndObject();
            }

            writer.EndObject();

            generator.numLaunches += 1;
        }
    }

    // Send the message to the client
    std::string str(writer.Finish());
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
    ChJSONReader reader;
    reader.Parse(message);

    std::string type;
    while (reader.HasMembers()) {
        reader.StartObject() >> type >> reader.GetObject();

        if (m_parsers.count(type))
            m_parsers[type].functor->Deserialize(reader);

        reader.EndObject();
    }

    return true;
}

// ------------------------------------------------------------------------------------

#define _INPUT(TYPE, v, it, GET)           \
    v = m_obj_iterator->value.GET##TYPE(); \
    it++;
#define INPUT(type, v) _INPUT(type, v, m_obj_iterator, Get)

ChJSONWriter::ChJSONWriter() : m_writer(m_buffer) {
    m_writer.StartArray();
}

ChJSONWriter& ChJSONWriter::operator<<(bool v) {
    m_writer.Bool(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const int v) {
    m_writer.Int(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const long int v) {
    m_writer.Int64(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const double v) {
    m_writer.Double(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(unsigned int v) {
    m_writer.Uint(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const char* v) {
    m_writer.String(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(std::string& v) {
    m_writer.String(v.c_str(), v.size());
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const std::string& v) {
    m_writer.String(v.c_str(), v.size());
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(unsigned long v) {
    m_writer.Uint64(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(unsigned long long v) {
    m_writer.Uint64(v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(ChVector<> v) {
    m_writer.StartArray();
    m_writer.Double(v.x());
    m_writer.Double(v.y());
    m_writer.Double(v.z());
    m_writer.EndArray();
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(ChQuaternion<> v) {
    m_writer.StartArray();
    m_writer.Double(v.e0());
    m_writer.Double(v.e1());
    m_writer.Double(v.e2());
    m_writer.Double(v.e3());
    m_writer.EndArray();
    return *this;
}

ChJSONWriter& ChJSONWriter::Key(const std::string& v) {
    m_writer.Key(v.c_str(), v.size());
    return *this;
}

ChJSONWriter& ChJSONWriter::PointerAsString(unsigned long v, int len) {
    std::string buffer(reinterpret_cast<char*>(v), len);
    m_writer.String(buffer.c_str(), len);
    return *this;
}

ChJSONWriter& ChJSONWriter::StartObject(const std::string& type, const std::string& id) {
    m_writer.StartObject();

    m_writer.Key("type");
    m_writer.String(type.c_str());

    m_writer.Key("id");
    m_writer.String(id.c_str());

    m_writer.Key("data");
    m_writer.StartObject();

    return *this;
}

ChJSONWriter& ChJSONWriter::EndObject() {
    m_writer.EndObject();

    return *this;
}

std::string ChJSONWriter::Finish() {
    m_writer.EndArray();
    std::string message(m_buffer.GetString());

    // Restart the buffer
    m_buffer.Clear();
    m_writer.Reset(m_buffer);
    m_writer.StartArray();

    return message;
}

// ------------------------------------------------------------------------------------

ChJSONReader::ChJSONReader() {}

void ChJSONReader::Parse(const std::string& message) {
    m_d.Parse(message.c_str());

    m_arr_iterator = m_d.Begin();
}

ChJSONReader& ChJSONReader::operator>>(bool& v) {
    INPUT(Bool, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(int& v) {
    INPUT(Int, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(long int& v) {
    INPUT(Int64, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(double& v) {
    INPUT(Double, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(float& v) {
    INPUT(Double, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(unsigned int& v) {
    INPUT(Uint, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(std::string& v) {
    INPUT(String, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(unsigned long& v) {
    INPUT(Uint64, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(unsigned long long& v) {
    INPUT(Uint64, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(std::array<double, 3>& v) {
    auto temp_arr_iterator = m_obj_iterator->value.GetArray().Begin();
    for (int i = 0; i < 3; i++, temp_arr_iterator++)
        v[i] = temp_arr_iterator->GetDouble();
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(std::array<double, 4>& v) {
    auto temp_arr_iterator = m_obj_iterator->value.GetArray().Begin();
    for (int i = 0; i < 4; i++, temp_arr_iterator++)
        v[i] = temp_arr_iterator->GetDouble();
    return *this;
}

bool ChJSONReader::GetBool() {
    bool v;
    (*this) >> v;
    return v;
}

int ChJSONReader::GetInt() {
    int v;
    (*this) >> v;
    return v;
}

long int ChJSONReader::GetLongInt() {
    long int v;
    (*this) >> v;
    return v;
}

double ChJSONReader::GetDouble() {
    double v;
    (*this) >> v;
    return v;
}

float ChJSONReader::GetFloat() {
    float v;
    (*this) >> v;
    return v;
}

unsigned int ChJSONReader::GetUint() {
    unsigned int v;
    (*this) >> v;
    return v;
}

std::string ChJSONReader::GetString() {
    std::string v;
    (*this) >> v;
    return v;
}

ChJSONReader& ChJSONReader::Next() {
    m_obj_iterator++;
    return *this;
}

ChJSONReader& ChJSONReader::Back() {
    m_obj_iterator--;
    return *this;
}

ChJSONReader& ChJSONReader::GetObject() {
    m_obj_iterator = m_obj_iterator->value.GetObject().MemberBegin();
    return *this;
}

ChJSONReader& ChJSONReader::StartObject() {
    m_obj_iterator = m_arr_iterator->MemberBegin();
    return *this;
}

ChJSONReader& ChJSONReader::EndObject() {
    m_arr_iterator++;
    return *this;
}

bool ChJSONReader::HasMembers() {
    return m_arr_iterator != m_d.End();
}

}  // end namespace vehicle
}  // end namespace chrono
