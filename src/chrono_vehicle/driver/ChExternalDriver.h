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

#ifndef CH_External_DRIVER_H
#define CH_External_DRIVER_H

#include "chrono_vehicle/ChDriver.h"
#include "chrono/utils/ChSocket.h"

#include "chrono_thirdparty/rapidjson/writer.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

#include <functional>
#include <array>

// Take care of Microsoft global definition
#ifdef _WIN32
    #ifdef GetObject
        #undef GetObject
    #endif
#endif

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

class ChJSONWriter;
class ChJSONReader;

/// Driver
class CH_VEHICLE_API ChExternalDriver : public ChDriver {
  public:
    ChExternalDriver(ChVehicle& vehicle, int port);
    virtual ~ChExternalDriver();

    /// Update the state of this driver system at the current time.
    virtual void Synchronize(double time) override;

    // ---------
    // Generator
    // ---------

    class CH_VEHICLE_API DataGeneratorFunctor {
      public:
        DataGeneratorFunctor(const std::string& type, const std::string& id) : type(type), id(id) {}
        virtual ~DataGeneratorFunctor() {}

        virtual void Serialize(ChJSONWriter& writer) = 0;
        virtual bool HasData() { return true; }

        std::string type;
        std::string id;
    };

    /// Add a data generator that is used to send data.
    /// On each Synchronize, generators are invoked based on their update rate,
    /// and data is sent to the client (external control stack).
    void AddDataGenerator(std::shared_ptr<DataGeneratorFunctor> functor, float updateRate = -1);

    // ------
    // Parser
    // ------

    class CH_VEHICLE_API DataParserFunctor {
      public:
        DataParserFunctor(const std::string& type) : type(type) {}
        virtual ~DataParserFunctor() {}

        virtual void Deserialize(ChJSONReader& reader) = 0;

        std::string type;
    };

    /// Add a data parser that is used when receiving data.
    /// On each Synchronize, parser are invoked based on the messages that are received
    void AddDataParser(std::shared_ptr<DataParserFunctor> functor);

  protected:
    /// Exchange data with the client, by sending a
    /// vector of floating point values over TCP socket
    /// connection (values are double precision, little endian, 4 bytes each)
    /// Simulator actual time is also passed as first value.
    bool SendData(double time);

    /// Exchange data with the client, by receiving a
    /// vector of floating point values over TCP socket
    /// connection (values are double precision, little endian, 4 bytes each)
    /// External time is also received as first value.
    bool ReceiveData();

    /// Wait for a client to connect to the interface,
    /// on a given port, and wait until not connected.
    /// \a port is a free port number, for example 50009.
    bool WaitConnection(int port);

  private:
    chrono::utils::ChSocketFramework m_framework;
    chrono::utils::ChSocketTCP* m_server;
    chrono::utils::ChSocketTCP* m_client;
    int m_port;

    struct DataGenerator {
        std::shared_ptr<DataGeneratorFunctor> functor;  ///< generates the data to be sent elsewhere
        float updateRate;                               ///< sensor update rate
        unsigned int numLaunches;                       ///< number of times the generator been updated
        float lastTimeUpdated;                          ///< time since previous update

        std::string type;  ///< the type used to unparse on receive
        std::string id;    ///< the name of the message type (for duplicate types)
    };
    std::vector<DataGenerator> m_generators;

    struct DataParser {
        std::shared_ptr<DataParserFunctor> functor;  ///< generates the data to be sent elsewhere

        std::string type;  ///< the type used to unparse on receive
    };
    std::map<std::string, DataParser> m_parsers;
};

/**
 * This is a helper class to generate serialized JSON messages that can be passed to/from Chrono. The expectation the
 * replciate class for this object should be used on the Chrono side
 *
 * This works as follows:
 * - The ChJSONWriter implements << operators that correspond to the types provided by rapidjson
 * - The ChJSONWriter is responsible for interacting with rapidjson, generating JSON buffers that are suitable to be
 * read on the other side
 */
class CH_VEHICLE_API ChJSONWriter {
  public:
    ChJSONWriter();

    //  Operators

    ChJSONWriter& operator<<(bool v);
    ChJSONWriter& operator<<(const int v);
    ChJSONWriter& operator<<(const long int v);
    ChJSONWriter& operator<<(const double v);
    ChJSONWriter& operator<<(unsigned int v);
    ChJSONWriter& operator<<(const char* v);
    ChJSONWriter& operator<<(std::string& v);
    ChJSONWriter& operator<<(const std::string& v);
    ChJSONWriter& operator<<(unsigned long v);
    ChJSONWriter& operator<<(unsigned long long v);
    ChJSONWriter& operator<<(ChVector<> v);
    ChJSONWriter& operator<<(ChQuaternion<> v);
    ChJSONWriter& operator<<(ChJSONWriter&) { return *this; }

    ChJSONWriter& Key(const std::string& v);

    ChJSONWriter& PointerAsString(unsigned long v, int len);

    ChJSONWriter& StartObject(const std::string& type, const std::string& id);
    ChJSONWriter& EndObject();

    std::string Finish();

  private:
    rapidjson::StringBuffer m_buffer;
    rapidjson::Writer<rapidjson::StringBuffer> m_writer;
};

// ------------------------------------------------------------------------------------

class CH_VEHICLE_API ChJSONReader {
  public:
    ChJSONReader();

    void Parse(const std::string& message);

    //  Operators

    ChJSONReader& operator>>(bool& v);
    ChJSONReader& operator>>(int& v);
    ChJSONReader& operator>>(long int& v);
    ChJSONReader& operator>>(double& v);
    ChJSONReader& operator>>(float& v);
    ChJSONReader& operator>>(unsigned int& v);
    ChJSONReader& operator>>(std::string& v);
    ChJSONReader& operator>>(unsigned long& v);
    ChJSONReader& operator>>(unsigned long long& v);
    ChJSONReader& operator>>(std::array<double, 3>& v);
    ChJSONReader& operator>>(std::array<double, 4>& v);
    ChJSONReader& operator>>(ChJSONReader&) { return *this; }

    // Primarily for Python users

    bool GetBool();
    int GetInt();
    long int GetLongInt();
    double GetDouble();
    float GetFloat();
    unsigned int GetUint();
    std::string GetString();

    ChJSONReader& Next();
    ChJSONReader& Back();

    ChJSONReader& StartObject();
    ChJSONReader& GetObject();
    ChJSONReader& EndObject();

    bool HasMembers();

  private:
    rapidjson::Document m_d;
    rapidjson::Value::ConstValueIterator m_arr_iterator;
    rapidjson::Value::ConstMemberIterator m_obj_iterator;
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
