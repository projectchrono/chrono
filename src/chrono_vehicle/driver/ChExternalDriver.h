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

#include <functional>

// Take care of Microsoft idiocy
#ifdef _WIN32
    #ifdef GetObject
        #undef GetObject
    #endif
#endif

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Driver
class CH_VEHICLE_API ChExternalDriver : public ChDriver {
  public:
    ChExternalDriver(ChVehicle& vehicle, int port, bool add_defaults = true);
    virtual ~ChExternalDriver();

    /// Update the state of this driver system at the current time.
    virtual void Synchronize(double time) override;

    // ---------
    // Generator
    // ---------

    class DataGeneratorFunctor {
      public:
        DataGeneratorFunctor(const std::string& type, const std::string& name) : type(type), name(name) {}
        virtual void Serialize(rapidjson::Writer<rapidjson::StringBuffer>& writer) = 0;
        virtual bool HasData() { return true; }

      protected:
        template <class Real = double>
        void Serialize_ChVector(rapidjson::Writer<rapidjson::StringBuffer>& writer, const ChVector<Real>& v) {
            writer.StartArray();

            writer.Double(v.x());
            writer.Double(v.y());
            writer.Double(v.z());

            writer.EndArray();
        }

        template <class Real = double>
        void Serialize_ChQuaternion(rapidjson::Writer<rapidjson::StringBuffer>& writer, const ChQuaternion<Real>& v) {
            writer.StartArray();

            writer.Double(v.e0());
            writer.Double(v.e1());
            writer.Double(v.e2());
            writer.Double(v.e3());

            writer.EndArray();
        }

      private:
        std::string type;
        std::string name;

        friend class ChExternalDriver;
    };

    /// Add a data generator that is used to send data.
    /// On each Synchronize, generators are invoked based on their update rate,
    /// and data is sent to the client (external control stack).
    void AddDataGenerator(std::shared_ptr<DataGeneratorFunctor> functor, float updateRate = -1);

    // ------
    // Parser
    // ------

    class DataParserFunctor {
      public:
        DataParserFunctor(const std::string& type) : type(type) {}
        virtual void Deserialize(rapidjson::Value& v) = 0;

      private:
        std::string type;

        friend class ChExternalDriver;
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
        std::string name;  ///< the name of the message type (for duplicate types)
    };
    std::vector<DataGenerator> m_generators;

    struct DataParser {
        std::shared_ptr<DataParserFunctor> functor;  ///< generates the data to be sent elsewhere

        std::string type;  ///< the type used to unparse on receive
    };
    std::map<std::string, DataParser> m_parsers;
};

// ------------------------------
// Common Data Generator Functors
// ------------------------------

class ChSystem_DataGeneratorFunctor : public ChExternalDriver::DataGeneratorFunctor {
  public:
    ChSystem_DataGeneratorFunctor(ChSystem* system) : DataGeneratorFunctor("ChSystem", "system"), m_system(system) {}

    virtual void Serialize(rapidjson::Writer<rapidjson::StringBuffer>& writer) override {
        writer.String("time");
        writer.Double(m_system->GetChTime());
    }

  private:
    ChSystem* m_system;
};

class ChVehicle_DataGeneratorFunctor : public ChExternalDriver::DataGeneratorFunctor {
  public:
    ChVehicle_DataGeneratorFunctor(ChVehicle& vehicle) : DataGeneratorFunctor("ChVehicle", "ego"), m_vehicle(vehicle) {}

    virtual void Serialize(rapidjson::Writer<rapidjson::StringBuffer>& writer) override {
        auto body = m_vehicle.GetChassisBody();

        writer.String("pos");
        this->Serialize_ChVector(writer, body->GetPos());
        writer.String("rot");
        this->Serialize_ChQuaternion(writer, body->GetRot());
        writer.String("pos_dt");
        this->Serialize_ChVector(writer, body->GetPos_dt());
        writer.String("rot_dt");
        this->Serialize_ChQuaternion(writer, body->GetRot_dt());
        writer.String("pos_dtdt");
        this->Serialize_ChVector(writer, body->GetPos_dtdt());
        writer.String("rot_dtdt");
        this->Serialize_ChQuaternion(writer, body->GetRot_dtdt());
    }

  private:
    ChVehicle& m_vehicle;
};

// -----------------------------
// Common Data Receiver Functors
// -----------------------------

class ChDriverInputs_DataParserFunctor : public ChExternalDriver::DataParserFunctor {
  public:
    ChDriverInputs_DataParserFunctor(ChDriver& driver) : DataParserFunctor("ChDriverInputs"), m_driver(driver) {}

    virtual void Deserialize(rapidjson::Value& v) override {
        m_driver.SetThrottle(v["throttle"].GetDouble());
        m_driver.SetSteering(v["steering"].GetDouble());
        m_driver.SetBraking(v["braking"].GetDouble());
    }

  private:
    ChDriver& m_driver;
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
