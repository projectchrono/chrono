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
// Authors: Radu Serban
// =============================================================================
//
// Track assembly (single-pin) model constructed from a JSON specification file
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"

#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketSinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeSinglePin.h"

#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"

#include "chrono_vehicle/tracked_vehicle/idler/SingleIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/DoubleIdler.h"

#include "chrono_vehicle/tracked_vehicle/suspension/LinearDamperRWAssembly.h"

#include "chrono_vehicle/tracked_vehicle/roller/DoubleRoller.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblySinglePin::ReadSprocket(const std::string& filename, int output) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a sprocket specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sprocket") == 0);

    // Check sprocket type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    assert(subtype.compare("SprocketSinglePin") == 0);

    // Create the sprocket using the appropriate template.
    m_sprocket = std::make_shared<SprocketSinglePin>(d);

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_sprocket->SetOutput(output == +1);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblySinglePin::ReadTrackShoes(const std::string& filename, int num_shoes, int output) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a track shoe specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackShoe") == 0);

    // Check track shoe type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    assert(subtype.compare("TrackShoeSinglePin") == 0);

    // Create the track shoes using the appropriate template.
    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(std::make_shared<TrackShoeSinglePin>(d));
    }

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_shoes[0]->SetOutput(output == +1);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackAssemblySinglePin::TrackAssemblySinglePin(const std::string& filename) : ChTrackAssemblySinglePin("", LEFT) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackAssemblySinglePin::TrackAssemblySinglePin(const rapidjson::Document& d) : ChTrackAssemblySinglePin("", LEFT) {
    Create(d);
}

void TrackAssemblySinglePin::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Create the sprocket
    {
        assert(d.HasMember("Sprocket"));
        std::string file_name = d["Sprocket"]["Input File"].GetString();
        int output = 0;
        if (d["Sprocket"].HasMember("Output")) {
            output = d["Sprocket"]["Output"].GetBool() ? +1 : -1;
        }
        ReadSprocket(vehicle::GetDataFile(file_name), output);
        m_sprocket_loc = ReadVectorJSON(d["Sprocket"]["Location"]);
    }

    // Create the brake
    {
        assert(d.HasMember("Brake"));
        std::string file_name = d["Brake"]["Input File"].GetString();
        m_brake = ReadTrackBrakeJSON(vehicle::GetDataFile(file_name));
        if (d["Brake"].HasMember("Output")) {
            m_brake->SetOutput(d["Brake"]["Output"].GetBool());
        }
    }

    // Create the idler
    {
        assert(d.HasMember("Idler"));
        std::string file_name = d["Idler"]["Input File"].GetString();
        m_idler = ReadIdlerJSON(vehicle::GetDataFile(file_name));
        if (d["Idler"].HasMember("Output")) {
            m_idler->SetOutput(d["Idler"]["Output"].GetBool());
        }
        m_idler_loc = ReadVectorJSON(d["Idler"]["Location"]);
    }

    // Create the suspensions
    assert(d.HasMember("Suspension Subsystems"));
    assert(d["Suspension Subsystems"].IsArray());
    m_num_susp = d["Suspension Subsystems"].Size();
    m_suspensions.resize(m_num_susp);
    m_susp_locs.resize(m_num_susp);
    for (int i = 0; i < m_num_susp; i++) {
        std::string file_name = d["Suspension Subsystems"][i]["Input File"].GetString();
        bool has_shock = d["Suspension Subsystems"][i]["Has Shock"].GetBool();
        m_suspensions[i] = ReadRoadWheelAssemblyJSON(vehicle::GetDataFile(file_name), has_shock);
        if (d["Suspension Subsystems"][i].HasMember("Output")) {
            m_suspensions[i]->SetOutput(d["Suspension Subsystems"][i]["Output"].GetBool());
        }
        m_susp_locs[i] = ReadVectorJSON(d["Suspension Subsystems"][i]["Location"]);
    }

    // Create the rollers
    m_num_rollers = 0;
    if (d.HasMember("Rollers")) {
        assert(d["Rollers"].IsArray());
        m_num_rollers = d["Rollers"].Size();
        m_rollers.resize(m_num_rollers);
        m_roller_locs.resize(m_num_rollers);
        for (int i = 0; i < m_num_rollers; i++) {
            std::string file_name = d["Rollers"][i]["Input File"].GetString();
            m_rollers[i] = ReadRollerJSON(vehicle::GetDataFile(file_name));
            if (d["Rollers"][i].HasMember("Output")) {
                m_rollers[i]->SetOutput(d["Rollers"][i]["Output"].GetBool());
            }
            m_roller_locs[i] = ReadVectorJSON(d["Rollers"][i]["Location"]);
        }
    }

    // Create the track shoes
    {
        assert(d.HasMember("Track Shoes"));
        std::string file_name = d["Track Shoes"]["Input File"].GetString();
        int output = 0;
        if (d["Track Shoes"].HasMember("Output")) {
            output = d["Track Shoes"]["Output"].GetBool() ? +1 : -1;
        }
        m_num_track_shoes = d["Track Shoes"]["Number Shoes"].GetInt();
        ReadTrackShoes(vehicle::GetDataFile(file_name), m_num_track_shoes, output);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
