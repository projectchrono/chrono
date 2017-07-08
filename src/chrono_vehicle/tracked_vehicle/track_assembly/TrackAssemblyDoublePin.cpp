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
// Track assembly (double-pin) model constructed from a JSON specification file
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"

#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeDoublePin.h"

#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"

#include "chrono_vehicle/tracked_vehicle/idler/SingleIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/DoubleIdler.h"

#include "chrono_vehicle/tracked_vehicle/suspension/LinearDamperRWAssembly.h"

#include "chrono_vehicle/tracked_vehicle/roller/DoubleRoller.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// This utility function returns a ChVector from the specified JSON array
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);

    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::LoadSprocket(const std::string& filename) {
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
    assert(subtype.compare("SprocketDoublePin") == 0);

    // Create the sprocket using the appropriate template.
    m_sprocket = std::make_shared<SprocketDoublePin>(d);

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::LoadBrake(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a brake specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackBrake") == 0);

    // Extract brake type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the brake using the appropriate template.
    if (subtype.compare("TrackBrakeSimple") == 0) {
        m_brake = std::make_shared<TrackBrakeSimple>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::LoadIdler(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is an idler specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Idler") == 0);

    // Extract idler type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the idler using the appropriate template.
    if (subtype.compare("SingleIdler") == 0) {
        m_idler = std::make_shared<SingleIdler>(d);
    } else if (subtype.compare("DoubleIdler") == 0) {
        m_idler = std::make_shared<DoubleIdler>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::LoadSuspension(const std::string& filename, int which, bool has_shock) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a road-wheel assembly specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("RoadWheelAssembly") == 0);

    // Extract road-wheel assembly type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the road-wheel assembly using the appropriate template.
    if (subtype.compare("LinearDamperRWAssembly") == 0) {
        m_suspensions[which] = std::make_shared<LinearDamperRWAssembly>(d, has_shock);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::LoadRoller(const std::string& filename, int which) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a roller specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Roller") == 0);

    // Extract roller type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the roller using the appropriate template.
    if (subtype.compare("DoubleRoller") == 0) {
        m_rollers[which] = std::make_shared<DoubleRoller>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::LoadTrackShoes(const std::string& filename, int num_shoes) {
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
    assert(subtype.compare("TrackShoeDoublePin") == 0);

    // Create the track shoes using the appropriate template.
    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(std::make_shared<TrackShoeDoublePin>(d));
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackAssemblyDoublePin::TrackAssemblyDoublePin(const std::string& filename) : ChTrackAssemblyDoublePin("", LEFT) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackAssemblyDoublePin::TrackAssemblyDoublePin(const rapidjson::Document& d) : ChTrackAssemblyDoublePin("", LEFT) {
    Create(d);
}

void TrackAssemblyDoublePin::Create(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    SetName(d["Name"].GetString());

    // Create the sprocket
    {
        assert(d.HasMember("Sprocket"));
        std::string file_name = d["Sprocket"]["Input File"].GetString();
        m_sprocket_loc = loadVector(d["Sprocket"]["Location"]);
        LoadSprocket(vehicle::GetDataFile(file_name));
    }

    // Create the brake
    {
        assert(d.HasMember("Brake"));
        std::string file_name = d["Brake"]["Input File"].GetString();
        LoadBrake(vehicle::GetDataFile(file_name));
    }

    // Create the idler
    {
        assert(d.HasMember("Idler"));
        std::string file_name = d["Idler"]["Input File"].GetString();
        m_idler_loc = loadVector(d["Idler"]["Location"]);
        LoadIdler(vehicle::GetDataFile(file_name));
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
        m_susp_locs[i] = loadVector(d["Suspension Subsystems"][i]["Location"]);
        LoadSuspension(vehicle::GetDataFile(file_name), i, has_shock);
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
            m_susp_locs[i] = loadVector(d["Rollers"][i]["Location"]);
            LoadRoller(vehicle::GetDataFile(file_name), i);
        }
    }

    // Create the track shoes
    {
        assert(d.HasMember("Track Shoes"));
        std::string file_name = d["Track Shoes"]["Input File"].GetString();
        m_num_track_shoes = d["Track Shoes"]["Number Shoes"].GetInt();
        LoadTrackShoes(vehicle::GetDataFile(file_name), m_num_track_shoes);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
