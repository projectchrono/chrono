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
// Tracked vehicle model constructed from a JSON specification file
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
//
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackedVehicle::TrackedVehicle(const std::string& filename, ChMaterialSurface::ContactMethod contact_method)
    : ChTrackedVehicle("", contact_method) {
    Create(filename);
}

TrackedVehicle::TrackedVehicle(ChSystem* system, const std::string& filename) : ChTrackedVehicle("", system) {
    Create(filename);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackedVehicle::Create(const std::string& filename) {
    // -------------------------------------------
    // Open and parse the input file
    // -------------------------------------------
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    std::string subtype = d["Template"].GetString();
    assert(type.compare("Vehicle") == 0);
    assert(subtype.compare("TrackedVehicle") == 0);

    SetName(name);

    // -------------------------------------------
    // Create the chassis system
    // -------------------------------------------
    assert(d.HasMember("Chassis"));

    {
        std::string file_name = d["Chassis"]["Input File"].GetString();
        m_chassis = ReadChassisJSON(vehicle::GetDataFile(file_name));
        if (d["Chassis"].HasMember("Output")) {
            m_chassis->SetOutput(d["Chassis"]["Output"].GetBool());
        }
    }

    // ------------------------------------
    // Create the track assembly subsystems
    // ------------------------------------

    assert(d.HasMember("Track Assemblies"));
    assert(d["Track Assemblies"].IsArray());
    int num_tracks = d["Track Assemblies"].Size();
    assert(num_tracks == 2);

    {
        std::string file_name = d["Track Assemblies"][0u]["Input File"].GetString();
        m_tracks[VehicleSide::LEFT] = ReadTrackAssemblySON(vehicle::GetDataFile(file_name));
        if (d["Track Assemblies"][0u].HasMember("Output")) {
            m_tracks[VehicleSide::LEFT]->SetOutput(d["Track Assemblies"][0u]["Output"].GetBool());
        }
        m_track_offset[LEFT] = d["Track Assemblies"][0u]["Offset"].GetDouble();
    }
    {
        std::string file_name = d["Track Assemblies"][1u]["Input File"].GetString();
        m_tracks[VehicleSide::RIGHT] = ReadTrackAssemblySON(vehicle::GetDataFile(file_name));
        if (d["Track Assemblies"][1u].HasMember("Output")) {
            m_tracks[VehicleSide::RIGHT]->SetOutput(d["Track Assemblies"][1u]["Output"].GetBool());
        }
        m_track_offset[RIGHT] = d["Track Assemblies"][1u]["Offset"].GetDouble();
    }

    // --------------------
    // Create the driveline
    // --------------------

    assert(d.HasMember("Driveline"));

    {
        std::string file_name = d["Driveline"]["Input File"].GetString();
        m_driveline = ReadTrackDrivelineJSON(vehicle::GetDataFile(file_name));
        if (d["Driveline"].HasMember("Output")) {
            m_driveline->SetOutput(d["Driveline"]["Output"].GetBool());
        }
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackedVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Invoke base class method to initialize the chassis.
    ChTrackedVehicle::Initialize(chassisPos, chassisFwdVel);

    // Initialize the left and right track assemblies.
    m_tracks[0]->Initialize(m_chassis->GetBody(), ChVector<>(0, m_track_offset[0], 0));
    m_tracks[1]->Initialize(m_chassis->GetBody(), ChVector<>(0, m_track_offset[1], 0));

    // Initialize the driveline
    m_driveline->Initialize(m_chassis->GetBody(), m_tracks[0], m_tracks[1]);
}

}  // end namespace vehicle
}  // end namespace chrono
