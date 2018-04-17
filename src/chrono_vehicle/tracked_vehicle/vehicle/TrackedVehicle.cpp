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

#include <cstdio>

#include "chrono/ChConfig.h"

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/tracked_vehicle/driveline/SimpleTrackDriveline.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandBushing.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#ifdef CHRONO_FEA
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandANCF.h"
#endif

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackedVehicle::LoadChassis(const std::string& filename, int output) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a chassis specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Chassis") == 0);

    // Extract the chassis type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    if (subtype.compare("RigidChassis") == 0) {
        m_chassis = std::make_shared<RigidChassis>(d);
    }

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_chassis->SetOutput(output == +1);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackedVehicle::LoadTrackAssembly(const std::string& filename, VehicleSide side, int output) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a steering specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackAssembly") == 0);

    // Extract the track assembly type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    if (subtype.compare("TrackAssemblySinglePin") == 0) {
        m_tracks[side] = std::make_shared<TrackAssemblySinglePin>(d);
    } else if (subtype.compare("TrackAssemblyDoublePin") == 0) {
        m_tracks[side] = std::make_shared<TrackAssemblyDoublePin>(d);
    } else if (subtype.compare("TrackAssemblyBandBushing") == 0) {
        m_tracks[side] = std::make_shared<TrackAssemblyBandBushing>(d);
    } else if (subtype.compare("TrackAssemblyBandANCF") == 0) {
#ifdef CHRONO_FEA
        m_tracks[side] = std::make_shared<TrackAssemblyBandANCF>(d);
#else
        std::cout << "ERROR: Attempting to load an ANCF-based continuous-band track, but FEA support is disabled."
                  << std::endl;
#endif
    }

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_tracks[side]->SetOutput(output == +1);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackedVehicle::LoadDriveline(const std::string& filename, int output) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a driveline specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackDriveline") == 0);

    // Extract the driveline type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the driveline using the appropriate template.
    if (subtype.compare("SimpleTrackDriveline") == 0) {
        m_driveline = std::make_shared<SimpleTrackDriveline>(d);
    }

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_driveline->SetOutput(output == +1);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

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
        int output = 0;
        if (d["Chassis"].HasMember("Output")) {
            output = d["Chassis"]["Output"].GetBool() ? +1 : -1;
        }
        LoadChassis(vehicle::GetDataFile(file_name), output);
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
        int output = 0;
        if (d["Track Assemblies"][0u].HasMember("Output")) {
            output = d["Track Assemblies"][0u]["Output"].GetBool() ? +1 : -1;
        }
        LoadTrackAssembly(vehicle::GetDataFile(file_name), VehicleSide::LEFT, output);
        m_track_offset[LEFT] = d["Track Assemblies"][0u]["Offset"].GetDouble();
    }
    {
        std::string file_name = d["Track Assemblies"][1u]["Input File"].GetString();
        int output = 0;
        if (d["Track Assemblies"][1u].HasMember("Output")) {
            output = d["Track Assemblies"][1u]["Output"].GetBool() ? +1 : -1;
        }
        LoadTrackAssembly(vehicle::GetDataFile(file_name), VehicleSide::RIGHT, output);
        m_track_offset[RIGHT] = d["Track Assemblies"][1u]["Offset"].GetDouble();
    }

    // --------------------
    // Create the driveline
    // --------------------

    assert(d.HasMember("Driveline"));

    {
        std::string file_name = d["Driveline"]["Input File"].GetString();
        int output = 0;
        if (d["Driveline"].HasMember("Output")) {
            output = d["Driveline"]["Output"].GetBool() ? +1 : -1;
        }
        LoadDriveline(vehicle::GetDataFile(file_name), output);
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
