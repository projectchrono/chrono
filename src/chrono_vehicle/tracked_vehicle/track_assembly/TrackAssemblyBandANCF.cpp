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
// Track assembly (band-ANCF) model constructed from a JSON specification file
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandANCF.h"

#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketBand.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandANCF.h"

#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"

#include "chrono_vehicle/tracked_vehicle/idler/TranslationalIdler.h"
//#include "chrono_vehicle/tracked_vehicle/idler/DistanceIdler.h"

#include "chrono_vehicle/tracked_vehicle/suspension/TranslationalDamperSuspension.h"

#include "chrono_vehicle/tracked_vehicle/track_wheel/DoubleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/SingleTrackWheel.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyBandANCF::ReadSprocket(const std::string& filename, int output) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    // Check that the given file is a sprocket specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sprocket") == 0);

    // Check sprocket type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    assert(subtype.compare("SprocketBand") == 0);

    // Create the sprocket using the appropriate template.
    m_sprocket = chrono_types::make_shared<SprocketBand>(d);

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_sprocket->SetOutput(output == +1);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyBandANCF::ReadTrackShoes(const std::string& filename, int num_shoes, int output) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    // Check that the given file is a track shoe specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackShoe") == 0);

    // Check track shoe type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    assert(subtype.compare("TrackShoeBandANCF") == 0);

    // Create the track shoes using the appropriate template.
    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(chrono_types::make_shared<TrackShoeBandANCF>(d));
    }

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_shoes[0]->SetOutput(output == +1);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackAssemblyBandANCF::TrackAssemblyBandANCF(const std::string& filename) : ChTrackAssemblyBandANCF("", LEFT) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackAssemblyBandANCF::TrackAssemblyBandANCF(const rapidjson::Document& d) : ChTrackAssemblyBandANCF("", LEFT) {
    Create(d);
}

void TrackAssemblyBandANCF::Create(const rapidjson::Document& d) {
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
        bool lock_arm = false;
        if (d["Suspension Subsystems"][i].HasMember("Lock Arm")) {
            lock_arm = d["Suspension Subsystems"][i]["Lock Arm"].GetBool();
        }
        m_suspensions[i] = ReadTrackSuspensionJSON(vehicle::GetDataFile(file_name), has_shock, lock_arm);
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
            m_rollers[i] = ReadTrackWheelJSON(vehicle::GetDataFile(file_name));
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

    // Read web mesh materials
    assert(d.HasMember("Layer Materials"));
    assert(d["Layer Materials"].HasMember("Steel"));
    assert(d["Layer Materials"].HasMember("Rubber"));

    double rho_steel = d["Layer Materials"]["Steel"]["Density"].GetDouble();
    ChVector<> E_steel = ReadVectorJSON(d["Layer Materials"]["Steel"]["E"]);
    ChVector<> nu_steel = ReadVectorJSON(d["Layer Materials"]["Steel"]["nu"]);
    ChVector<> G_steel = ReadVectorJSON(d["Layer Materials"]["Steel"]["G"]);
    SetRubberLayerMaterial(rho_steel, E_steel, nu_steel, G_steel);

    double rho_rubber = d["Layer Materials"]["Rubber"]["Density"].GetDouble();
    ChVector<> E_rubber = ReadVectorJSON(d["Layer Materials"]["Rubber"]["E"]);
    ChVector<> nu_rubber = ReadVectorJSON(d["Layer Materials"]["Rubber"]["nu"]);
    ChVector<> G_rubber = ReadVectorJSON(d["Layer Materials"]["Rubber"]["G"]);
    SetRubberLayerMaterial(rho_rubber, E_rubber, nu_rubber, G_rubber);

    ChVector<> ply_angles = ReadVectorJSON(d["Layer Materials"]["Ply Angles"]);
    SetLayerFiberAngles(ply_angles[0], ply_angles[1], ply_angles[2]);

    double alpha = d["Layer Materials"]["Structural Damping Coefficient"].GetDouble();
    SetElementStructuralDamping(alpha);

    // Read web contact material properties
    assert(d.HasMember("Mesh Contact Material"));
    m_mat_info = ReadMaterialInfoJSON(d["Mesh Contact Material"]);
}

void TrackAssemblyBandANCF::CreateContactMaterial(ChContactMethod contact_method) {
    m_contact_material = m_mat_info.CreateMaterial(contact_method);
}

}  // end namespace vehicle
}  // end namespace chrono
