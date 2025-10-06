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

#include "chrono_vehicle/tracked_vehicle/idler/TranslationalIdler.h"
//#include "chrono_vehicle/tracked_vehicle/idler/DistanceIdler.h"

#include "chrono_vehicle/tracked_vehicle/suspension/TranslationalDamperSuspension.h"

#include "chrono_vehicle/tracked_vehicle/track_wheel/DoubleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/SingleTrackWheel.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::ReadSprocket(const std::string& filename, int output) {
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
    assert(subtype.compare("SprocketDoublePin") == 0);

    // Create the sprocket using the appropriate template.
    m_sprocket = chrono_types::make_shared<SprocketDoublePin>(d);

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_sprocket->SetOutput(output == +1);
    }

    std::cout << "  Loaded JSON " << filename << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackAssemblyDoublePin::ReadTrackShoes(const std::string& filename, int num_shoes, int output) {
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
    assert(subtype.compare("TrackShoeDoublePin") == 0);

    // Create the track shoes using the appropriate template.
    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(chrono_types::make_shared<TrackShoeDoublePin>(d));
    }

    // A non-zero value of 'output' indicates overwriting the subsystem's flag
    if (output != 0) {
        m_shoes[0]->SetOutput(output == +1);
    }

    std::cout << "  Loaded JSON " << filename << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackAssemblyDoublePin::TrackAssemblyDoublePin(const std::string& filename) : ChTrackAssemblyDoublePin("", LEFT) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

TrackAssemblyDoublePin::TrackAssemblyDoublePin(const rapidjson::Document& d) : ChTrackAssemblyDoublePin("", LEFT) {
    Create(d);
}

void TrackAssemblyDoublePin::Create(const rapidjson::Document& d) {
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
        ReadSprocket(GetVehicleDataFile(file_name), output);
        m_sprocket_loc = ReadVectorJSON(d["Sprocket"]["Location"]);
    }

    // Create the brake
    {
        assert(d.HasMember("Brake"));
        std::string file_name = d["Brake"]["Input File"].GetString();
        m_brake = ReadTrackBrakeJSON(GetVehicleDataFile(file_name));
        if (d["Brake"].HasMember("Output")) {
            m_brake->SetOutput(d["Brake"]["Output"].GetBool());
        }
    }

    // Create the idler
    {
        assert(d.HasMember("Idler"));
        std::string file_name = d["Idler"]["Input File"].GetString();
        m_idler = ReadIdlerJSON(GetVehicleDataFile(file_name));
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
        m_suspensions[i] = ReadTrackSuspensionJSON(GetVehicleDataFile(file_name), has_shock, lock_arm);
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
            m_rollers[i] = ReadTrackWheelJSON(GetVehicleDataFile(file_name));
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
        ReadTrackShoes(GetVehicleDataFile(file_name), m_num_track_shoes, output);

        if (d["Track Shoes"].HasMember("RSDA Data")) {
            double k = d["Track Shoes"]["RSDA Data"]["Stiffness Rotational"].GetDouble();
            double c = d["Track Shoes"]["RSDA Data"]["Damping Rotational"].GetDouble();
            m_torque_funct = chrono_types::make_shared<ChTrackAssemblySegmented::TrackBendingFunctor>(k, c);
        }

        if (d["Track Shoes"].HasMember("Bushing Data")) {
            m_bushing_data = ReadBushingDataJSON(d["Track Shoes"]["Bushing Data"]);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
