// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Fiala tire constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"

#include "thirdparty/rapidjson/filereadstream.h"

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
FialaTire::FialaTire(const std::string& filename) : ChFialaTire("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

FialaTire::FialaTire(const rapidjson::Document& d) : ChFialaTire("") {
    Create(d);
}

FialaTire::~FialaTire() {
}

void FialaTire::Create(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    SetName(d["Name"].GetString());

    // Read in Fiala tire model parameters
    m_unloaded_radius = d["Fiala Parameters"]["Unloaded Radius"].GetDouble();
    m_width = d["Fiala Parameters"]["Width"].GetDouble();
    m_normalStiffness = d["Fiala Parameters"]["Vertical Stiffness"].GetDouble();
    m_normalDamping = d["Fiala Parameters"]["Vertical Damping"].GetDouble();
    m_rolling_resistance = 0;  // d["Fiala Parameters"]["Rolling Resistance"].GetDouble();
    m_c_slip = d["Fiala Parameters"]["CSLIP"].GetDouble();
    m_c_alpha = d["Fiala Parameters"]["CALPHA"].GetDouble();
    m_u_min = d["Fiala Parameters"]["UMIN"].GetDouble();
    m_u_max = d["Fiala Parameters"]["UMAX"].GetDouble();
    m_relax_length_x = d["Fiala Parameters"]["X Relaxation Length"].GetDouble();
    m_relax_length_y = d["Fiala Parameters"]["Y Relaxation Length"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
