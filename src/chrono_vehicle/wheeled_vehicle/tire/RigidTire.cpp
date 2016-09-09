// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Rigid tire constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigidTire::RigidTire(const std::string& filename) : ChRigidTire("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RigidTire::RigidTire(const rapidjson::Document& d) : ChRigidTire("") {
    Create(d);
}

void RigidTire::Create(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    SetName(d["Name"].GetString());

    float mu = d["Coefficient of Friction"].GetDouble();
    float cr = d["Coefficient of Restitution"].GetDouble();
    float ym = d["Young Modulus"].GetDouble();
    float pr = d["Poisson Ratio"].GetDouble();
    float kn = d["Normal Stiffness"].GetDouble();
    float gn = d["Normal Damping"].GetDouble();
    float kt = d["Tangential Stiffness"].GetDouble();
    float gt = d["Tangential Damping"].GetDouble();

    SetContactMaterial(mu, cr, ym, pr, kn, gn, kt, gt);

    m_radius = d["Radius"].GetDouble();
    m_width = d["Width"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
