// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// MB tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/MBTire.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MBTire::MBTire(const std::string& filename) : ChMBTire("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

MBTire::MBTire(const rapidjson::Document& d) : ChMBTire("") {
    Create(d);
}

MBTire::~MBTire() {}

void MBTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read tire mass
    assert(d.HasMember("Mass"));

    double mass = d["Mass"].GetDouble();
    SetTireMass(mass);

    // Read tire geometry
    assert(d.HasMember("Geometry"));

    int num_radius = d["Geometry"]["Radius"].Size();
    assert((int)d["Geometry"]["Offset"].Size() == num_radius);
    std::vector<double> radius(num_radius);
    std::vector<double> offset(num_radius);
    for (int i = 0; i < num_radius; i++) {
        radius[i] = d["Geometry"]["Radius"][i].GetDouble();
        offset[i] = d["Geometry"]["Offset"][i].GetDouble();
    }

    int num_divs = d["Geometry"]["Number Divisions"].GetInt();

    double rim_radius = d["Geometry"]["Rim Radius"].GetDouble();

    SetTireGeometry(radius, offset, num_divs, rim_radius);

    // Read spring properties
    assert(d.HasMember("Spring Properties"));

    auto kcR = d["Spring Properties"]["Radial"].GetArray();
    double kR = kcR[0].GetDouble();
    double cR = kcR[1].GetDouble();
    auto kcC = d["Spring Properties"]["Circumferential"].GetArray();
    double kC = kcC[0].GetDouble();
    double cC = kcC[1].GetDouble();
    auto kcT = d["Spring Properties"]["Transversal"].GetArray();
    double kT = kcT[0].GetDouble();
    double cT = kcT[1].GetDouble();

    SetRadialSpringCoefficients(kR, cR);
    SetMeshSpringCoefficients(kC, cC, kT, cT);

    // Bending springs: either common coefficients ("Bending") or separate coefficients for bending along the rings
    // ("Circumferential Bending") and across the rings ("Transversal Bending")
    if (d["Spring Properties"].HasMember("Bending")) {
        auto kcB = d["Spring Properties"]["Bending"].GetArray();
        SetBendingSpringCoefficients(kcB[0].GetDouble(), kcB[1].GetDouble());
    } else {
        assert(d["Spring Properties"].HasMember("Circumferential Bending"));
        assert(d["Spring Properties"].HasMember("Transversal Bending"));
        auto kcBC = d["Spring Properties"]["Circumferential Bending"].GetArray();
        auto kcBT = d["Spring Properties"]["Transversal Bending"].GetArray();
        SetBendingSpringCoefficients(kcBC[0].GetDouble(), kcBC[1].GetDouble(), kcBT[0].GetDouble(), kcBT[1].GetDouble());
    }

    // Interpretation of spring coefficients (default: distributed)
    if (d["Spring Properties"].HasMember("Type")) {
        std::string type = d["Spring Properties"]["Type"].GetString();
        if (type == "Distributed")
            SetSpringCoefficientType(SpringCoefficientType::DISTRIBUTED);
        else if (type == "Per Spring")
            SetSpringCoefficientType(SpringCoefficientType::PER_SPRING);
        else
            throw std::runtime_error("Unknown MB tire spring coefficient type: " + type);
    }

    // Read default internal pressure
    assert(d.HasMember("Default Pressure"));

    m_default_pressure = d["Default Pressure"].GetDouble();

    // Read contact material
    assert(d.HasMember("Contact Material"));

    ChContactMaterialData mat_info = ReadMaterialInfoJSON(d["Contact Material"]);
    SetTireContactMaterial(mat_info);

    if (d["Contact Material"].HasMember("Reference Divisions"))
        SetContactReferenceDivisions(d["Contact Material"]["Reference Divisions"].GetInt());

    // Read optional model settings
    if (d.HasMember("Model Settings")) {
        if (d["Model Settings"].HasMember("Follower Pressure"))
            EnableFollowerPressure(d["Model Settings"]["Follower Pressure"].GetBool());
        if (d["Model Settings"].HasMember("Area Weighted Lumping"))
            EnableAreaWeightedLumping(d["Model Settings"]["Area Weighted Lumping"].GetBool());
    }

    // Read optional numerical settings
    if (d.HasMember("Numerical Settings")) {
        if (d["Numerical Settings"].HasMember("Stiff")) {
            bool is_stiff = d["Numerical Settings"]["Stiff"].GetBool();
            IsStiff(is_stiff);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
