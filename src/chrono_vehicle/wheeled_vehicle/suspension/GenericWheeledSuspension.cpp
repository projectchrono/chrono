// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Generic wheeled vehicle suspension constructed with data from file.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/suspension/GenericWheeledSuspension.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a suspension using data from the specified JSON file.
// -----------------------------------------------------------------------------
GenericWheeledSuspension::GenericWheeledSuspension(const std::string& filename) : ChGenericWheeledSuspension("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

GenericWheeledSuspension::GenericWheeledSuspension(const rapidjson::Document& d) : ChGenericWheeledSuspension("") {
    Create(d);
}

GenericWheeledSuspension::~GenericWheeledSuspension() {}

ChGenericWheeledSuspension::BodyIdentifier ReadBodyIdentifierJSON(const Value& a) {
    assert(a.IsString() || a.IsArray());
    std::string name;
    std::string side;
    int vehicleSide = -1;
    if (a.IsArray() && a.GetArray().Size() == 2) {
        name = a[0].GetString();
        side = a[1].GetString();
        if (side == "Left") {
            vehicleSide = VehicleSide::LEFT;
        }
        else if (side == "Right") {
            vehicleSide = VehicleSide::RIGHT;
        }
    }
    else if (a.IsString()) {
        name = a.GetString();
    }
    if (name.compare("Chassis") == 0) {
        return ChGenericWheeledSuspension::ChassisIdentifier();
    } else if (name.compare("Subchassis") == 0) {
        return ChGenericWheeledSuspension::SubchassisIdentifier();
    } else if (name.compare("Steering") == 0) {
        return ChGenericWheeledSuspension::SteeringIdentifier();
    } else {
        return ChGenericWheeledSuspension::BodyIdentifier(name, vehicleSide);
    }
}

// -----------------------------------------------------------------------------
// Worker function for creating a GenericWheeledSuspension suspension using data
// in the specified RapidJSON document.
// -----------------------------------------------------------------------------
void GenericWheeledSuspension::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read suspension characteristics
    assert(d.HasMember("Steerable"));
    assert(d.HasMember("Independent"));
    m_steerable = d["Steerable"].GetBool();
    m_independent = d["Independent"].GetBool();

    // Read camber and toe data
    if (d.HasMember("Camber Angle (deg)")) {
        m_camberAngle = d["Camber Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    } else {
        m_camberAngle = 0;
    }
    if (d.HasMember("Toe Angle (deg)")) {
        m_toeAngle = d["Toe Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    } else {
        m_toeAngle = 0;
    }

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());
    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_spindlePosition = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();
    assert(d["Spindle"].HasMember("Attachment Body"));
    m_spindleAttachmentBody = ReadBodyIdentifierJSON(d["Spindle"]["Attachment Body"]);

    // Read body for attaching an antiroll bar subsystem
    if (m_independent) {
        assert(d.HasMember("Antiroll Body"));
        m_antirollBody = ReadBodyIdentifierJSON(d["Antiroll Body"]);
    } else {
        m_antirollBody = BodyIdentifier("");
    }

    // Read bodies
    if (d.HasMember("Bodies")) {
        auto bodies = d["Bodies"].GetArray();
        for (auto& body : bodies) {
            auto name = body["Name"].GetString();
            auto mirrored = body["Mirrored"].GetBool();
            auto pos = ReadVectorJSON(body["Position"]);
            auto rot = ReadQuaternionJSON(body["Rotation"]);
            auto mass = body["Mass"].GetDouble();
            auto inertia_moments = ReadVectorJSON(body["Moments of Inertia"]);
            auto inertia_products = ReadVectorJSON(body["Products of Inertia"]);
            auto geometry = std::make_shared<ChVehicleGeometry>(ReadVehicleGeometryJSON(body));
            DefineBody(name, mirrored, pos, rot, mass, inertia_moments, inertia_products, geometry);
        }
    }

    // Read joints
    if (d.HasMember("Joints")) {
        auto joints = d["Joints"].GetArray();
        for (auto& joint : joints) {
            auto name = joint["Name"].GetString();
            auto mirrored = joint["Mirrored"].GetBool();
            auto type = ReadVehicleJointTypeJSON(joint["Type"]);
            auto body1 = ReadBodyIdentifierJSON(joint["Body1"]);
            auto body2 = ReadBodyIdentifierJSON(joint["Body2"]);
            auto pos = ReadVectorJSON(joint["Position"]);
            auto rot = ReadQuaternionJSON(joint["Rotation"]);
            std::shared_ptr<ChVehicleBushingData> bushingData = nullptr;
            if (joint.HasMember("Bushing Data") && joint["Bushing Data"].IsObject()) {
                bushingData = ReadBushingDataJSON(joint["Bushing Data"]);
            }
            DefineJoint(name, mirrored, type, body1, body2, pos, rot, bushingData);
        }
    }

    // Read distance constraints
    if (d.HasMember("Distance Constraints")) {
        auto dists = d["Distance Constraints"].GetArray();
        for (auto& dist : dists) {
            auto name = dist["Name"].GetString();
            auto mirrored = dist["Mirrored"].GetBool();
            auto body1 = ReadBodyIdentifierJSON(dist["Body1"]);
            auto body2 = ReadBodyIdentifierJSON(dist["Body2"]);
            auto point1 = ReadVectorJSON(dist["Point1"]);
            auto point2 = ReadVectorJSON(dist["Point2"]);
            DefineDistanceConstraint(name, mirrored, body1, body2, point1, point2);
        }
    }

    // Read TSDAs
    if (d.HasMember("TSDAs")) {
        auto tsdas = d["TSDAs"].GetArray();
        for (auto& tsda : tsdas) {
            auto name = tsda["Name"].GetString();
            auto mirrored = tsda["Mirrored"].GetBool();
            auto body1 = ReadBodyIdentifierJSON(tsda["Body1"]);
            auto body2 = ReadBodyIdentifierJSON(tsda["Body2"]);
            auto point1 = ReadVectorJSON(tsda["Point1"]);
            auto point2 = ReadVectorJSON(tsda["Point2"]);
            double free_length;
            auto force = ReadTSDAFunctorJSON(tsda, free_length);
            auto geometry = std::make_shared<ChTSDAGeometry>(ReadTSDAGeometryJSON(tsda));
            DefineTSDA(name, mirrored, body1, body2, point1, point2, free_length, force, geometry);
        }
    }

    // Read TSDAs
    if (d.HasMember("RSDAs")) {
        auto rsdas = d["RSDAs"].GetArray();
        for (auto& rsda : rsdas) {
            auto name = rsda["Name"].GetString();
            auto mirrored = rsda["Mirrored"].GetBool();
            auto body1 = ReadBodyIdentifierJSON(rsda["Body1"]);
            auto body2 = ReadBodyIdentifierJSON(rsda["Body2"]);
            auto pos = ReadVectorJSON(rsda["Position"]);
            auto axis = ReadVectorJSON(rsda["Axis"]);
            double free_angle;
            auto torque = ReadRSDAFunctorJSON(rsda, free_angle);
            DefineRSDA(name, mirrored, body1, body2, pos, axis, free_angle, torque);
        }
    }

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
