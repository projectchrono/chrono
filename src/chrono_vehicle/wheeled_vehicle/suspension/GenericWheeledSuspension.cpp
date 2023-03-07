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
GenericWheeledSuspension::GenericWheeledSuspension(const std::string& filename)
    : ChGenericWheeledSuspension("")
    {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

GenericWheeledSuspension::GenericWheeledSuspension(const rapidjson::Document& d)
    : ChGenericWheeledSuspension("")
    {
    Create(d);
}

GenericWheeledSuspension::~GenericWheeledSuspension() {}

// Although this is a utility function, I doubt it has much use outside of this suspension.
// Also I had problems linking the code when I tried to move this to ChUtilsJSON.
ChGenericWheeledSuspension::BodyIdentifier ReadBodyIdentifierJSON(const Value& a) {
    assert(a.IsString());
    std::string name = a.GetString();
    if (name.compare("Chassis") == 0) {
        return ChGenericWheeledSuspension::ChassisIdentifier();
    }
    else if (name.compare("Subchassis") == 0) {
        return ChGenericWheeledSuspension::SubchassisIdentifier();
    }
    else if (name.compare("Steering") == 0) {
        return ChGenericWheeledSuspension::SteeringIdentifier();
    }
    else {
        return ChGenericWheeledSuspension::BodyIdentifier(name);
    }
}

std::shared_ptr<ChPointPointShape> ReadTSDAGeometryJSON(const Value &a) {
    if (a.HasMember("Visualization") && a["Visualization"].IsObject()) {
        auto& vis = a["Visualization"];
        assert(vis.IsObject());
        std::string type = vis["Type"].GetString();
        if (type == "Segment") {
            auto segment = std::make_shared<ChSegmentShape>();
            return segment;
        }
        else if (type == "Spring") {
            // the default below are copied from the actual class, and since there
            // are no setters I can't just take the default constructor and override
            // the properties the user specifies (my only alternative would be to
            // construct and read from a prototype instance)
            double radius = 0.05;
            double resolution = 65;
            double turns = 5;
            if (vis.HasMember("Radius") && vis["Radius"].IsDouble()) {
                radius = vis["Radius"].GetDouble();
            }
            if (vis.HasMember("Resolution") && vis["Resolution"].IsDouble()) {
                resolution = vis["Resolution"].GetDouble();
            }
            if (vis.HasMember("Turns") && vis["Turns"].IsDouble()) {
                turns = vis["Turns"].GetDouble();
            }
            auto spring = std::make_shared<ChSpringShape>(radius, resolution, turns);
            return spring;
        }
    }
    return nullptr;
}

// -----------------------------------------------------------------------------
// Worker function for creating a GenericWheeledSuspension suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void GenericWheeledSuspension::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read camber and toe data
    if (d.HasMember("Camber Angle (deg)")) {
        m_camber_angle = d["Camber Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    }
    else {
        m_camber_angle = 0;
    }
    if (d.HasMember("Toe Angle (deg)")) {
        m_toe_angle = d["Toe Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    }
    else {
        m_toe_angle = 0;
    }

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());
    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_spindlePosition = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    if (d.HasMember("Spindle Attachment Body")) {
        m_spindleAttachmentBody = ReadBodyIdentifierJSON(d["Spindle Attachment Body"]);
    }
    if (d.HasMember("Antiroll Body")) {
        m_antirollBody = ReadBodyIdentifierJSON(d["Antiroll Body"]);
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
            auto geometry = std::make_shared<ChVehicleGeometry>(ReadVehicleGeometry(body));
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
            auto csys = ReadCoordinateSystemJSON(joint["Coordinate System"]);
            std::shared_ptr<ChVehicleBushingData> bushingData = nullptr;
            if (joint.HasMember("Bushing Data") && joint["Bushing Data"].IsObject()) {
                bushingData = ReadBushingDataJSON(joint["Bushing Data"]);
            }
            DefineJoint(name, mirrored, type, body1, body2, csys, bushingData);
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
            auto force = ReadTSDAFunctorJSON(tsda["Force"], free_length);
            auto geometry = ReadTSDAGeometryJSON(tsda);
            // TODO confirm that 'free_length' is the 'rest_length' that the function below expects
            DefineTSDA(name, mirrored, body1, body2, point1, point2, free_length, force, geometry);
            std::cout << "DefineTSDA: " << name << " is mirrored? " << (mirrored ? "True" : "False") << std::endl;
        }
    }

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
