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
// Definition of a track testing mechanism (as a vehicle).
// The tested track can be specified through a stand-alone JSON file or as a
// specified track assembly in a vehicle JSON specification file.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackTestRig.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "thirdparty/rapidjson/document.h"
#include "thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion, respectively,
// from the specified JSON array.
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackTestRig::ChTrackTestRig(const std::string& filename,
                               VehicleSide side,
                               ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method) {
    // ---------------------------------------------------------------
    // Open and parse the input file (vehicle JSON specification file)
    // ---------------------------------------------------------------
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Create the chassis (ground) body, fixed, no visualizastion
    m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef(m_system->GetContactMethod()));
    m_chassis->SetIdentifier(0);
    m_chassis->SetName("ground");
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetBodyFixed(true);

    ChSharedPtr<ChColorAsset> blue(new ChColorAsset);
    blue->SetColor(ChColor(0.2f, 0.2f, 0.8f));
    m_chassis->AddAsset(blue);

    m_system->Add(m_chassis);

    //// TODO

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChTrackTestRig::ChTrackTestRig(const std::string& filename, ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method) {
    // -----------------------------------------------------------
    // Open and parse the input file (rig JSON specification file)
    // -----------------------------------------------------------

    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Create the chassis (ground) body, fixed, no visualizastion
    m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef(m_system->GetContactMethod()));
    m_chassis->SetIdentifier(0);
    m_chassis->SetName("ground");
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetBodyFixed(true);

    ChSharedPtr<ChColorAsset> blue(new ChColorAsset);
    blue->SetColor(ChColor(0.2f, 0.2f, 0.8f));
    m_chassis->AddAsset(blue);

    m_system->Add(m_chassis);

    //// TODO

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

void ChTrackTestRig::Initialize(const ChCoordsys<>& chassisPos) {
    // ---------------------------------
    // Initialize the vehicle subsystems
    // ---------------------------------

    m_track->Initialize(m_chassis, m_sprocketLoc, m_idlerLoc, m_suspLocs);

    /*

    // ------------------------------------------
    // Create and initialize the shaker post body
    // ------------------------------------------

    double post_height = 0.1;
    double post_rad = 0.4;

    ChVector<> spindle_L_pos = m_suspension->GetSpindlePos(LEFT);
    ChVector<> post_L_pos = spindle_L_pos;
    post_L_pos.z -= (m_wheels[LEFT]->GetRadius() + post_height / 2.0);

    m_post = ChSharedPtr<ChBody>(new ChBody(m_system->GetContactMethod()));
    m_post->SetPos(post_L_pos);
    m_system->Add(m_post);
    AddVisualize_post(m_post, m_chassis, post_height, post_rad, ChColor(0.1f, 0.8f, 0.15f));

    // ------------------------------------------
    // Create and initialize joints and actuators
    // ------------------------------------------

    // Prismatic joint to force vertical translation
    m_post_prismatic = ChSharedPtr<ChLinkLockPrismatic>(new ChLinkLockPrismatic);
    m_post_prismatic->SetNameString("L_post_prismatic");
    m_post_prismatic->Initialize(m_chassis, m_post, ChCoordsys<>(ChVector<>(post_L_pos), QUNIT));
    m_system->AddLink(m_post_prismatic);

    // Post actuator
    ChVector<> m1 = post_L_pos;
    m1.z -= 1.0;  // offset marker 1 location 1 meter below marker 2
    m_post_linact = ChSharedPtr<ChLinkLinActuator>(new ChLinkLinActuator);
    m_post_linact->SetNameString("Post_linActuator");
    m_post_linact->Initialize(m_chassis, m_post, false, ChCoordsys<>(m1, QUNIT), ChCoordsys<>(post_L_pos, QUNIT));
    m_post_linact->Set_lin_offset(1.0);
    ChSharedPtr<ChFunction_Const> func(new ChFunction_Const(0));
    m_post_linact->Set_dist_funct(func);
    m_system->AddLink(m_post_linact);

    // Constrain in a horizontal plane (based on current post location)
    m_post_ptPlane = ChSharedPtr<ChLinkLockPointPlane>(new ChLinkLockPointPlane());
    m_post_ptPlane->SetNameString("Post_pointPlane");
    m_post_ptPlane->Initialize(m_suspension->GetSpindle(LEFT), m_post, ChCoordsys<>(spindle_L_pos, QUNIT));
    m_system->AddLink(m_post_ptPlane);

    */
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackTestRig::GetActuatorDisp() {
    double time = GetSystem()->GetChTime();
    return m_post_linact->Get_dist_funct()->Get_y(time);
}

double ChTrackTestRig::GetActuatorForce() {
    return m_post_linact->Get_react_force().x;
}

double ChTrackTestRig::GetActuatorMarkerDist() {
    return m_post_linact->GetDist();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackTestRig::Update(double time, double disp, const TrackShoeForces& shoe_forces) {
    // Apply the displacements to the left/right post actuators
    if (ChSharedPtr<ChFunction_Const> func = m_post_linact->Get_dist_funct().DynamicCastTo<ChFunction_Const>())
        func->Set_yconst(disp);

    // Apply contact forces to track shoe bodies.

    //// TODO

    // Cache driver inputs.
    m_displ = disp;
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChTrackTestRig::LogConstraintViolations() {
    GetLog().SetNumFormat("%16.4e");

    //// TODO

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackTestRig::AddVisualize_post(ChSharedPtr<ChBody> post_body,
                                            ChSharedPtr<ChBody> chassis_body,
                                            double height,
                                            double rad,
                                            const ChColor& color) {
    // Platform (on post body)
    ChSharedPtr<ChCylinderShape> base_cyl(new ChCylinderShape);
    base_cyl->GetCylinderGeometry().rad = rad;
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, height / 2.0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -height / 2.0);
    post_body->AddAsset(base_cyl);

    ChSharedPtr<ChColorAsset> col(new ChColorAsset);
    col->SetColor(color);
    post_body->AddAsset(col);

    // Piston (on post body)
    ChSharedPtr<ChCylinderShape> piston(new ChCylinderShape);
    piston->GetCylinderGeometry().rad = rad / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -height / 2.0);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -height * 12.0);
    post_body->AddAsset(piston);  // add asset to post body

    // Post sleve (on chassis/ground body)
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    cyl->GetCylinderGeometry().rad = rad / 4.0;
    cyl->GetCylinderGeometry().p1 = post_body->GetPos() - ChVector<>(0, 0, 8 * height);
    cyl->GetCylinderGeometry().p2 = post_body->GetPos() - ChVector<>(0, 0, 16 * height);
    chassis_body->AddAsset(cyl);
}

}  // end namespace vehicle
}  // end namespace chrono
