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
#include <cmath>
#include <algorithm>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackTestRig.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Defintion of a chassis for a track test rig
// -----------------------------------------------------------------------------
class ChTrackTestRigChassis : public ChChassis {
  public:
    ChTrackTestRigChassis();
    virtual double GetMass() const override { return m_mass; }
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_COM_loc;
    static const ChCoordsys<> m_driverCsys;
};

const double ChTrackTestRigChassis::m_mass = 1;
const ChVector<> ChTrackTestRigChassis::m_inertiaXX(1, 1, 1);
const ChVector<> ChTrackTestRigChassis::m_COM_loc(0, 0, 0);
const ChCoordsys<> ChTrackTestRigChassis::m_driverCsys(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

ChTrackTestRigChassis::ChTrackTestRigChassis() : ChChassis("Ground") {
    m_inertia = ChMatrix33<>(m_inertiaXX);
}

void ChTrackTestRigChassis::AddVisualizationAssets(VisualizationType vis) {
    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(0.1, 0.1, 0.1));
    m_body->AddAsset(box);

    auto blue = std::make_shared<ChColorAsset>();
    blue->SetColor(ChColor(0.2f, 0.2f, 0.8f));
    m_body->AddAsset(blue);
}

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

ChTrackTestRig::ChTrackTestRig(const std::string& filename,
                               const ChVector<>& location,
                               ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method), m_location(location), m_max_torque(0) {
    // Open and parse the input file (track assembly JSON specification file)
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackAssembly") == 0);
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    assert(d.HasMember("Name"));

    // Create the track assembly from the specified JSON file.
    if (subtype.compare("TrackAssemblySinglePin") == 0) {
        m_track = std::make_shared<TrackAssemblySinglePin>(d);
    }
    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChTrackTestRig::ChTrackTestRig(std::shared_ptr<ChTrackAssembly> assembly,
                               const ChVector<>& location,
                               ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method),
      m_track(assembly),
      m_location(location),
      m_max_torque(0) {
}

void ChTrackTestRig::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // ----------------------------
    // Create the chassis subsystem
    // ----------------------------
    m_chassis = std::make_shared<ChTrackTestRigChassis>();
    m_chassis->Initialize(m_system, chassisPos, 0);
    m_chassis->GetBody()->SetBodyFixed(true);

    // ---------------------------------
    // Initialize the vehicle subsystems
    // ---------------------------------

    m_track->Initialize(m_chassis->GetBody(), m_location);

    // ------------------------------------------
    // Create and initialize the shaker post body
    // ------------------------------------------

    // Find the lowest road-wheel.
    double zmin = 100;
    for (size_t i = 0; i < m_track->GetNumRoadWheelAssemblies(); ++i) {
        if (m_track->GetRoadWheel(i)->GetWheelBody()->GetPos().z() < zmin)
            zmin = m_track->GetRoadWheel(i)->GetWheelBody()->GetPos().z();
    }

    double idler_radius = m_track->GetIdler()->GetWheelRadius();
    double rw_radius = m_track->GetRoadWheel(0)->GetWheelRadius();
    double shoe_height = 0;  //// m_track->GetTrackShoe(0)->GetHeight();

    // Calculate post position under sprocket
    ChVector<> sprocket_pos = m_track->GetSprocket()->GetGearBody()->GetPos();
    ChVector<> idler_pos = m_track->GetIdler()->GetWheelBody()->GetPos();

    double post_height = 0.1;
    double post_width = 0.4;
    double post_length = std::abs(sprocket_pos.x() - idler_pos.x()) + 3 * idler_radius;

    m_post_pos = 0.5 * (sprocket_pos + idler_pos);
    m_post_pos.z() = zmin - (rw_radius + shoe_height + post_height / 2.0);

    m_post = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post->SetPos(m_post_pos);
    m_system->Add(m_post);
    ////AddVisualize_post(m_post, m_chassis, post_length, post_width, post_height, ChColor(0.1f, 0.8f, 0.15f));

    // ------------------------------------------
    // Create and initialize joints and actuators
    // ------------------------------------------

    // Prismatic joint to force vertical translation
    m_post_prismatic = std::make_shared<ChLinkLockPrismatic>();
    m_post_prismatic->SetNameString("L_post_prismatic");
    m_post_prismatic->Initialize(m_chassis->GetBody(), m_post, ChCoordsys<>(ChVector<>(m_post_pos), QUNIT));
    m_system->AddLink(m_post_prismatic);

    // Post actuator
    ChVector<> m1 = m_post_pos;
    m1.z() -= 1.0;  // offset marker 1 location 1 meter below marker 2
    m_post_linact = std::make_shared<ChLinkLinActuator>();
    m_post_linact->SetNameString("Post_linActuator");
    m_post_linact->Initialize(m_chassis->GetBody(), m_post, false, ChCoordsys<>(m1, QUNIT), ChCoordsys<>(m_post_pos, QUNIT));
    m_post_linact->Set_lin_offset(1.0);
    auto func = std::make_shared<ChFunction_Const>(0);
    m_post_linact->Set_dist_funct(func);
    m_system->AddLink(m_post_linact);

    /*
    // Constrain in a horizontal plane (based on current post location)
    m_post_ptPlane = std::make_shared<ChLinkLockPointPlane>();
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
    return m_post_linact->Get_react_force().x();
}

double ChTrackTestRig::GetActuatorMarkerDist() {
    return m_post_linact->GetDist();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackTestRig::Synchronize(double time, double disp, double throttle, const TrackShoeForces& shoe_forces) {
    // Apply the displacements to the left/right post actuators
    if (auto func = std::dynamic_pointer_cast<ChFunction_Const>(m_post_linact->Get_dist_funct()))
        func->Set_yconst(disp);

    // Apply a torque to the sprocket's shaft
    m_track->GetSprocket()->GetAxle()->SetAppliedTorque(-m_max_torque * throttle);


    // Apply contact forces to track shoe bodies.

    //// TODO

    // Cache driver inputs.
    m_displ = disp;
}

// -----------------------------------------------------------------------------
// Override collision flags for various subsystems
// -----------------------------------------------------------------------------
void ChTrackTestRig::SetCollide(int flags) {
    m_track->GetIdler()->SetCollide((flags & static_cast<int>(TrackCollide::IDLER_LEFT)) != 0);

    m_track->GetSprocket()->SetCollide((flags & static_cast<int>(TrackCollide::SPROCKET_LEFT)) != 0);

    bool collide_wheels = (flags & static_cast<int>(TrackCollide::WHEELS_LEFT)) != 0;
    for (size_t i = 0; i < m_track->GetNumRoadWheelAssemblies(); ++i)
        m_track->GetRoadWheel(i)->SetCollide(collide_wheels);

    bool collide_shoes = (flags & static_cast<int>(TrackCollide::SHOES_LEFT)) != 0;
    for (size_t i = 0; i < m_track->GetNumTrackShoes(); ++i)
        m_track->GetTrackShoe(i)->SetCollide(collide_shoes);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackTestRig::GetVehicleMass() const {
    return m_track->GetMass();
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
void ChTrackTestRig::AddVisualize_post(std::shared_ptr<ChBody> post_body,
                                       std::shared_ptr<ChBody> chassis_body,
                                       double length,
                                       double width,
                                       double height,
                                       const ChColor& color) {
    // Platform (on post body)
    auto base_box = std::make_shared<ChBoxShape>();
    base_box->GetBoxGeometry().SetLengths(ChVector<>(length, width, height));
    post_body->AddAsset(base_box);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    post_body->AddAsset(col);

    // Piston (on post body)
    auto piston = std::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = width / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -height / 2.0);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -height * 12.0);
    post_body->AddAsset(piston);  // add asset to post body

    // Post sleve (on chassis/ground body)
    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = width / 4.0;
    cyl->GetCylinderGeometry().p1 = post_body->GetPos() - ChVector<>(0, 0, 8 * height);
    cyl->GetCylinderGeometry().p2 = post_body->GetPos() - ChVector<>(0, 0, 16 * height);
    chassis_body->AddAsset(cyl);
}

}  // end namespace vehicle
}  // end namespace chrono
