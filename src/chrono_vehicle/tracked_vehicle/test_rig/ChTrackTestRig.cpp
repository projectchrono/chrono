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
// Definition of a track testing mechanism (as a vehicle).
// The tested track can be specified through a stand-alone JSON file or as a
// specified track assembly in a vehicle JSON specification file.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Definition of a chassis for a track test rig
// -----------------------------------------------------------------------------
class ChTrackTestRigChassis : public ChRigidChassis {
  public:
    ChTrackTestRigChassis();
    virtual double GetMass() const override { return m_mass; }
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

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

ChTrackTestRigChassis::ChTrackTestRigChassis() : ChRigidChassis("Ground") {
    m_inertia = ChMatrix33<>(m_inertiaXX);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackTestRig::ChTrackTestRig(const std::string& filename,
                               bool create_track,
                               ChMaterialSurface::ContactMethod contact_method)
    : ChVehicle("TrackTestRig", contact_method),
      m_create_track(create_track),
      m_ride_height(-1),
      m_max_torque(0),
      m_collide_flags(0xFFFF),
      m_vis_sprocket(VisualizationType::NONE),
      m_vis_idler(VisualizationType::NONE),
      m_vis_roadwheel_assembly(VisualizationType::NONE),
      m_vis_roadwheel(VisualizationType::NONE),
      m_vis_shoe(VisualizationType::NONE) {
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
    } else if (subtype.compare("TrackAssemblyDoublePin") == 0) {
        m_track = std::make_shared<TrackAssemblyDoublePin>(d);
    }
    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";

    Create();
}

ChTrackTestRig::ChTrackTestRig(std::shared_ptr<ChTrackAssembly> assembly,
                               bool create_track,
                               ChMaterialSurface::ContactMethod contact_method)
    : ChVehicle("TrackTestRig", contact_method),
      m_create_track(create_track),
      m_track(assembly),
      m_ride_height(-1),
      m_max_torque(0),
      m_collide_flags(0xFFFF),
      m_vis_sprocket(VisualizationType::NONE),
      m_vis_idler(VisualizationType::NONE),
      m_vis_roadwheel_assembly(VisualizationType::NONE),
      m_vis_roadwheel(VisualizationType::NONE),
      m_vis_shoe(VisualizationType::NONE) {
    Create();
}

void ChTrackTestRig::Create() {
    // Create the chassis subsystem
    m_chassis = std::make_shared<ChTrackTestRigChassis>();
    m_chassis->Initialize(m_system, ChCoordsys<>(), 0);
    m_chassis->SetFixed(true);

    // Initialize the track assembly subsystem
    m_track->Initialize(m_chassis->GetBody(), ChVector<>(0, 0, 0), m_create_track);

    // Create and initialize the shaker post body
    auto num_wheels = m_track->GetNumRoadWheelAssemblies();
    double rw_radius = m_track->GetRoadWheel(0)->GetWheelRadius();

    m_post_hheight = 0.05;
    m_post_radius = 0.9 * rw_radius;

    // Resize vectors
    m_displ_input.resize(num_wheels);

    // Find center height of the lowest road-wheel
    double zmin = 100;
    for (size_t i = 0; i < num_wheels; ++i) {
        if (m_track->GetRoadWheel(i)->GetWheelBody()->GetPos().z() < zmin)
            zmin = m_track->GetRoadWheel(i)->GetWheelBody()->GetPos().z();
    }
    zmin -= m_create_track ? (rw_radius + m_track->GetTrackShoe(0)->GetHeight() + 0.2) : rw_radius;

    // Create posts and associated actuators under each road wheel
    for (size_t i = 0; i < num_wheels; ++i) {
        auto post_pos = m_track->GetRoadWheel(i)->GetWheelBody()->GetPos();
        post_pos.z() = zmin;

        auto post = std::shared_ptr<ChBody>(m_system->NewBody());
        post->SetPos(post_pos);
        post->SetMass(100);
        post->SetCollide(true);
        m_system->Add(post);

        post->GetCollisionModel()->ClearModel();
        post->GetCollisionModel()->AddCylinder(m_post_radius, m_post_radius, m_post_hheight,
                                               ChVector<>(0, 0, -m_post_hheight),
                                               ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
        post->GetCollisionModel()->BuildModel();

        AddPostVisualization(post, m_chassis->GetBody(), ChColor(0.1f, 0.8f, 0.15f));

        auto linact = std::make_shared<ChLinkMotorLinearPosition>();
        linact->SetNameString("post_actuator");
        linact->SetMotionFunction(std::make_shared<ChFunction_Const>());
        linact->Initialize(m_chassis->GetBody(), post, ChFrame<>(ChVector<>(post_pos), Q_from_AngY(CH_C_PI_2)));
        m_system->AddLink(linact);

        m_post.push_back(post);
        m_post_linact.push_back(linact);
    }
}

void ChTrackTestRig::Initialize() {
    if (!m_driver) {
        throw ChException("No driver system provided");
    }

    // Calculate post displacement offset (if any) to set reference position at specified ride height
    m_displ_offset = 0;
    m_displ_delay = 0;
    if (m_ride_height > 0) {
        m_displ_offset = -m_ride_height - m_post[LEFT]->GetPos().z();
        m_displ_delay = 0.2;
    }

    // Set visualization modes
    m_track->SetSprocketVisualizationType(m_vis_sprocket);
    m_track->SetIdlerVisualizationType(m_vis_idler);
    m_track->SetRoadWheelAssemblyVisualizationType(m_vis_roadwheel_assembly);
    m_track->SetRoadWheelVisualizationType(m_vis_roadwheel);
    m_track->SetTrackShoeVisualizationType(m_vis_shoe);

    // Set collisions
    m_track->GetIdler()->SetCollide((m_collide_flags & static_cast<int>(TrackedCollisionFlag::IDLER_LEFT)) != 0);

    m_track->GetSprocket()->SetCollide((m_collide_flags & static_cast<int>(TrackedCollisionFlag::SPROCKET_LEFT)) != 0);

    bool collide_wheels = (m_collide_flags & static_cast<int>(TrackedCollisionFlag::WHEELS_LEFT)) != 0;
    for (size_t i = 0; i < m_track->GetNumRoadWheelAssemblies(); ++i)
        m_track->GetRoadWheel(i)->SetCollide(collide_wheels);

    bool collide_shoes = (m_collide_flags & static_cast<int>(TrackedCollisionFlag::SHOES_LEFT)) != 0;
    for (size_t i = 0; i < m_track->GetNumTrackShoes(); ++i)
        m_track->GetTrackShoe(i)->SetCollide(collide_shoes);
 
    // Initialize the driver system
    m_driver->SetTimeDelay(m_displ_delay);
    m_driver->Initialize(m_post.size());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackTestRig::SetDriver(std::unique_ptr<ChDriverTTR> driver) {
    m_driver = std::move(driver);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackTestRig::GetActuatorDisp(int index) {
    double time = GetSystem()->GetChTime();
    return m_post_linact[index]->GetMotionFunction()->Get_y(time);
}

double ChTrackTestRig::GetActuatorForce(int index) {
    return m_post_linact[index]->Get_react_force().x();
}

double ChTrackTestRig::GetActuatorMarkerDist(int index) {
    return m_post_linact[index]->GetMotorPos();
}

double ChTrackTestRig::GetRideHeight() const {
    // Note: the track assembly reference frame is constructed at a height of 0.
    //// TODO: what is a meaningful quantity here?!?!
    return -m_post[0]->GetPos().z();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackTestRig::Advance(double step) {
    double time = GetChTime();

    m_displ_limit = 0.2;

    // Driver inputs
    std::vector<double> displ(m_post.size(), 0.0);
    if (time < m_displ_delay) {
        m_throttle_input = 0;
        for (int i = 0; i < m_post.size(); i++) {
            m_displ_input[i] = 0;
            displ[i] = m_displ_offset * time / m_displ_delay;
        }
    } else {
        m_throttle_input = m_driver->GetThrottle();
        for (int i = 0; i < m_post.size(); i++) {
            m_displ_input[i] = m_driver->GetDisplacement(i);
            displ[i] = m_displ_offset + m_displ_limit * m_displ_input[i];
        }
    }

    m_throttle_input = m_driver->GetThrottle();
    for (int i = 0; i < m_post.size(); i++) {
        m_displ_input[i] = m_driver->GetDisplacement(i);
    }

    // Synchronize driver system
    m_driver->Synchronize(time);

    // Apply a torque to the sprocket's shaft
    m_track->GetSprocket()->GetAxle()->SetAppliedTorque(-m_max_torque * m_throttle_input);

    // Update post displacements
    for (int i = 0; i < m_post.size(); i++) {
        auto func = std::static_pointer_cast<ChFunction_Const>(m_post_linact[i]->GetMotionFunction());
        func->Set_yconst(displ[i]);
    }

    // Advance state of entire system
    ChVehicle::Advance(step);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackTestRig::GetMass() const {
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
void ChTrackTestRig::AddPostVisualization(std::shared_ptr<ChBody> post,
                                       std::shared_ptr<ChBody> chassis,
                                       const ChColor& color) {
    // Platform (on post body)
    auto base_cyl = std::make_shared<ChCylinderShape>();
    base_cyl->GetCylinderGeometry().rad = m_post_radius;
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -2 * m_post_hheight);
    post->AddAsset(base_cyl);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    post->AddAsset(col);

    // Piston (on post body)
    auto piston = std::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = m_post_radius / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -2 * m_post_hheight);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -30 * m_post_hheight);
    post->AddAsset(piston);

    // Post sleeve (on chassis/ground body)
    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = m_post_radius / 4.0;
    cyl->GetCylinderGeometry().p1 = post->GetPos() - ChVector<>(0, 0, 16 * m_post_hheight);
    cyl->GetCylinderGeometry().p2 = post->GetPos() - ChVector<>(0, 0, 32 * m_post_hheight);
    chassis->AddAsset(cyl);
}

}  // end namespace vehicle
}  // end namespace chrono
