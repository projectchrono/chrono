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
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"

#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Definition of a chassis for a track test rig
// -----------------------------------------------------------------------------
class ChTrackTestRigChassis : public ChRigidChassis {
  public:
    ChTrackTestRigChassis() : ChRigidChassis("Ground") {}
    virtual double GetBodyMass() const override { return 1; }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFrame<>(); }
    virtual ChMatrix33<> GetBodyInertia() const override { return ChMatrix33<>(1); }
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return ChCoordsys<>(); }
};

// -----------------------------------------------------------------------------

ChTrackTestRig::ChTrackTestRig(const std::string& filename,
                               bool create_track,
                               ChContactMethod contact_method,
                               bool detracking_control)
    : ChVehicle("TrackTestRig", contact_method),
      m_ride_height(-1),
      m_max_torque(0),
      m_displ_limit(0),
      m_displ_delay(0),
      m_driver_logfile(""),
      m_collide_flags(0xFFFF),
      m_vis_sprocket(VisualizationType::NONE),
      m_vis_idler(VisualizationType::NONE),
      m_vis_suspension(VisualizationType::NONE),
      m_vis_idlerwheel(VisualizationType::NONE),
      m_vis_roadwheel(VisualizationType::NONE),
      m_vis_shoe(VisualizationType::NONE),
      m_plot_output(false),
      m_plot_output_step(0),
      m_next_plot_output_time(0),
      m_csv(nullptr) {
    // Open and parse the input file (track assembly JSON specification file)
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    // Read top-level data
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackAssembly") == 0);
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    assert(d.HasMember("Name"));

    // Create the track assembly from the specified JSON file.
    if (subtype.compare("TrackAssemblySinglePin") == 0) {
        m_track = chrono_types::make_shared<TrackAssemblySinglePin>(d);
    } else if (subtype.compare("TrackAssemblyDoublePin") == 0) {
        m_track = chrono_types::make_shared<TrackAssemblyDoublePin>(d);
    }
    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";

    Create(create_track, detracking_control);
    m_contact_manager = chrono_types::make_shared<ChTrackContactManager>();
}

ChTrackTestRig::ChTrackTestRig(std::shared_ptr<ChTrackAssembly> assembly,
                               bool create_track,
                               ChContactMethod contact_method,
                               bool detracking_control)
    : ChVehicle("TrackTestRig", contact_method),
      m_track(assembly),
      m_ride_height(-1),
      m_max_torque(0),
      m_displ_limit(0),
      m_displ_delay(0),
      m_driver_logfile(""),
      m_collide_flags(0xFFFF),
      m_vis_sprocket(VisualizationType::NONE),
      m_vis_idler(VisualizationType::NONE),
      m_vis_suspension(VisualizationType::NONE),
      m_vis_idlerwheel(VisualizationType::NONE),
      m_vis_roadwheel(VisualizationType::NONE),
      m_vis_shoe(VisualizationType::NONE),
      m_plot_output(false),
      m_plot_output_step(0),
      m_next_plot_output_time(0),
      m_csv(nullptr) {
    Create(create_track, detracking_control);
    m_contact_manager = chrono_types::make_shared<ChTrackContactManager>();
}

ChTrackTestRig::~ChTrackTestRig() {
    delete m_csv;
}

void ChTrackTestRig::Create(bool create_track, bool detracking_control) {
    // Create a contact material for the posts (shared)
    //// TODO: are default material properties ok?
    ChContactMaterialData minfo;
    auto post_mat = minfo.CreateMaterial(m_system->GetContactMethod());

    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<ChTrackTestRigChassis>();
    m_chassis->Initialize(m_system, ChCoordsys<>(), 0);
    m_chassis->SetFixed(true);

    // Disable detracking control if requested
    if (!detracking_control)
        m_track->GetSprocket()->DisableLateralContact();

    // Initialize the track assembly subsystem
    m_track->Initialize(m_chassis, ChVector<>(0, 0, 0), create_track);

    // Create and initialize the shaker post body
    auto num_wheels = m_track->GetNumTrackSuspensions();
    double rw_radius = m_track->GetRoadWheel(0)->GetRadius();

    m_post_hheight = 0.05;
    m_post_radius = 0.9 * rw_radius;

    // Resize vectors
    m_displ_input.resize(num_wheels);

    // Find center height of the lowest road-wheel
    double zmin = 100;
    for (size_t i = 0; i < num_wheels; ++i) {
        if (m_track->GetRoadWheel(i)->GetBody()->GetPos().z() < zmin)
            zmin = m_track->GetRoadWheel(i)->GetBody()->GetPos().z();
    }
    zmin -= create_track ? (rw_radius + m_track->GetTrackShoe(0)->GetHeight() + 0.2) : rw_radius;

    // Create posts and associated actuators under each road wheel
    for (size_t i = 0; i < num_wheels; ++i) {
        auto post_pos = m_track->GetRoadWheel(i)->GetBody()->GetPos();
        post_pos.z() = zmin;

        auto post = std::shared_ptr<ChBody>(m_system->NewBody());
        post->SetPos(post_pos);
        post->SetMass(100);
        post->SetCollide(true);
        m_system->Add(post);

        post->GetCollisionModel()->ClearModel();
        post->GetCollisionModel()->AddCylinder(post_mat, m_post_radius, m_post_radius, m_post_hheight,
                                               ChVector<>(0, 0, -m_post_hheight),
                                               ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
        post->GetCollisionModel()->BuildModel();

        AddPostVisualization(post, m_chassis->GetBody(), ChColor(0.1f, 0.8f, 0.15f));

        auto linact = chrono_types::make_shared<ChLinkMotorLinearPosition>();
        linact->SetNameString("post_actuator");
        linact->SetMotionFunction(chrono_types::make_shared<ChFunction_Setpoint>());
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
    if (m_ride_height > 0) {
        m_displ_offset = -m_ride_height - m_post[LEFT]->GetPos().z();
    }

    // Set visualization modes
    m_track->SetSprocketVisualizationType(m_vis_sprocket);
    m_track->SetIdlerVisualizationType(m_vis_idler);
    m_track->SetSuspensionVisualizationType(m_vis_suspension);
    m_track->SetIdlerWheelVisualizationType(m_vis_idlerwheel);
    m_track->SetRoadWheelVisualizationType(m_vis_roadwheel);
    m_track->SetTrackShoeVisualizationType(m_vis_shoe);

    // Set collisions
    m_track->GetIdlerWheel()->SetCollide((m_collide_flags & static_cast<int>(TrackedCollisionFlag::IDLER_LEFT)) != 0);

    m_track->GetSprocket()->SetCollide((m_collide_flags & static_cast<int>(TrackedCollisionFlag::SPROCKET_LEFT)) != 0);

    bool collide_wheels = (m_collide_flags & static_cast<int>(TrackedCollisionFlag::WHEELS_LEFT)) != 0;
    for (size_t i = 0; i < m_track->GetNumTrackSuspensions(); ++i)
        m_track->GetRoadWheel(i)->SetCollide(collide_wheels);

    bool collide_shoes = (m_collide_flags & static_cast<int>(TrackedCollisionFlag::SHOES_LEFT)) != 0;
    for (size_t i = 0; i < m_track->GetNumTrackShoes(); ++i)
        m_track->GetTrackShoe(i)->SetCollide(collide_shoes);

    // Post locations (in X direction)
    auto idler_x = m_track->GetIdlerWheel()->GetBody()->GetPos().x();
    bool front_sprocket = m_track->GetSprocket()->GetGearBody()->GetPos().x() > idler_x;
    std::vector<double> locations;
    for (int i = 0; i < m_post.size(); i++) {
        auto loc = front_sprocket ? m_post[i]->GetPos().x() : m_post[i]->GetPos().x() - idler_x;
        locations.push_back(loc);
    }

    // Initialize the driver system
    m_driver->SetTimeDelay(m_displ_delay);
    m_driver->Initialize(m_post.size(), locations);
    m_driver->LogInit(m_driver_logfile);

    // Invoke base class method
    ChVehicle::Initialize(ChCoordsys<>(), 0.0);
}

// -----------------------------------------------------------------------------

void ChTrackTestRig::SetDriver(std::shared_ptr<ChTrackTestRigDriver> driver) {
    m_driver = std::move(driver);
}

void ChTrackTestRig::SetPostCollide(bool flag) {
    for (auto& p : m_post)
        p->SetCollide(flag);
}

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

void ChTrackTestRig::Advance(double step) {
    double time = GetChTime();

    // Driver inputs
    std::vector<double> displ(m_post.size(), 0.0);
    std::vector<double> displ_speed(m_post.size(), 0.0);
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
            double displ_input_speed = m_driver->GetDisplacementSpeed(i);
            displ[i] = m_displ_offset + m_displ_limit * m_displ_input[i];
            displ_speed[i] = m_displ_limit * displ_input_speed;
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
        auto func = std::static_pointer_cast<ChFunction_Setpoint>(m_post_linact[i]->GetMotionFunction());
        func->SetSetpointAndDerivatives(displ[i], displ_speed[i], 0.0);
    }

    // Advance state of entire system
    ChVehicle::Advance(step);

    // Process contacts.
    m_contact_manager->Process(this);

    // Generate output for plotting if requested
    time = GetChTime();
    if (!m_driver->Started()) {
        m_next_plot_output_time = time + m_plot_output_step;
    } else if (m_plot_output && time > m_next_plot_output_time) {
        CollectPlotData(time);
        m_next_plot_output_time += m_plot_output_step;
    }
}

// -----------------------------------------------------------------------------

double ChTrackTestRig::GetMass() const {
    return m_track->GetMass();
}

// -----------------------------------------------------------------------------
// Log driver inputs
// -----------------------------------------------------------------------------
void ChTrackTestRig::LogDriverInputs() {
    m_driver->Log(GetChTime());
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

void ChTrackTestRig::AddPostVisualization(std::shared_ptr<ChBody> post,
                                          std::shared_ptr<ChBody> chassis,
                                          const ChColor& color) {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor({color.R, color.G, color.B});

    // Platform (on post body)
    auto base_cyl = chrono_types::make_shared<ChCylinderShape>();
    base_cyl->GetCylinderGeometry().rad = m_post_radius;
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -2 * m_post_hheight);
    base_cyl->AddMaterial(mat);
    post->AddVisualShape(base_cyl);

    // Piston (on post body)
    auto piston = chrono_types::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = m_post_radius / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -2 * m_post_hheight);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -30 * m_post_hheight);
    piston->AddMaterial(mat);
    post->AddVisualShape(piston);

    // Post sleeve (on chassis/ground body)
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = m_post_radius / 4.0;
    cyl->GetCylinderGeometry().p1 = post->GetPos() - ChVector<>(0, 0, 16 * m_post_hheight);
    cyl->GetCylinderGeometry().p2 = post->GetPos() - ChVector<>(0, 0, 32 * m_post_hheight);
    cyl->AddMaterial(mat);
    chassis->AddVisualShape(cyl);
}

// -----------------------------------------------------------------------------

void ChTrackTestRig::SetTrackAssemblyOutput(bool state) {
    m_track->SetOutput(state);
}

void ChTrackTestRig::Output(int frame, ChVehicleOutput& database) const {
    database.WriteTime(frame, m_system->GetChTime());

    if (m_track->OutputEnabled()) {
        m_track->Output(database);
    }
}

void ChTrackTestRig::SetPlotOutput(double output_step) {
    m_plot_output = true;
    m_plot_output_step = output_step;
    m_csv = new utils::CSV_writer(" ");
}

void ChTrackTestRig::CollectPlotData(double time) {
    *m_csv << time;

    ////const ChFrameMoving<>& c_ref = GetChassisBody()->GetFrame_REF_to_abs();
    ////const ChVector<>& i_pos_abs = m_track->GetIdler()->GetWheelBody()->GetPos();
    ////const ChVector<>& s_pos_abs = m_track->GetSprocket()->GetGearBody()->GetPos();
    ////ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
    ////ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);

    *m_csv << m_track->GetSprocket()->GetGearBody()->GetPos();
    *m_csv << m_track->GetIdler()->GetWheelBody()->GetPos();

    for (auto suspension : m_track->GetTrackSuspensions()) {
        *m_csv << suspension->GetRoadWheel()->GetBody()->GetPos();
        //// TODO: spring and shock forces
    }

    *m_csv << std::endl;
}

void ChTrackTestRig::PlotOutput(const std::string& out_dir, const std::string& out_name) {
    if (!m_plot_output)
        return;

    std::string out_file = out_dir + "/" + out_name + ".txt";
    m_csv->write_to_file(out_file);

#ifdef CHRONO_POSTPROCESS
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    std::string title = "Suspension test rig - Wheel positions";
    mplot.OutputWindow(0);
    mplot.SetTitle(title.c_str());
    mplot.SetLabelX("time [s]");
    mplot.SetLabelY("wheel z [m]");
    mplot.SetCommand("set format y '%4.1e'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.Plot(out_file.c_str(), 1, 4, "sprocket", " with lines lw 2");
    mplot.Plot(out_file.c_str(), 1, 7, "idler", " with lines lw 2");
    for (int i = 0; i < m_track->GetNumTrackSuspensions(); i++) {
        std::string label = "wheel #" + std::to_string(i);
        mplot.Plot(out_file.c_str(), 1, 7 + 3 * i + 3, label.c_str(), " with lines lw 2");
    }

    //// TODO: spring and shock forces
#endif
}

}  // end namespace vehicle
}  // end namespace chrono
