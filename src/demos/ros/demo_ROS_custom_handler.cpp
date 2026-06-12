// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Demo: adding custom ROS data pathways to a Chrono simulation.
//
// Chrono::ROS is schema-driven: any installed ROS 2 message type is addressed
// by its type name string, and fields are set by name. Adding a topic never
// requires writing or rebuilding bridge code - everything below lives in this
// application.
//
// This demo publishes the falling box's height (std_msgs/msg/Float64) and
// pose (geometry_msgs/msg/PoseStamped), and subscribes to a force command
// (geometry_msgs/msg/Vector3) that is applied to the box. Try:
//
//   ros2 topic echo /demo/output/pose
//   ros2 topic pub /demo/input/force geometry_msgs/msg/Vector3 "{z: 12000.0}"
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSManager.h"

#include <chrono>
#include <iostream>

using namespace chrono;
using namespace chrono::ros;

// =============================================================================
// A publisher handler: extract state from Chrono, publish at a fixed rate.
class BoxStateHandler : public ChROSHandler {
  public:
    BoxStateHandler(std::shared_ptr<ChBody> box, double rate) : ChROSHandler(rate), m_box(box) {}

    bool Initialize(ChROSBridge& bridge) override {
        m_height_pub = bridge.CreatePublisher("~/output/height", "std_msgs/msg/Float64");
        m_pose_pub = bridge.CreatePublisher("~/output/pose", "geometry_msgs/msg/PoseStamped");

        // Field discovery, straight from the running ROS installation:
        std::cout << "PoseStamped layout:\n" << m_pose_pub->DescribeType() << std::endl;
        return true;
    }

    void Tick(double time) override {
        auto height = m_height_pub->NewMessage();
        height.SetDouble("data", m_box->GetPos().z());
        m_height_pub->Publish(height);

        auto pose = m_pose_pub->NewMessage();
        pose.SetTime("header.stamp", time);
        pose.SetString("header.frame_id", "world");
        pose.SetDouble("pose.position.x", m_box->GetPos().x());
        pose.SetDouble("pose.position.y", m_box->GetPos().y());
        pose.SetDouble("pose.position.z", m_box->GetPos().z());
        const ChQuaternion<> rotation = m_box->GetRot();
        pose.SetDouble("pose.orientation.w", rotation.e0());
        pose.SetDouble("pose.orientation.x", rotation.e1());
        pose.SetDouble("pose.orientation.y", rotation.e2());
        pose.SetDouble("pose.orientation.z", rotation.e3());
        m_pose_pub->Publish(pose);
    }

  private:
    std::shared_ptr<ChBody> m_box;
    std::shared_ptr<ChROSPublisher> m_height_pub;
    std::shared_ptr<ChROSPublisher> m_pose_pub;
};

// =============================================================================
// A subscriber handler: receive commands from ROS, apply them to Chrono.
// The callback fires inside ChROSManager::Update() on this thread, so writing
// to the Chrono system is safe.
class ForceCommandHandler : public ChROSHandler {
  public:
    explicit ForceCommandHandler(std::shared_ptr<ChBody> box) : ChROSHandler(0 /*every step*/), m_box(box) {}

    bool Initialize(ChROSBridge& bridge) override {
        m_subscription = bridge.CreateSubscription(  //
            "~/input/force", "geometry_msgs/msg/Vector3", [this](const ChROSMessageView& message) {
                m_force.x() = message.GetDouble("x");
                m_force.y() = message.GetDouble("y");
                m_force.z() = message.GetDouble("z");
                std::cout << "Received force command: " << m_force << std::endl;
            });
        return true;
    }

    void Tick(double /*time*/) override {
        m_box->EmptyAccumulators();
        m_box->AccumulateForce(m_force, m_box->GetPos(), false);
    }

  private:
    std::shared_ptr<ChBody> m_box;
    std::shared_ptr<ChROSSubscription> m_subscription;
    ChVector3d m_force;
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // Create the system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration({0, 0, -9.81});

    auto phys_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    phys_mat->SetFriction(0.5f);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(20, 20, 1, 1000, true, true, phys_mat);
    floor->SetPos({0, 0, -1});
    floor->SetFixed(true);
    floor->SetName("floor");
    sys.AddBody(floor);

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, true, phys_mat);
    box->SetPos({0, 0, 5});
    box->SetRot(QuatFromAngleAxis(0.2, {1, 0, 0}));
    box->SetName("box");
    sys.AddBody(box);

    // ------------

    // Create the ROS manager and register the custom handlers
    auto ros_manager = chrono_types::make_shared<ChROSManager>("demo");
    ros_manager->RegisterHandler(chrono_types::make_shared<BoxStateHandler>(box, 25 /*Hz*/));
    ros_manager->RegisterHandler(chrono_types::make_shared<ForceCommandHandler>(box));
    ros_manager->Initialize();

    // ------------

    // Simulation loop
    double time = 0;
    const double step_size = 2e-3;
    const double time_end = 1000;

    ChRealtimeStepTimer realtime_timer;
    while (time < time_end) {
        time = sys.GetChTime();

        if (!ros_manager->Update(time, step_size)) {
            std::cerr << "Chrono::ROS bridge node stopped; ending simulation." << std::endl;
            break;
        }

        sys.DoStepDynamics(step_size);
        realtime_timer.Spin(step_size);
    }

    return 0;
}
