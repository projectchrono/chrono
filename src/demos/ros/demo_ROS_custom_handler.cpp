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

#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"

#include <chrono>
#include <iostream>

using namespace chrono;
using namespace chrono::ros;

// =============================================================================
// A custom publisher handler. Mirrors the Chrono 9.0 demo's MyCustomHandler
// (publishes an incrementing Int64 on "~/my_topic" at 1 Hz) — the difference
// is purely that you address the type by name and never touch rclcpp:
//   9.0:  interface->GetNode()->create_publisher<std_msgs::msg::Int64>(topic, 1);
//   now:  bridge.CreatePublisher(topic, "std_msgs/msg/Int64");
class MyCustomHandler : public ChROSHandler {
  public:
    MyCustomHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic), m_ticker(0) {}

    bool Initialize(ChROSBridge& bridge) override {
        std::cout << "Creating publisher for topic " << m_topic << " ..." << std::endl;
        m_publisher = bridge.CreatePublisher(m_topic, "std_msgs/msg/Int64");
        return true;
    }

    void Tick(double time) override {
        std::cout << "Publishing " << m_ticker << " ..." << std::endl;
        auto msg = m_publisher->NewMessage();
        msg.SetInt("data", m_ticker);
        m_publisher->Publish(msg);
        m_ticker++;
    }

  private:
    const std::string m_topic;
    std::shared_ptr<ChROSPublisher> m_publisher;
    int64_t m_ticker;
};

// =============================================================================
// New in the schema-driven design: custom SUBSCRIBERS are just as easy (the
// 9.0 demo had no custom subscriber). Receives a force command from ROS and
// applies it to the box.
// A subscriber handler: receive commands from ROS, apply them to Chrono.
// The callback fires inside ChROSManager::Update() on this thread, so writing
// to the Chrono system is safe.
class ForceCommandHandler : public ChROSHandler {
  public:
    explicit ForceCommandHandler(std::shared_ptr<ChBody> box) : ChROSHandler(0 /*every step*/), m_box(box) {}

    bool Initialize(ChROSBridge& bridge) override {
        // One force/torque accumulator on the box, cleared and refilled each step.
        m_force_accumulator = m_box->AddAccumulator();

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
        m_box->EmptyAccumulator(m_force_accumulator);
        m_box->AccumulateForce(m_force_accumulator, m_force, m_box->GetPos(), false);
    }

  private:
    std::shared_ptr<ChBody> m_box;
    std::shared_ptr<ChROSSubscription> m_subscription;
    unsigned int m_force_accumulator = 0;
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

    // Create the ROS manager and register the handlers. The first three are
    // built-in handlers, registered exactly as in the Chrono 9.0 demo (same
    // call interfaces): the clock (publishes /clock every step), the box state
    // (pose/twist/accel at 25 Hz under ~/box), and tf (at 100 Hz).
    auto ros_manager = chrono_types::make_shared<ChROSManager>("demo");
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSBodyHandler>(25, box, "~/box"));
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSTFHandler>(100));
    // ... and the two application handlers: a custom Int64 publisher and the
    // force-command subscriber (the latter is new to the schema-driven design).
    ros_manager->RegisterHandler(chrono_types::make_shared<MyCustomHandler>("~/my_topic"));
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
