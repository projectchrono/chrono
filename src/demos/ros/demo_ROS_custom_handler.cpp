// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Demo: how to create a custom Chrono::ROS handler.
//
// Chrono::ROS is schema-driven: a handler addresses any installed ROS 2 message
// type by its type-name string and sets fields by name - no bridge code, no
// recompiling Chrono, no rclcpp. The custom handler below publishes an
// incrementing Int64 at 1 Hz on "~/my_topic".
//
//   ros2 topic echo /chrono_ros_node/my_topic
//
// =============================================================================

#include "chrono/core/ChTypes.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"

#include <iostream>

using namespace chrono;
using namespace chrono::ros;

// =============================================================================

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

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // Create the system.
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

    // Create the ROS manager and register the handlers.
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // The clock automatically publishes on every tick on topic /clock.
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());

    // Publish the state of the box at 25 Hz on the topic "~/box".
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSBodyHandler>(25, box, "~/box"));

    // Publish the transform of the box relative to the floor.
    auto tf_handler = chrono_types::make_shared<ChROSTFHandler>(100);
    tf_handler->AddTransform(floor, floor->GetName(), box, box->GetName());
    ros_manager->RegisterHandler(tf_handler);

    // The custom handler.
    ros_manager->RegisterHandler(chrono_types::make_shared<MyCustomHandler>("~/my_topic"));

    ros_manager->Initialize();

    // ------------

    // Simulation loop.
    double time = 0;
    const double step_size = 2e-3;
    const double time_end = 1000;

    ChRealtimeStepTimer realtime_timer;
    while (time < time_end) {
        time = sys.GetChTime();

        if (!ros_manager->Update(time, step_size))
            break;

        sys.DoStepDynamics(step_size);
        realtime_timer.Spin(step_size);
    }

    return 0;
}
