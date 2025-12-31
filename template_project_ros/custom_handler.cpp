// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Demo to show how to create a custom Chrono::ROS handler
//
// =============================================================================

#include "chrono/core/ChTypes.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"

#include <std_msgs/msg/int64.hpp>

#include <chrono>

using namespace chrono;
using namespace chrono::ros;

// =============================================================================

class MyCustomHandler : public ChROSHandler {
  public:
    MyCustomHandler(const std::string& topic) : ChROSHandler(1), m_topic(topic), m_ticker(0) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override {
        std::cout << "Creating publisher for topic " << m_topic << " ..." << std::endl;
        m_publisher = interface->GetNode()->create_publisher<std_msgs::msg::Int64>(m_topic, 1);
        return true;
    }

    virtual std::vector<uint8_t> GetSerializedData(double time) override {
        std::vector<uint8_t> data(sizeof(int64_t));
        std::memcpy(data.data(), &m_ticker, sizeof(int64_t));
        m_ticker++;
        return data;
    }

    virtual void PublishFromSerialized(const std::vector<uint8_t>& data, 
                                       std::shared_ptr<ChROSInterface> interface) override {
        if (data.size() != sizeof(int64_t)) return;
        int64_t val;
        std::memcpy(&val, data.data(), sizeof(int64_t));
        
        std::cout << "Publishing " << val << " ..." << std::endl;
        std_msgs::msg::Int64 msg;
        msg.data = val;
        m_publisher->publish(msg);
    }

  private:
    const std::string m_topic;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr m_publisher;
    int m_ticker;
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // Create the system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration({0, 0, -9.81});

    // Add a mesh object to make the scene interesting
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

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create a handler that will publish the state of the box
    // It will publish at a rate of 25Hz on the topic "~/box"
    auto box_handler = chrono_types::make_shared<ChROSBodyHandler>(25, box, "~/box");
    ros_manager->RegisterHandler(box_handler);

    // Create a TF handler that will publish the transform of the box relative to the floor
    auto tf_handler = chrono_types::make_shared<ChROSTFHandler>(100);
    tf_handler->AddTransform(floor, floor->GetName(), box, box->GetName());
    ros_manager->RegisterHandler(tf_handler);

    // Create a custom handler
    auto custom_handler = chrono_types::make_shared<MyCustomHandler>("~/my_topic");
    ros_manager->RegisterHandler(custom_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    // ------------

    // Simulation
    double time = 0;
    double step_size = 2e-3;
    double time_end = 1000;

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    while (time < time_end) {
        time = sys.GetChTime();

        // Updates
        if (!ros_manager->Update(time, step_size))
            break;

        sys.DoStepDynamics(step_size);

        realtime_timer.Spin(step_size);
    }

    return 0;
}
