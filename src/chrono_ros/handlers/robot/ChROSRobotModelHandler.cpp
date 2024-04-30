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
// Authors: Aaron Young
// =============================================================================
//
// Handler responsible for publishing a Robot Model to be visualized in RViz
//
// =============================================================================

#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <tinyxml2.h>
#include <fstream>

namespace chrono {
namespace ros {

ChROSRobotModelHandler::ChROSRobotModelHandler(const std::string& robot_model, const std::string& topic_name)
    : ChROSHandler(std::numeric_limits<double>::max()), m_robot_model(robot_model), m_topic_name(topic_name) {}

#ifdef CHRONO_PARSERS_URDF
class CustomProcessorFilenameResolver : public chrono::parsers::ChParserURDF::CustomProcessor {
  public:
    CustomProcessorFilenameResolver(const std::string& filename)
        : m_filepath(filesystem::path(filename).parent_path().str()) {}

    /// This method will parse the links and resolve each filename to be an absolute path.
    virtual void Process(tinyxml2::XMLElement& elem, ChSystem& system) override {
        // filename exists in visual/geometry/mesh, visual/material, and collision/geometry/mesh
        for (tinyxml2::XMLElement* vis = elem.FirstChildElement("visual");  //
             vis;                                                           //
             vis = elem.NextSiblingElement("visual")                        //
        ) {
            // Geometry is required
            tinyxml2::XMLElement* geom = vis->FirstChildElement("geometry");
            if (!ParseGeometry(geom)) {
                std::cerr << "Failed to parse visual/geometry." << std::endl;
                return;
            }

            // Material is optional
            if (tinyxml2::XMLElement* mat = vis->FirstChildElement("material")) {
                if (!ParseMaterial(mat)) {
                    std::cerr << "Failed to parse visual/material." << std::endl;
                    return;
                }
            }
        }

        for (tinyxml2::XMLElement* col = elem.FirstChildElement("collision");  //
             col;                                                              //
             col = elem.NextSiblingElement("collision")                        //
        ) {
            // Geometry is required
            tinyxml2::XMLElement* geom = col->FirstChildElement("geometry");
            if (!ParseGeometry(geom)) {
                std::cerr << "Failed to parse collision/geometry." << std::endl;
                return;
            }
        }
    }

  private:
    bool ParseGeometry(tinyxml2::XMLElement* geom) {
        if (!geom || !geom->FirstChildElement())
            return false;

        if (tinyxml2::XMLElement* shape = geom->FirstChildElement(); std::string(shape->Value()) == "mesh") {
            shape->SetAttribute("filename", ResolveFilename(shape->Attribute("filename")).c_str());
        }

        return true;
    }

    bool ParseMaterial(tinyxml2::XMLElement* mat) {
        if (tinyxml2::XMLElement* tex = mat->FirstChildElement("texture")) {
            tex->SetAttribute("filename", ResolveFilename(tex->Attribute("filename")).c_str());
        }

        return true;
    }

    /// rviz2 expects URI. Convert all the url filenames (i.e. don't have ://) to an absolute path prefixed with
    /// file://.
    std::string ResolveFilename(const std::string& filename) {
        std::string resolved_filename = filename;

        const std::string separator("://");
        const size_t pos_separator = filename.find(separator);
        if (pos_separator == std::string::npos) {
            filesystem::path path(filename);
            if (!path.is_absolute())
                path = (filesystem::path(m_filepath) / path).make_absolute();
            else
                path = filesystem::path(filename);
            resolved_filename = "file://" + path.str();
        }

        return resolved_filename;
    }

  private:
    const std::string m_filepath;
};

ChROSRobotModelHandler::ChROSRobotModelHandler(chrono::parsers::ChParserURDF& parser, const std::string& topic_name)
    : ChROSHandler(std::numeric_limits<double>::max()), m_topic_name(topic_name) {
    auto filename = parser.GetFilename();
    auto xml = parser.CustomProcess("link", std::make_shared<CustomProcessorFilenameResolver>(filename));
    if (!xml) {
        std::cerr << "Failed to parse RobotModel file: " << filename << std::endl;
        return;
    }

    tinyxml2::XMLPrinter printer;
    xml->Print(&printer);
    m_robot_model = printer.CStr();
}
#endif

bool ChROSRobotModelHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    // rviz expects the QoS to use TRANSIENT_LOCAL.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    m_publisher = node->create_publisher<std_msgs::msg::String>(m_topic_name, qos);

    m_msg.data = m_robot_model;

    return true;
}

void ChROSRobotModelHandler::Tick(double time) {
    m_publisher->publish(m_msg);
}

}  // namespace ros
}  // namespace chrono
