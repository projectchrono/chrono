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
// Handler that publishes a robot model (URDF) on /robot_description for RViz.
//
// =============================================================================

#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSQoS.h"

#ifdef CHRONO_HAS_URDF
    #include "chrono/physics/ChSystem.h"
    #include <tinyxml2.h>
    #include <filesystem>
    #include <iostream>
#endif

namespace chrono {
namespace ros {

#ifdef CHRONO_HAS_URDF
// Re-emits the URDF with every mesh/texture filename rewritten to an absolute
// file:// URI, which is what RViz requires. Ported from the Chrono 9.0 handler.
namespace {
class CustomProcessorFilenameResolver : public chrono::parsers::ChParserURDF::CustomProcessor {
  public:
    explicit CustomProcessorFilenameResolver(const std::string& filename)
        : m_filepath(std::filesystem::path(filename).parent_path().string()) {}

    void Process(tinyxml2::XMLElement& elem, ChSystem& /*system*/) override {
        for (tinyxml2::XMLElement* vis = elem.FirstChildElement("visual"); vis;
             vis = elem.NextSiblingElement("visual")) {
            if (!ParseGeometry(vis->FirstChildElement("geometry"))) {
                std::cerr << "ChROSRobotModelHandler: failed to parse visual/geometry." << std::endl;
                return;
            }
            if (tinyxml2::XMLElement* mat = vis->FirstChildElement("material"))
                ParseMaterial(mat);
        }
        for (tinyxml2::XMLElement* col = elem.FirstChildElement("collision"); col;
             col = elem.NextSiblingElement("collision")) {
            if (!ParseGeometry(col->FirstChildElement("geometry"))) {
                std::cerr << "ChROSRobotModelHandler: failed to parse collision/geometry." << std::endl;
                return;
            }
        }
    }

  private:
    bool ParseGeometry(tinyxml2::XMLElement* geom) {
        if (!geom || !geom->FirstChildElement())
            return false;
        if (tinyxml2::XMLElement* shape = geom->FirstChildElement(); std::string(shape->Value()) == "mesh")
            shape->SetAttribute("filename", ResolveFilename(shape->Attribute("filename")).c_str());
        return true;
    }

    void ParseMaterial(tinyxml2::XMLElement* mat) {
        if (tinyxml2::XMLElement* tex = mat->FirstChildElement("texture"))
            tex->SetAttribute("filename", ResolveFilename(tex->Attribute("filename")).c_str());
    }

    // rviz2 expects a URI; convert any plain path (no "://") to an absolute file:// URI.
    std::string ResolveFilename(const std::string& filename) {
        if (filename.find("://") != std::string::npos)
            return filename;
        std::filesystem::path path(filename);
        if (!path.is_absolute())
            path = std::filesystem::absolute(std::filesystem::path(m_filepath) / path);
        return "file://" + path.string();
    }

    const std::string m_filepath;
};
}  // namespace
#endif

// update_rate 0 -> Tick runs every step, but we publish exactly once (the model
// is static) and rely on the latched QoS to deliver to late subscribers. (9.0
// passed a huge update_rate, which actually republished the string every step;
// this is the efficient equivalent.)
ChROSRobotModelHandler::ChROSRobotModelHandler(const std::string& robot_model, const std::string& topic_name)
    : ChROSHandler(0), m_topic_name(topic_name), m_robot_model(robot_model) {}

#ifdef CHRONO_HAS_URDF
ChROSRobotModelHandler::ChROSRobotModelHandler(chrono::parsers::ChParserURDF& parser, const std::string& topic_name)
    : ChROSHandler(0), m_topic_name(topic_name) {
    const auto filename = parser.GetFilename();
    auto xml = parser.CustomProcess("link", std::make_shared<CustomProcessorFilenameResolver>(filename));
    if (!xml) {
        std::cerr << "ChROSRobotModelHandler: failed to parse URDF file: " << filename << std::endl;
        return;
    }
    tinyxml2::XMLPrinter printer;
    xml->Print(&printer);
    m_robot_model = printer.CStr();
}
#endif

bool ChROSRobotModelHandler::Initialize(ChROSBridge& bridge) {
    m_publisher = bridge.CreatePublisher(m_topic_name, "std_msgs/msg/String", ChROSQoS::Latched());
    return true;
}

void ChROSRobotModelHandler::Tick(double time) {
    if (m_published)
        return;

    auto msg = m_publisher->NewMessage();
    msg.SetString("data", m_robot_model);
    if (m_publisher->Publish(msg))
        m_published = true;  // static model; the latched QoS serves late joiners
}

}  // namespace ros
}  // namespace chrono
