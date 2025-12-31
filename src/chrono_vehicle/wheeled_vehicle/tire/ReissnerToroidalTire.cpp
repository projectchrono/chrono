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
// Authors: Alessandro Tasora, Radu Serban, Antonio Recuero
// =============================================================================
//
// Sample toroidal tire based on Reissner shell elements
// This is a customizable Reissner tire class which uses a toroidal tire mesh
// composed of single-layer Reissner shell elements.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerToroidalTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

ReissnerToroidalTire::ReissnerToroidalTire(const std::string& name)
    : ChReissnerTire(name),
      m_rim_radius(0.35),
      m_height(0.195),
      m_thickness(0.014),
      m_div_circumference(60),
      m_div_width(12),
      m_default_pressure(320.0e3),
      m_alpha(0.015) {
    // default contact material
    m_mat = chrono_types::make_shared<ChContactMaterialSMC>();
}

void ReissnerToroidalTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    // Create an isotropic material (shared by all elements)
    auto melasticity = chrono_types::make_shared<ChElasticityReissnerIsothropic>(9.0e7, 0.3, 1.0, 0.01);
    auto mdamping = chrono_types::make_shared<ChDampingReissnerRayleigh>(melasticity, m_alpha);
    auto mat = chrono_types::make_shared<ChMaterialShellReissner>(melasticity, nullptr, mdamping);
    mat->SetDensity(500);

    // In case you need also damping it would add...
    // auto mdamping = chrono_types::make_shared<ChDampingReissnerRayleigh>(melasticity,0.01);
    // auto mat = chrono_types::make_shared<ChMaterialShellReissner>(melasticity, nullptr, mdamping);

    // Create the mesh nodes.
    // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
    // and are then transformed to the global frame.
    for (int i = 0; i < m_div_circumference; i++) {
        double phi = (CH_2PI * i) / m_div_circumference;

        for (int j = 0; j <= m_div_width; j++) {
            double theta = -CH_PI_2 + (CH_PI * j) / m_div_width;

            double x = (m_rim_radius + m_height * std::cos(theta)) * std::cos(phi);
            double y = m_height * std::sin(theta);
            double z = (m_rim_radius + m_height * std::cos(theta)) * std::sin(phi);
            ChVector3d loc = wheel_frame.TransformPointLocalToParent(ChVector3d(x, y, z));

            double nx = std::cos(theta) * std::cos(phi);
            double ny = std::sin(theta);
            double nz = std::cos(theta) * std::sin(phi);
            ChVector3d dir = wheel_frame.TransformDirectionLocalToParent(ChVector3d(nx, ny, nz));
            ChMatrix33<> mrot;
            mrot.SetFromAxisX(dir, VECT_Y);

            auto node = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(loc, mrot));

            m_mesh->AddNode(node);
        }
    }

    // Element dimensions
    double dz = m_thickness;
    ////double dx = CH_2PI * (m_rim_radius + m_height) / (2 * m_div_circumference);
    ////double dy = CH_PI * m_height / m_div_width;

    // Create the Reissner shell elements
    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j < m_div_width; j++) {
            // Adjacent nodes
            int inode0, inode1, inode2, inode3;
            inode1 = j + i * (m_div_width + 1);
            inode2 = j + 1 + i * (m_div_width + 1);
            if (i == m_div_circumference - 1) {
                inode0 = j;
                inode3 = j + 1;
            } else {
                inode0 = j + (i + 1) * (m_div_width + 1);
                inode3 = j + 1 + (i + 1) * (m_div_width + 1);
            }

            auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode0));
            auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode1));
            auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode2));
            auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode3));

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellReissner4>();
            element->SetNodes(node0, node1, node2, node3);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);

            // Add element to mesh
            m_mesh->AddElement(element);
        }
    }

    // automatic gravity
    m_mesh->SetAutomaticGravity(true);
}

std::vector<std::shared_ptr<ChNodeFEAbase>> ReissnerToroidalTire::GetConnectedNodes() const {
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> nodes;

    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j <= m_div_width; j++) {
            int index = j + i * (m_div_width + 1);
            if (index % (m_div_width + 1) == 0) {
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(m_mesh->GetNode(index)));
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(m_mesh->GetNode(index + m_div_width)));
            }
        }
    }

    return nodes;
}

void ReissnerToroidalTire::CreateContactMaterial() {
    m_contact_mat = m_mat;
}

}  // end namespace vehicle
}  // end namespace chrono
