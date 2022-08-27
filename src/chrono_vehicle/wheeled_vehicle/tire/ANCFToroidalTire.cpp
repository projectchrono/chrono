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
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// ANCF toroidal tire.
// This is a concrete ANCF tire class which uses a semi-toroidal tire mesh
// composed of single-layer ANCF shell elements.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

ANCFToroidalTire::ANCFToroidalTire(const std::string& name)
    : ChANCFTire(name),
      m_rim_radius(0.35),
      m_height(0.195),
      m_thickness(0.014),
      m_div_circumference(60),
      m_div_width(12),
      m_default_pressure(320.0e3),
      m_alpha(0.15) {
    // default contact material
    m_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
}

void ANCFToroidalTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    // Create an isotropic material (shared by all elements)
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(500, 9.0e7, 0.3);

    // Create the mesh nodes.
    // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
    // and are then transformed to the global frame.
    for (int i = 0; i < m_div_circumference; i++) {
        double phi = (CH_C_2PI * i) / m_div_circumference;

        for (int j = 0; j <= m_div_width; j++) {
            double theta = -CH_C_PI_2 + (CH_C_PI * j) / m_div_width;

            double x = (m_rim_radius + m_height * cos(theta)) * cos(phi);
            double y = m_height * sin(theta);
            double z = (m_rim_radius + m_height * cos(theta)) * sin(phi);
            ChVector<> loc = wheel_frame.TransformPointLocalToParent(ChVector<>(x, y, z));

            double nx = cos(theta) * cos(phi);
            double ny = sin(theta);
            double nz = cos(theta) * sin(phi);
            ChVector<> dir = wheel_frame.TransformDirectionLocalToParent(ChVector<>(nx, ny, nz));

            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
            node->SetMass(0);
            m_mesh->AddNode(node);
        }
    }

    // Element thickness
    double dz = m_thickness;

    // Create the ANCF shell elements
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

            auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode0));
            auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode1));
            auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode2));
            auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode3));

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(node0, node1, node2, node3);

            // Element dimensions
            double dx =
                0.5 * ((node1->GetPos() - node0->GetPos()).Length() + (node3->GetPos() - node2->GetPos()).Length());
            double dy = 
                0.5 * ((node2->GetPos() - node1->GetPos()).Length() + (node3->GetPos() - node0->GetPos()).Length());

            // Set element dimensions
            element->SetDimensions(dx, dy);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(m_alpha);

            // Add element to mesh
            m_mesh->AddElement(element);
        }
    }
}

std::vector<std::shared_ptr<ChNodeFEAbase>> ANCFToroidalTire::GetConnectedNodes() const {
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

void ANCFToroidalTire::CreateContactMaterial() {
    m_contact_mat = m_mat;
}

}  // end namespace vehicle
}  // end namespace chrono
