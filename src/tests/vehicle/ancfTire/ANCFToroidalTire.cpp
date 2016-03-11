// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Sample ANCF toroidal tire.
// This is a concrete ANCF tire class which uses a semi-toroidal tire mesh
// composed of single-layer ANCF shell elements.
//
// =============================================================================

#include "ANCFToroidalTire.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fea;

const double ANCFToroidalTire::m_rim_radius = 0.35;
const double ANCFToroidalTire::m_height = 0.195;
const double ANCFToroidalTire::m_thickness = 0.014;

const int ANCFToroidalTire::m_div_circumference = 60;
const int ANCFToroidalTire::m_div_width = 12;

const double ANCFToroidalTire::m_default_pressure = 320.0e3;
const double ANCFToroidalTire::m_alpha = 0.15;

ANCFToroidalTire::ANCFToroidalTire(const std::string& name) : ChANCFTire(name) {
    SetContactMaterial(0.9f, 0.1f, 2e7f, 0.3f);
    SetContactSurfaceType(ChANCFTire::NODE_CLOUD);
}

void ANCFToroidalTire::CreateMesh(std::shared_ptr<fea::ChMesh> mesh,
                                  const ChFrameMoving<>& wheel_frame,
                                  VehicleSide side) {
    // Create an isotropic material (shared by all elements)
    auto mat = std::make_shared<ChMaterialShellANCF>(500, 9.0e7, 0.3);

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

            auto node = std::make_shared<ChNodeFEAxyzD>(loc, dir);
            node->SetMass(0);
            mesh->AddNode(node);
        }
    }

    // Element dimensions
    double dz = m_thickness;
    double dx = CH_C_2PI * (m_rim_radius + m_height) / (2 * m_div_circumference);
    double dy = CH_C_PI * m_height / m_div_width;

    // Create the ANCF shell elements
    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j < m_div_width; j++) {
            // Adjacent nodes
            int inode0, inode1, inode2, inode3;
            inode1 = j + i * (m_div_width + 1);
            inode3 = j + 1 + i * (m_div_width + 1);
            if (i == m_div_circumference - 1) {
                inode0 = j;
                inode2 = j + 1;
            }
            else {
                inode0 = j + (i + 1) * (m_div_width + 1);
                inode2 = j + 1 + (i + 1) * (m_div_width + 1);
            }

            auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode0));
            auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode1));
            auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode2));
            auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode3));

            // Create the element and set its nodes.
            auto element = std::make_shared<ChElementShellANCF>();
            element->SetNodes(node0, node1, node2, node3);

            // Set element dimensions
            element->SetDimensions(dx, dy);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(m_alpha);
            element->SetGravityOn(true);

            // Add element to mesh
            mesh->AddElement(element);
        }
    }

    // Switch off automatic gravity
    mesh->SetAutomaticGravity(false);
}

NodeList ANCFToroidalTire::GetConnectedNodes(const std::shared_ptr<fea::ChMesh>& mesh) const {
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> nodes;

    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j <= m_div_width; j++) {
            int index = j + i * (m_div_width + 1);
            if (index % (m_div_width + 1) == 0) {
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(index)));
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(index + m_div_width)));
            }
        }
    }

    return nodes;
}
