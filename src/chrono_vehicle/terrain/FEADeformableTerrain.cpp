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
// Authors: Antonio Recuero, Bryan Peterson
// =============================================================================
//
// FEA deformable terrain
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"

#include "chrono_fea/ChElementBrick_9.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/FEADeformableTerrain.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the FEADeformableTerrain wrapper class
// -----------------------------------------------------------------------------
FEADeformableTerrain::FEADeformableTerrain(ChSystem* system)
    : m_E(1.379e7),
      m_nu(0.3),
      m_rho(200.0),
      m_yield_stress(10000.0),
      m_hardening_slope(5000),
      m_friction_angle(0.00001),
      m_dilatancy_angle(0.00001) {
    m_mesh = std::make_shared<fea::ChMesh>();
    system->Add(m_mesh);
}

// Return the terrain height at the specified location
double FEADeformableTerrain::GetHeight(double x, double y) const {
    //// TODO
    return 0;
}

// Return the terrain normal at the specified location
ChVector<> FEADeformableTerrain::GetNormal(double x, double y) const {
    //// TODO
    return ChVector<>(0, 0, 1);
}

// Set properties of the FEA soil model
void FEADeformableTerrain::SetSoilParametersFEA(double rho,              ///< Soil density
                                                double Emod,             ///< Soil modulus of elasticity
                                                double nu,               ///< Soil Poisson ratio
                                                double yield_stress,     ///< Soil yield stress, for plasticity
                                                double hardening_slope,  ///< Soil hardening slope, for plasticity
                                                double friction_angle,   ///< Soil internal friction angle
                                                double dilatancy_angle   ///< Soil dilatancy angle
                                                ) {
    m_rho = rho;
    m_E = Emod;
    m_nu = nu;
    m_yield_stress = yield_stress;
    m_hardening_slope = hardening_slope;
    m_friction_angle = friction_angle;
    m_dilatancy_angle = dilatancy_angle;
}

// Initialize the terrain as a box of 9-node brick elements of given dimensions.
void FEADeformableTerrain::Initialize(const ChVector<>& start_point,
                                      const ChVector<>& terrain_dimension,
                                      const ChVector<int>& terrain_discretization) {
    // Specification of the mesh (40,20,6)
    int numDiv_x = terrain_discretization.x();
    int numDiv_y = terrain_discretization.y();
    int numDiv_z = terrain_discretization.z();

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // Calculate total number of elements based on user input
    int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = (numDiv_z + 1) * XYNumNodes + TotalNumElements;

    // For uniform mesh
    double dx = terrain_dimension.x() / numDiv_x;
    double dy = terrain_dimension.y() / numDiv_y;
    double dz = terrain_dimension.z() / numDiv_z;

    bool Plasticity = true;

    // Define location of nodes and create/add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = start_point.x() + (i % (numDiv_x + 1)) * dx;
            double loc_y = start_point.y() + (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = start_point.z() + j * dz;

            // Create the node
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
            node->SetMass(0);
            // Fix all nodes along the axis Z = 0
            if (j == 0) {
                node->SetFixed(true);
            }
            // Add node to mesh
            m_mesh->AddNode(node);
        }
    }

    // Fix nodes at the boundaries of the FEA 'box'
    for (int iz = 0; iz < numDiv_z; iz++) {
        for (int ix = 0; ix < N_x; ix++) {
            auto sidenode = std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(iz * N_x * N_y + ix));
            sidenode->SetFixed(true);

            auto farsidenode =
                std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode((N_x * numDiv_y) + iz * N_x * N_y + ix));
            farsidenode->SetFixed(true);
        }
    }
    for (int iz = 0; iz < numDiv_z; iz++) {
        for (int iy = 0; iy < N_y; iy++) {
            auto sidenode = std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode((iy + iz * N_y) * N_y));
            sidenode->SetFixed(true);

            auto farsidenode =
                std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(numDiv_x + iz * N_x * N_y + iy * N_y));
            farsidenode->SetFixed(true);
        }
    }
    // Initialize coordinates for curvature (central) node
    for (int i = 0; i < TotalNumElements; i++) {
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        m_mesh->AddNode(node);
    }

    // Basic material properties for soil.
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(m_rho);
    material->Set_E(m_E);
    material->Set_v(m_nu);

    // Initial plastic deformation tensor: Initially identity (elastic).
    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }
    int jj = -1;
    int kk;
    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        if (i % (numDiv_x * numDiv_y) == 0) {
            jj++;
            kk = 0;
        }
        // Define node sequence for element node0 thru node7 are corner nodes
        // Node8 is the central curvature vector node.
        int node0 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + jj * (N_x * N_y);
        int node1 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + jj * (N_x * N_y);
        int node2 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + jj * (N_x * N_y);
        int node3 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + jj * (N_x * N_y);
        int node4 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + XYNumNodes + jj * (N_x * N_y);
        int node5 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + XYNumNodes + jj * (N_x * N_y);
        int node6 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + XYNumNodes + jj * (N_x * N_y);
        int node7 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + XYNumNodes + jj * (N_x * N_y);
        int node8 = (numDiv_z + 1) * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementBrick_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(m_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(5e-4);    // Structural damping for this element
        element->SetGravityOn(true);    // Turn internal gravitational force calculation
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(ChElementBrick_9::Hencky);
        element->SetPlasticityFormulation(ChElementBrick_9::DruckerPrager);
        if (element->GetStrainFormulation() == ChElementBrick_9::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(m_yield_stress);
                element->SetHardeningSlope(m_hardening_slope);
                element->SetCCPInitial(CCPInitial);
                if (element->GetPlasticityFormulation() == ChElementBrick_9::DruckerPrager) {
                    element->SetFriction(m_friction_angle);
                    element->SetDilatancy(m_dilatancy_angle);
                    element->SetDPType(3);
                }
            }
        }

        // Add element to mesh
        m_mesh->AddElement(element);
        kk++;
    }

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(m_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.995);
    mvisualizemesh->SetSmoothFaces(false);
    m_mesh->AddAsset(mvisualizemesh);

    // Deactivate mesh gravity (added through system)
    m_mesh->SetAutomaticGravity(false);
}
}  // end namespace vehicle
}  // end namespace chrono
