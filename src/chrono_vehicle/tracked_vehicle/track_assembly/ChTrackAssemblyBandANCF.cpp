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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for a continuous band track assembly using an ANCFshell-based web
// (template definition).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandANCF.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the custom callback class for culling broadphase collisions
// -----------------------------------------------------------------------------
ChTrackAssemblyBandANCF::BroadphaseCulling::BroadphaseCulling(ChTrackAssemblyBandANCF* assembly)
    : m_assembly(assembly) {}

bool ChTrackAssemblyBandANCF::BroadphaseCulling::OnBroadphase(ChCollisionModel* modelA, ChCollisionModel* modelB) {
    auto contactableA = modelA->GetContactable();
    auto contactableB = modelB->GetContactable();

    if (dynamic_cast<fea::ChContactNodeXYZsphere*>(contactableA) ||
        dynamic_cast<fea::ChContactTriangleXYZ*>(contactableA)) {
        // Reject this candidate pair if contactableB is a track shoe tread body
        for (auto shoe : m_assembly->m_shoes) {
            if (contactableB == shoe->GetShoeBody().get())
                return false;
        }
    }

    if (dynamic_cast<fea::ChContactNodeXYZsphere*>(contactableB) ||
        dynamic_cast<fea::ChContactTriangleXYZ*>(contactableB)) {
        // Reject this candidate pair if contactableA is a track shoe tread body
        for (auto shoe : m_assembly->m_shoes) {
            if (contactableA == shoe->GetShoeBody().get())
                return false;
        }
    }

    // Accept this candidate
    return true;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackAssemblyBandANCF::ChTrackAssemblyBandANCF(const std::string& name, VehicleSide side)
    : ChTrackAssemblyBand(name, side),
      m_contact_type(ContactSurfaceType::TRIANGLE_MESH),
      m_callback(nullptr),
      m_rubber_rho(1100),
      m_rubber_E(1e7),
      m_rubber_nu(0.49),
      m_rubber_G(0.5 * 1e7 / (1 + 0.49)),
      m_steel_rho(7900),
      m_steel_E(210e9),
      m_steel_nu(0.3),
      m_steel_G(0.5 * 210e9 / (1 + 0.3)),
      m_angle_1(0),
      m_angle_2(0),
      m_angle_3(0),
      m_alpha(0.05) {}

ChTrackAssemblyBandANCF::~ChTrackAssemblyBandANCF() {
    auto sys = m_track_mesh->GetSystem();
    if (sys) {
        sys->Remove(m_track_mesh);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackAssemblyBandANCF::SetRubberLayerMaterial(double rho,
                                                     const ChVector3d& E,
                                                     const ChVector3d& nu,
                                                     const ChVector3d& G) {
    m_rubber_rho = rho;
    m_rubber_E = E;
    m_rubber_nu = nu;
    m_rubber_G = G;
}

void ChTrackAssemblyBandANCF::SetSteelLayerMaterial(double rho,
                                                    const ChVector3d& E,
                                                    const ChVector3d& nu,
                                                    const ChVector3d& G) {
    m_steel_rho = rho;
    m_steel_E = E;
    m_steel_nu = nu;
    m_steel_G = G;
}

void ChTrackAssemblyBandANCF::SetElementStructuralDamping(double alpha) {
    m_alpha = alpha;
}

void ChTrackAssemblyBandANCF::SetLayerFiberAngles(double angle_1, double angle_2, double angle_3) {
    m_angle_1 = angle_1;
    m_angle_2 = angle_2;
    m_angle_3 = angle_3;
}

// -----------------------------------------------------------------------------
// Assemble the track shoes over the wheels.
//
// Returns true if the track shoes were initialized in a counter clockwise
// direction and false otherwise.
//
// This procedure is performed in the chassis reference frame, taking into
// account the convention that the chassis reference frame has the x-axis
// pointing to the front of the vehicle and the z-axis pointing up.
// It is also assumed that the sprocket, idler, and road wheels lie in the
// same vertical plane (in the chassis reference frame). The assembly is done
// in the (x-z) plane.
//
// TODO: NEEDS fixes for clock-wise wrapping (idler in front of sprocket)
//
// -----------------------------------------------------------------------------
bool ChTrackAssemblyBandANCF::Assemble(std::shared_ptr<ChChassis> chassis) {
    // Only SMC contact is currently possible with FEA
    assert(chassis->GetSystem()->GetContactMethod() == ChContactMethod::SMC);

    // Number of track shoes
    int num_shoes = static_cast<int>(m_shoes.size());

    // Set up web connection lengths (always 2 for this type of track shoe)
    std::vector<double> connection_lengths(2);
    connection_lengths[0] = m_shoes[0]->GetToothBaseLength();
    connection_lengths[1] = m_shoes[0]->GetWebLength();

    // Create ANCF materials (shared by all track shoes)
    auto rubber_mat =
        chrono_types::make_shared<fea::ChMaterialShellANCF>(m_rubber_rho, m_rubber_E, m_rubber_nu, m_rubber_G);
    auto steel_mat = chrono_types::make_shared<fea::ChMaterialShellANCF>(m_steel_rho, m_steel_E, m_steel_nu, m_steel_G);

    // Calculate assembly points
    std::vector<ChVector2d> shoe_points;
    bool ccw = FindAssemblyPoints(chassis->GetBody(), num_shoes, connection_lengths, shoe_points);

    // Create and add the mesh container for the track shoe webs to the system
    m_track_mesh = chrono_types::make_shared<ChMesh>();
    chassis->GetSystem()->Add(m_track_mesh);

    // Now create all of the track shoes at the located points
    auto num_shoe_elements = connection_lengths.size();
    for (int s = 0; s < num_shoes; s++) {
        std::vector<ChCoordsys<>> shoe_components_coordsys;
        for (auto i = 0; i < num_shoe_elements; i++) {
            ChVector2d mid = (shoe_points[i + 1 + s * num_shoe_elements] + shoe_points[i + s * num_shoe_elements]) / 2;
            ChVector2d dir = shoe_points[i + 1 + s * num_shoe_elements] - shoe_points[i + s * num_shoe_elements];
            ChVector3d loc(mid.x(), m_sprocket_offset, mid.y());
            double ang = std::atan2(dir.y(), dir.x());
            ChQuaternion<> rot = QuatFromAngleY(-ang);  // Negative of the angle in 3D

            shoe_components_coordsys.push_back(ChCoordsys<>(loc, rot));
        }

        // Set shoe index within the track assembly
        m_shoes[s]->SetIndex(s);
        // Pass the track mesh container to the shoe so that it adds to it
        m_shoes[s]->SetWebMesh(m_track_mesh);
        // Pass material properties to the shoe
        m_shoes[s]->SetWebMeshProperties(rubber_mat, steel_mat, m_angle_1, m_angle_2, m_angle_3, m_alpha);

        // Initialize the track shoe system
        m_shoes[s]->Initialize(chassis, shoe_components_coordsys);
    }

    // Add contact for the mesh -- only if SMC!!!

    if (chassis->GetSystem()->GetContactMethod() == ChContactMethod::SMC) {
        // Create the contact material
        CreateContactMaterial(ChContactMethod::SMC);

        double thickness = m_shoes[0]->GetWebThickness();

        switch (m_contact_type) {
            case ContactSurfaceType::NODE_CLOUD: {
                auto contact_surf = chrono_types::make_shared<ChContactSurfaceNodeCloud>(m_contact_material);
                contact_surf->AddAllNodes(*m_track_mesh, thickness / 2);
                m_track_mesh->AddContactSurface(contact_surf);

                // Place all collision triangles in the same collision family and disable contact with each other
                for (auto& node : contact_surf->GetNodes()) {
                    node->GetCollisionModel()->SetFamily(VehicleCollisionFamily::SHOE_FAMILY);
                    node->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);
                }

                break;
            }
            case ContactSurfaceType::TRIANGLE_MESH: {
                auto contact_surf = chrono_types::make_shared<ChContactSurfaceMesh>(m_contact_material);
                contact_surf->AddFacesFromBoundary(*m_track_mesh, thickness / 2, false);
                m_track_mesh->AddContactSurface(contact_surf);

                // Place all collision triangles in the same collision family and disable contact with each other
                for (auto& face : contact_surf->GetTrianglesXYZ()) {
                    face->GetCollisionModel()->SetFamily(VehicleCollisionFamily::SHOE_FAMILY);
                    face->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);
                }

                break;
            }
            case ContactSurfaceType::NONE: {
                break;
            }
        }
    }

    // Create and register the custom broadphase callback.
    m_callback = chrono_types::make_shared<BroadphaseCulling>(this);
    chassis->GetSystem()->GetCollisionSystem()->RegisterBroadphaseCallback(m_callback);

    return ccw;
}

void ChTrackAssemblyBandANCF::RemoveTrackShoes() {
    m_shoes.clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void ChTrackAssemblyBandANCF::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColormapRange(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    m_track_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    m_track_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    m_track_mesh->AddVisualShapeFEA(mvisualizemeshC);
}

void ChTrackAssemblyBandANCF::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_track_mesh);
}

}  // end namespace vehicle
}  // end namespace chrono
