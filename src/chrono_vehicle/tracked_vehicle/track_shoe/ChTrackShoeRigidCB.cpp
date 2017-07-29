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
// Base class for a rigid-link track shoe in a continuous band track.
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoeRigidCB::ChTrackShoeRigidCB(const std::string& name) : ChTrackShoe(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                    const ChVector<>& location,
                                    const ChQuaternion<>& rotation) {
    // Express the track shoe location and orientation in global frame.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    ChVector<> xdir = rot.GetXaxis();
    ChVector<> ydir = rot.GetYaxis();
    ChVector<> zdir = rot.GetZaxis();

    // Create the shoe body.
    m_shoe = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shoe->SetNameString(m_name + "_shoe");
    ////m_shoe->SetPos(loc);
    ////m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    chassis->GetSystem()->AddBody(m_shoe);

    // Add contact geometry.
    m_shoe->SetCollide(true);

    switch (m_shoe->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_shoe->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_shoe->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_shoe->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_shoe->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_shoe->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_shoe->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_shoe->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_shoe->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_shoe->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_shoe->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }

    AddShoeContact();

    // Create the tooth body.
    m_tooth = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_tooth->SetNameString(m_name + "_tooth");
    ////m_tooth->SetPos(loc);
    ////m_tooth->SetRot(rot);
    m_tooth->SetMass(GetToothMass());
    m_tooth->SetInertiaXX(GetToothInertia());
    chassis->GetSystem()->AddBody(m_tooth);

    // Set contact material properties.
    //// TODO: different material
    switch (m_tooth->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_tooth->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_tooth->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_tooth->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_tooth->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_tooth->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_tooth->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_tooth->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_tooth->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_tooth->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_tooth->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }

    // Create the link bodies.
    for (int i = 0; i < GetNumLinks(); i++) {
        m_links[i] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
        m_links[i]->SetNameString(m_name + "_link_" + std::to_string(i));
        ////m_links[i]->SetPos(loc);
        ////m_links[i]->SetRot(rot);
        m_links[i]->SetMass(GetToothMass());
        m_links[i]->SetInertiaXX(GetToothInertia());
        chassis->GetSystem()->AddBody(m_links[i]);

        // Set contact material properties.
        //// TODO: different material
        switch (m_links[i]->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_links[i]->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_links[i]->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_links[i]->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_links[i]->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_links[i]->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_links[i]->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_links[i]->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_links[i]->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_links[i]->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_links[i]->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
        }
    }
}

void ChTrackShoeRigidCB::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                    const ChVector<>& loc_shoe,
                                    const ChQuaternion<>& rot_shoe,
                                    const ChVector<>& loc_tooth,
                                    const ChQuaternion<>& rot_tooth,
                                    const std::vector<ChVector<>>& loc_links,
                                    const std::vector<ChQuaternion<>>& rot_links) {
    // Initialize at origin.
    Initialize(chassis, VNULL, QUNIT);

    // Overwrite absolute body locations and orientations.
    m_shoe->SetPos(chassis->TransformPointLocalToParent(loc_shoe));
    m_shoe->SetRot(chassis->GetRot() * rot_shoe);

    m_tooth->SetPos(chassis->TransformPointLocalToParent(loc_tooth));
    m_tooth->SetRot(chassis->GetRot() * rot_tooth);

    for (int i = 0; i < GetNumLinks(); i++) {
        m_links[i]->SetPos(chassis->TransformPointLocalToParent(loc_links[i]));
        m_links[i]->SetRot(chassis->GetRot() * rot_links[i]);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeRigidCB::GetMass() const {
    return GetShoeMass() + GetToothMass() + GetNumLinks() * GetLinkMass();
}

double ChTrackShoeRigidCB::GetPitch() const {
    return GetShoeLength() + GetNumLinks() * GetLinkLength();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddShoeContact() {
    m_shoe->GetCollisionModel()->ClearModel();

    //// TODO

    m_shoe->GetCollisionModel()->BuildModel();
}

void ChTrackShoeRigidCB::AddLinkContact(int link_index) {
    m_links[link_index]->GetCollisionModel()->ClearModel();

    //// TODO

    m_links[link_index]->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddToothVisualization();
    AddShoeVisualization();
    for (int i = 0; i < GetNumLinks(); i++) {
        AddLinkVisualization(i);
    }
}

void ChTrackShoeRigidCB::RemoveVisualizationAssets() {
    m_tooth->GetAssets().clear();
    m_shoe->GetAssets().clear();
    for (int i = 0; i < GetNumLinks(); i++) {
        m_links[i]->GetAssets().clear();
    }
}

void ChTrackShoeRigidCB::AddToothVisualization() {
    //// TODO
}

void ChTrackShoeRigidCB::AddShoeVisualization() {
    //// TODO
}

void ChTrackShoeRigidCB::AddLinkVisualization(int link_index) {
    //// TODO
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::Connect(std::shared_ptr<ChTrackShoe> next) {
    ChSystem* system = m_shoe->GetSystem();

    //// TODO: create all internal joints and force elements
    ////       - connect shoe body to tooth body (weld joint)
    ////       - connect links (bushings)
}

}  // end namespace vehicle
}  // end namespace chrono
