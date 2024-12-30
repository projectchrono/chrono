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
// Base class for a continuous band track shoe using a bushing-based web
// (template definition).
//
// =============================================================================

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChTrackShoeBandBushing::ChTrackShoeBandBushing(const std::string& name) : ChTrackShoeBand(name) {}

ChTrackShoeBandBushing::~ChTrackShoeBandBushing() {}

// -----------------------------------------------------------------------------
void ChTrackShoeBandBushing::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                        const ChVector3d& location,
                                        const ChQuaternion<>& rotation) {
    // Initialize base class (create tread body)
    ChTrackShoeBand::Initialize(chassis, location, rotation);

    // Cache values calculated from template parameters.
    m_seg_length = GetWebLength() / GetNumWebSegments();
    m_seg_mass = GetWebMass() / GetNumWebSegments();
    m_seg_inertia = GetWebInertia();  //// TODO - properly distribute web inertia

    // Express the tread body location and orientation in global frame.
    ChVector3d loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    ChVector3d xdir = rot.GetAxisX();

    // Create the required number of web segment bodies
    auto web_mat = m_body_matinfo.CreateMaterial(chassis->GetSystem()->GetContactMethod());
    ChVector3d seg_loc = loc + (0.5 * GetToothBaseLength()) * xdir;
    for (unsigned int is = 0; is < GetNumWebSegments(); is++) {
        m_web_segments.push_back(chrono_types::make_shared<ChBody>());
        m_web_segments[is]->SetName(m_name + "_web_" + std::to_string(is));
        m_web_segments[is]->SetPos(seg_loc + ((2 * is + 1) * m_seg_length / 2) * xdir);
        m_web_segments[is]->SetRot(rot);
        m_web_segments[is]->SetMass(m_seg_mass);
        m_web_segments[is]->SetInertiaXX(m_seg_inertia);
        m_web_segments[is]->EnableCollision(true);
        chassis->GetSystem()->AddBody(m_web_segments[is]);

        // Add contact geometry
        AddWebContact(m_web_segments[is], web_mat);
    }
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandBushing::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                        const std::vector<ChCoordsys<>>& component_pos) {
    // Check the number of provided locations and orientations.
    assert(component_pos.size() == GetNumWebSegments() + 1);

    // Initialize at origin.
    Initialize(chassis, VNULL, QUNIT);

    // Overwrite absolute body locations and orientations.
    m_shoe->SetPos(chassis->TransformPointLocalToParent(component_pos[0].pos));
    m_shoe->SetRot(chassis->GetRot() * component_pos[0].rot);

    for (unsigned int is = 0; is < GetNumWebSegments(); is++) {
        m_web_segments[is]->SetPos(chassis->TransformPointLocalToParent(component_pos[is + 1].pos));
        m_web_segments[is]->SetRot(chassis->GetRot() * component_pos[is + 1].rot);
    }
}

void ChTrackShoeBandBushing::InitializeInertiaProperties() {
    m_mass = GetTreadMass() + GetWebMass();
}

void ChTrackShoeBandBushing::UpdateInertiaProperties() {
    m_xform = m_shoe->GetFrameRefToAbs();

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_shoe->GetFrameCOMToAbs(), m_shoe->GetMass(), m_shoe->GetInertia());
    for (unsigned int is = 0; is < GetNumWebSegments(); is++) {
        composite.AddComponent(m_web_segments[is]->GetFrameCOMToAbs(), m_web_segments[is]->GetMass(),
                               m_web_segments[is]->GetInertia());
    }

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandBushing::AddWebContact(std::shared_ptr<ChBody> segment,
                                           std::shared_ptr<ChContactMaterial> web_mat) {
    auto shape =
        chrono_types::make_shared<ChCollisionShapeBox>(web_mat, m_seg_length, GetBeltWidth(), GetWebThickness());
    segment->AddCollisionShape(shape);

    segment->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    segment->GetCollisionModel()->DisallowCollisionsWith(TrackedCollisionFamily::SHOES);
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandBushing::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
    for (auto& segment : m_web_segments)
        AddWebVisualization(segment);
}

void ChTrackShoeBandBushing::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_shoe);
    for (auto& segment : m_web_segments) {
        ChPart::RemoveVisualizationAssets(segment);
    }
}

void ChTrackShoeBandBushing::AddWebVisualization(std::shared_ptr<ChBody> segment) {
    auto box = chrono_types::make_shared<ChVisualShapeBox>(m_seg_length, GetBeltWidth(), GetWebThickness());
    segment->AddVisualShape(box);

    double radius = GetWebThickness() / 4;
    utils::ChBodyGeometry::AddVisualizationCylinder(
        segment,                                                            //
        ChVector3d(m_seg_length / 2, -GetBeltWidth() / 2 - 2 * radius, 0),  //
        ChVector3d(m_seg_length / 2, +GetBeltWidth() / 2 + 2 * radius, 0),  //
        radius);
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandBushing::Connect(std::shared_ptr<ChTrackShoe> next,
                                     ChTrackAssembly* assembly,
                                     ChChassis* chassis,
                                     bool ccw) {
    int index = 0;

    auto z2y = QuatFromAngleX(CH_PI_2);

    // Connect tread body to the first web segment.
    {
        ChVector3d loc = m_shoe->TransformPointLocalToParent(ChVector3d(GetToothBaseLength() / 2, 0, 0));
        ChQuaternion<> rot = m_shoe->GetRot() * z2y;
        auto bushing = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::REVOLUTE, m_name + "_bushing_" + std::to_string(index++), m_web_segments[0], m_shoe,
            ChFrame<>(loc, rot), GetBushingData());
        chassis->AddJoint(bushing);
        m_web_bushings.push_back(bushing->GetAsBushing());
    }

    // Connect the web segments to each other.
    for (size_t is = 0; is < GetNumWebSegments() - 1; is++) {
        ChVector3d loc = m_web_segments[is]->TransformPointLocalToParent(ChVector3d(m_seg_length / 2, 0, 0));
        ChQuaternion<> rot = m_web_segments[is]->GetRot() * z2y;
        auto bushing = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::REVOLUTE, m_name + "_bushing_" + std::to_string(index++), m_web_segments[is + 1],
            m_web_segments[is], ChFrame<>(loc, rot), GetBushingData());
        chassis->AddJoint(bushing);
        m_web_bushings.push_back(bushing->GetAsBushing());
    }

    {
        // Connect the last web segment to the tread body from the next track shoe.
        int is = GetNumWebSegments() - 1;
        ChVector3d loc = m_web_segments[is]->TransformPointLocalToParent(ChVector3d(m_seg_length / 2, 0, 0));
        ChQuaternion<> rot = m_web_segments[is]->GetRot() * z2y;
        auto bushing = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::REVOLUTE, m_name + "_bushing_" + std::to_string(index++), next->GetShoeBody(),
            m_web_segments[is], ChFrame<>(loc, rot), GetBushingData());
        chassis->AddJoint(bushing);
        m_web_bushings.push_back(bushing->GetAsBushing());
    }
}

ChVector3d ChTrackShoeBandBushing::GetTension() const {
    return m_web_bushings[0]->GetForce();
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandBushing::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    bodies.insert(bodies.end(), m_web_segments.begin(), m_web_segments.end());
    ExportBodyList(jsonDocument, bodies);

    ExportBodyLoadList(jsonDocument, m_web_bushings);
}

void ChTrackShoeBandBushing::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    bodies.insert(bodies.end(), m_web_segments.begin(), m_web_segments.end());
    database.WriteBodies(bodies);

    database.WriteBodyLoads(m_web_bushings);
}

}  // end namespace vehicle
}  // end namespace chrono
