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

#include "chrono/assets/ChPointPointShape.h"
#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkRevoluteSpherical.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"

namespace chrono {

void ChPointPointShape::Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
    // Extract two positions from updater if it has any, and then update line geometry from these positions.
    if (auto link_markers = dynamic_cast<ChLinkMarkers*>(updater)) {
        UpdateLineGeometry(frame.TransformPointParentToLocal(link_markers->GetMarker1()->GetAbsCoord().pos),
                           frame.TransformPointParentToLocal(link_markers->GetMarker2()->GetAbsCoord().pos));
    } else if (auto link_dist = dynamic_cast<ChLinkDistance*>(updater)) {
        UpdateLineGeometry(frame.TransformPointParentToLocal(link_dist->GetEndPoint1Abs()),
                           frame.TransformPointParentToLocal(link_dist->GetEndPoint2Abs()));
    } else if (auto link_rs = dynamic_cast<ChLinkRevoluteSpherical*>(updater)) {
        UpdateLineGeometry(frame.TransformPointParentToLocal(link_rs->GetPoint1Abs()),
                           frame.TransformPointParentToLocal(link_rs->GetPoint2Abs()));
    } else if (auto link_tsda = dynamic_cast<ChLinkTSDA*>(updater)) {
        UpdateLineGeometry(frame.TransformPointParentToLocal(link_tsda->GetPoint1Abs()),
                           frame.TransformPointParentToLocal(link_tsda->GetPoint2Abs()));
    } else if (auto link_mate = dynamic_cast<ChLinkMateGeneric*>(updater)) {
        auto pt1 = link_mate->GetBody1()->TransformPointLocalToParent(link_mate->GetFrame1().GetPos());
        auto pt2 = link_mate->GetBody2()->TransformPointLocalToParent(link_mate->GetFrame2().GetPos());
        UpdateLineGeometry(frame.TransformPointParentToLocal(pt1), frame.TransformPointParentToLocal(pt2));
    } else if (auto link = dynamic_cast<ChLink*>(updater)) {
        UpdateLineGeometry(frame.TransformPointParentToLocal(link->GetBody1()->GetPos()),
                           frame.TransformPointParentToLocal(link->GetBody2()->GetPos()));
    }
}

// Set line geometry as a segment between two end point
void ChSegmentShape::UpdateLineGeometry(const ChVector<>& endpoint1, const ChVector<>& endpoint2) {
    this->SetLineGeometry(std::static_pointer_cast<geometry::ChLine>(
        chrono_types::make_shared<geometry::ChLineSegment>(endpoint1, endpoint2)));
};

// Set line geometry as a coil between two end point
void ChSpringShape::UpdateLineGeometry(const ChVector<>& endpoint1, const ChVector<>& endpoint2) {
    auto linepath = chrono_types::make_shared<geometry::ChLinePath>();

    // Following part was copied from irrlicht::tools::drawSpring()
    ChVector<> dist = endpoint2 - endpoint1;
    ChVector<> Vx, Vy, Vz;
    double length = dist.Length();
    ChVector<> dir = dist.GetNormalized();
    XdirToDxDyDz(dir, VECT_Y, Vx, Vy, Vz);

    ChMatrix33<> rel_matrix(Vx, Vy, Vz);
    ChCoordsys<> mpos(endpoint1, rel_matrix.Get_A_quaternion());

    double phaseA = 0;
    double phaseB = 0;
    double heightA = 0;
    double heightB = 0;

    for (int iu = 1; iu <= resolution; iu++) {
        phaseB = turns * CH_C_2PI * (double)iu / (double)resolution;
        heightB = length * ((double)iu / (double)resolution);
        ChVector<> V1(heightA, radius * cos(phaseA), radius * sin(phaseA));
        ChVector<> V2(heightB, radius * cos(phaseB), radius * sin(phaseB));

        auto segment = geometry::ChLineSegment(mpos.TransformLocalToParent(V1), mpos.TransformLocalToParent(V2));
        linepath->AddSubLine(segment);
        phaseA = phaseB;
        heightA = heightB;
    }

    this->SetLineGeometry(std::static_pointer_cast<geometry::ChLine>(linepath));
}

void ChRotSpringShape::Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
    // Do nothing if not associated with an RSDA.
    auto rsda = dynamic_cast<ChLinkRSDA*>(updater);
    if (!rsda)
        return;

    // The asset frame for an RSDA is the frame on body1.
    auto linepath = chrono_types::make_shared<geometry::ChLinePath>();
    double del_angle = rsda->GetAngle() / m_resolution;
    ChVector<> V1(m_radius, 0, 0);
    for (int iu = 1; iu <= m_resolution; iu++) {
        double crt_angle = iu * del_angle;
        double crt_radius = m_radius - (iu * del_angle / CH_C_2PI) * (m_radius / 10);
        ChVector<> V2(crt_radius * std::cos(crt_angle), crt_radius * std::sin(crt_angle), 0);
        auto segment = geometry::ChLineSegment(V1, V2);
        linepath->AddSubLine(segment);
        V1 = V2;
    }

    this->SetLineGeometry(std::static_pointer_cast<geometry::ChLine>(linepath));
}

}  // end namespace chrono
