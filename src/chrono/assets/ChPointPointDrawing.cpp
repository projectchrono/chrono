//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkRevoluteSpherical.h"

namespace chrono {

void ChPointPointDrawing::Update(ChPhysicsItem* updater, const ChCoordsys<>& coords) {
    // Extract two positions from updater if it has any, and then update line geometry from these positions.
    if (auto link_markers = dynamic_cast<ChLinkMarkers*>(updater)) {
        UpdateLineGeometry(coords.TransformPointParentToLocal(link_markers->GetMarker1()->GetAbsCoord().pos),
                           coords.TransformPointParentToLocal(link_markers->GetMarker2()->GetAbsCoord().pos));
    } else if (auto link_dist = dynamic_cast<ChLinkDistance*>(updater)) {
        UpdateLineGeometry(coords.TransformPointParentToLocal(link_dist->GetEndPoint1Abs()),
                           coords.TransformPointParentToLocal(link_dist->GetEndPoint2Abs()));
    } else if (auto link_rs = dynamic_cast<ChLinkRevoluteSpherical*>(updater)) {
        UpdateLineGeometry(coords.TransformPointParentToLocal(link_rs->GetPoint1Abs()),
                           coords.TransformPointParentToLocal(link_rs->GetPoint2Abs()));
    } else if (auto link = dynamic_cast<ChLink*>(updater)) {
        UpdateLineGeometry(coords.TransformPointParentToLocal(link->GetBody1()->GetPos()),
                           coords.TransformPointParentToLocal(link->GetBody2()->GetPos()));
    }

    // Inherit patent class (ChLineShape)
    ChLineShape::Update(updater, coords);
}

// Set line geometry as a segment between two end point
void ChPointPointSegment::UpdateLineGeometry(const ChVector<>& endpoint1, const ChVector<>& endpoint2) {
	this->SetLineGeometry(std::static_pointer_cast<geometry::ChLine>(std::make_shared<geometry::ChLineSegment>(endpoint1, endpoint2)));
};

// Set line geometry as a coil between two end point
void ChPointPointSpring::UpdateLineGeometry(const ChVector<>& endpoint1, const ChVector<>& endpoint2) {
	auto linepath = std::make_shared<geometry::ChLinePath>();

	// Following part was copied from irrlicht::ChIrrTools::drawSpring()
	ChVector<> dist = endpoint2 - endpoint1;
	ChVector<> Vx, Vy, Vz;
	double length = dist.Length();
	ChVector<> dir = dist.GetNormalized();
    XdirToDxDyDz(dir, VECT_Y, Vx, Vy, Vz);

	ChMatrix33<> rel_matrix;
	rel_matrix.Set_A_axis(Vx, Vy, Vz);
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

}  // end namespace chrono
