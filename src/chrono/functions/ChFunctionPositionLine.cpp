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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPositionLine)

static const double FD_STEP = 1e-4;

ChFunctionPositionLine::ChFunctionPositionLine() {
    // default trajectory is a segment
    this->trajectory_line = chrono_types::make_shared<ChLineSegment>();

    // default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunctionRamp>(0, 1.);
}

ChFunctionPositionLine::ChFunctionPositionLine(const ChFunctionPositionLine& other) {
    // trajectory_line = other.trajectory_line;
    trajectory_line =
        std::shared_ptr<ChLine>((ChLine*)other.trajectory_line->Clone());  // deep copy

    // space_fx = other.space_fx;
    space_fx = std::shared_ptr<ChFunction>(other.space_fx->Clone());  // deep copy
}

ChFunctionPositionLine::~ChFunctionPositionLine() {}

ChVector3d ChFunctionPositionLine::Get_p(double s) const {
    double u = space_fx->GetVal(s);
    return trajectory_line->Evaluate(u);
}

ChVector3d ChFunctionPositionLine::Get_p_ds(double s) const {
    double u = space_fx->GetVal(s);
    double du_ds = space_fx->GetDer(s);
    auto dp_du = trajectory_line->GetTangent(u);
    return dp_du * du_ds;
}

ChVector3d ChFunctionPositionLine::Get_p_dsds(double s) const {
    // note: same as not implementing the function and let the fallback default BDF numerical differentiation do similar
    // computation... to remove?
    double tstep = FD_STEP;
    double tr_time = s;
    double tr_timeB = s + tstep;
    double tr_timeA = s - tstep;

    auto result = trajectory_line->Evaluate(tr_time);
    auto resultA = trajectory_line->Evaluate(tr_timeA);
    auto resultB = trajectory_line->Evaluate(tr_timeB);

    return (resultA + resultB - result * 2) * (4 / pow(2 * tstep, 2));
}

void ChFunctionPositionLine::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionPosition>();
    // serialize parent class
    ChFunctionPosition::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(trajectory_line);
    marchive << CHNVP(space_fx);
}

void ChFunctionPositionLine::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChFunctionPositionLine>();
    // deserialize parent class
    ChFunctionPosition::ArchiveIn(marchive);
    // deserialize all member data:
    marchive >> CHNVP(trajectory_line);
    marchive >> CHNVP(space_fx);
}

}  // end namespace chrono