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

#include <cmath>

#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPositionLine)

static const double FD_STEP = 1e-4;

ChFunctionPositionLine::ChFunctionPositionLine() {
    // default trajectory is a segment
    this->m_trajectory_line = chrono_types::make_shared<ChLineSegment>();

    // default s(t) function. User will provide better fx.
    m_space_fun = chrono_types::make_shared<ChFunctionRamp>(0, 1.);
}

ChFunctionPositionLine::ChFunctionPositionLine(std::shared_ptr<ChLine> line) {
    SetLine(line);
}

ChFunctionPositionLine::ChFunctionPositionLine(const ChFunctionPositionLine& other) {
    // m_trajectory_line = other.m_trajectory_line;
    m_trajectory_line = std::shared_ptr<ChLine>((ChLine*)other.m_trajectory_line->Clone());  // deep copy

    // m_space_fun = other.m_space_fun;
    m_space_fun = std::shared_ptr<ChFunction>(other.m_space_fun->Clone());  // deep copy
}

ChFunctionPositionLine::~ChFunctionPositionLine() {}

ChVector3d ChFunctionPositionLine::GetPos(double s) const {
    double u = m_space_fun->GetVal(s);
    return m_trajectory_line->Evaluate(u);
}

ChVector3d ChFunctionPositionLine::GetLinVel(double s) const {
    double u = m_space_fun->GetVal(s);
    double du_ds = m_space_fun->GetDer(s);
    auto dp_du = m_trajectory_line->GetTangent(u);
    return dp_du * du_ds;
}

ChVector3d ChFunctionPositionLine::GetLinAcc(double s) const {
    // note: same as not implementing the function and let the fallback default BDF numerical differentiation do similar
    // computation... to remove?
    double tstep = FD_STEP;
    double tr_time = s;
    double tr_timeB = s + tstep;
    double tr_timeA = s - tstep;

    auto result = m_trajectory_line->Evaluate(tr_time);
    auto resultA = m_trajectory_line->Evaluate(tr_timeA);
    auto resultB = m_trajectory_line->Evaluate(tr_timeB);

    return (resultA + resultB - result * 2) * (4 / std::pow(2 * tstep, 2));
}

void ChFunctionPositionLine::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionPosition>();
    // serialize parent class
    ChFunctionPosition::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_trajectory_line);
    archive_out << CHNVP(m_space_fun);
}

void ChFunctionPositionLine::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionPositionLine>();
    // deserialize parent class
    ChFunctionPosition::ArchiveIn(archive_in);
    // deserialize all member data:
    archive_in >> CHNVP(m_trajectory_line);
    archive_in >> CHNVP(m_space_fun);
}

}  // end namespace chrono