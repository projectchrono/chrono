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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/geometry/ChLinePath.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinePath)

ChLinePath::ChLinePath(const ChLinePath& source) : ChLine(source) {
    lines = source.lines;
    end_times = source.end_times;
    durations = source.durations;
}

double ChLinePath::Length(int sampling) const {
    double tot = 0;
    for (int i = 0; i < lines.size(); ++i) {
        tot += lines[i]->Length(sampling);
    }
    return tot;
}

void ChLinePath::Evaluate(ChVector<>& pos, const double parU) const {
    if (lines.size() == 0)
        return;

    double u = parU;

    // wrap u if it is a closed loop.
    if (this->closed)
        u = fmod(parU, this->GetPathDuration());

    double uA = 0;
    // Search sub line covering the parU
    // (brute force search.. assuming a limited number of added lines, it is ok anyway.)
    int i;
    for (i = 0; i < lines.size(); ++i) {
        if (u <= end_times[i])
            break;
    }
    if (i == lines.size())  // beyond end
        i -= 1;
    if (i > 0)
        uA = end_times[i - 1];

    double local_U = (u - uA) / durations[i];
    lines[i]->Evaluate(pos, local_U);
}

void ChLinePath::SetSubLineDurationN(size_t n, double mduration) {
    durations[n] = mduration;

    double last_t = 0;
    if (n > 0)
        last_t = end_times[n - 1];
    for (size_t i = n; i < end_times.size(); ++i) {
        last_t += durations[n];
        end_times[n] = last_t;
    }
}

void ChLinePath::AddSubLine(std::shared_ptr<ChLine> mline, double duration) {
    lines.push_back(mline);
    durations.push_back(0);
    end_times.push_back(0);
    SetSubLineDurationN(lines.size() - 1, duration);
}

void ChLinePath::AddSubLine(ChLine& mline, double duration) {
    std::shared_ptr<ChLine> pline((ChLine*)mline.Clone());
    AddSubLine(pline, duration);
}

void ChLinePath::InsertSubLine(size_t n, std::shared_ptr<ChLine> mline, double duration) {
    lines.insert(lines.begin() + n, mline);
    durations.push_back(0);
    end_times.push_back(0);
    // force recompute following end times:
    SetSubLineDurationN(n, duration);
}

void ChLinePath::InsertSubLine(size_t n, ChLine& mline, double duration) {
    std::shared_ptr<ChLine> pline((ChLine*)mline.Clone());
    InsertSubLine(n, pline, duration);
}

void ChLinePath::EraseSubLine(size_t n) {
    lines.erase(lines.begin() + n);
    durations.erase(durations.begin() + n);
    end_times.pop_back();
    // force recompute all end times:
    if (lines.size())
        SetSubLineDurationN(0, durations[0]);
}

double ChLinePath::GetPathDuration() const {
    if (end_times.size())
        return end_times.back();
    return 0;
}

void ChLinePath::SetPathDuration(double mUduration) {
    double factor = mUduration / GetPathDuration();
    double last_t = 0;
    for (size_t i = 0; i < end_times.size(); ++i) {
        durations[i] *= factor;
        last_t += durations[i];
        end_times[i] = last_t;
    }
}

double ChLinePath::GetContinuityMaxError() const {
    double maxerr = 0;
    for (size_t i = 1; i < lines.size(); ++i) {
        std::shared_ptr<ChLine> prec_line = lines[i - 1];
        std::shared_ptr<ChLine> next_line = lines[i];
        double gap = (prec_line->GetEndB() - next_line->GetEndA()).Length();
        if (gap > maxerr)
            maxerr = gap;
    }
    if (this->closed) {
        double gap = (lines.back()->GetEndA() - lines.front()->GetEndB()).Length();
        if (gap > maxerr)
            maxerr = gap;
    }
    return maxerr;
}

void ChLinePath::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinePath>();
    // serialize parent class
    ChLine::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(lines);
    marchive << CHNVP(end_times);
    marchive << CHNVP(durations);
}

void ChLinePath::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinePath>();
    // deserialize parent class
    ChLine::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(lines);
    marchive >> CHNVP(end_times);
    marchive >> CHNVP(durations);
}

}  // end namespace geometry
}  // end namespace chrono
