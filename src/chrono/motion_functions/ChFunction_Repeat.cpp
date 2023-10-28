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

#include "chrono/motion_functions/ChFunction_Repeat.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Repeat)

ChFunction_Repeat::ChFunction_Repeat(std::shared_ptr<ChFunction> func, double start, double length, double phase)
    : fa(func), window_start(start), window_length(length), window_phase(phase) {}

ChFunction_Repeat::ChFunction_Repeat(const ChFunction_Repeat& other) {
    window_start = other.window_start;
    window_length = other.window_length;
    window_phase = other.window_phase;
    fa = std::shared_ptr<ChFunction>(other.fa->Clone());
}

double ChFunction_Repeat::Get_y(double x) const {
    return fa->Get_y(this->window_start + fmod(x + this->window_phase, this->window_length));
}

void ChFunction_Repeat::Estimate_x_range(double& xmin, double& xmax) const {
    fa->Estimate_x_range(xmin, xmax);
}

void ChFunction_Repeat::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChFunction_Repeat>();
    // serialize parent class
    ChFunction::ArchiveOut(archive);
    // serialize all member data:
    archive << CHNVP(fa);
    archive << CHNVP(window_start);
    archive << CHNVP(window_length);
    archive << CHNVP(window_phase);
}

void ChFunction_Repeat::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/ archive.VersionRead<ChFunction_Repeat>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive);
    // stream in all member data:
    archive >> CHNVP(fa);
    archive >> CHNVP(window_start);
    archive >> CHNVP(window_length);
    archive >> CHNVP(window_phase);
}

}  // end namespace chrono
