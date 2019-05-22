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

#include "chrono/solver/ChVariables.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChVariables)

ChVariables::ChVariables() : disabled(false), ndof(0), qb(nullptr), fb(nullptr), offset(0) {}

ChVariables::ChVariables(int m_ndof) : disabled(false), ndof(m_ndof), offset(0) {
    if (ndof > 0) {
        qb = new ChMatrixDynamic<>(Get_ndof(), 1);
        fb = new ChMatrixDynamic<>(Get_ndof(), 1);
        qb->Reset();
        fb->Reset();
    } else {
        qb = fb = nullptr;
    }
}

ChVariables::~ChVariables() {
    delete qb;
    delete fb;
}

ChVariables& ChVariables::operator=(const ChVariables& other) {
    if (&other == this)
        return *this;

    this->disabled = other.disabled;

    if (other.qb) {
        if (!qb)
            qb = new ChMatrixDynamic<>;
        qb->CopyFromMatrix(*other.qb);
    } else {
        delete qb;
        qb = nullptr;
    }

    if (other.fb) {
        if (!fb)
            fb = new ChMatrixDynamic<>;
        fb->CopyFromMatrix(*other.fb);
    } else {
        delete fb;
        fb = nullptr;
    }

    this->ndof = other.ndof;
    this->offset = other.offset;

    return *this;
}



}  // end namespace chrono
