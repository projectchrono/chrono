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
// Authors: Andrea Favali
// =============================================================================

#include "chrono/fea/ChGaussPoint.h"

namespace chrono {
namespace fea {

ChGaussPoint::ChGaussPoint(int number, ChVector<>* coord, double weight)
    : m_number(number), m_local_coordinates(*coord), m_weight(weight), m_coordinates(nullptr) {
    MatrB = new ChMatrixDynamic<>(1, 1);
}

ChGaussPoint::~ChGaussPoint() {
    delete m_coordinates;
}

ChVector<> ChGaussPoint::GetCoordinates() const {
    if (m_coordinates) {
        return *m_coordinates;
    } else {
        return m_local_coordinates;
    }
}

void ChGaussPoint::SetCoordinates(const ChVector<>& c) {
    if (m_coordinates) {
        *m_coordinates = c;
    } else {
        m_coordinates = new ChVector<>(c);
    }
}

}  // end namespace fea
}  // end namespace chrono
