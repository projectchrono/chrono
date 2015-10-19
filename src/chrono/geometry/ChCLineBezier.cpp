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
// Geometric object representing a piecewise cubic Bezier curve in 3D.
//
// =============================================================================

#include <cmath>

#include "chrono/geometry/ChCLineBezier.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLineBezier> a_registration_ChLineBezier;

ChLineBezier::ChLineBezier(ChBezierCurve* path) : m_own_data(false), m_path(path) {
    complexityU = static_cast<int>(m_path->getNumPoints());
}

ChLineBezier::ChLineBezier(const std::string& filename) {
    m_path = ChBezierCurve::read(filename);
    m_own_data = true;
    complexityU = static_cast<int>(m_path->getNumPoints());
}

ChLineBezier::ChLineBezier(const ChLineBezier& source) {
    m_path = source.m_path;
    m_own_data = false;
    complexityU = source.complexityU;
}

ChLineBezier::ChLineBezier(const ChLineBezier* source) {
    m_path = source->m_path;
    m_own_data = false;
    complexityU = source->complexityU;
}

ChLineBezier::~ChLineBezier() {
    if (m_own_data)
        delete m_path;
}

void ChLineBezier::Evaluate(Vector& pos, const double parU, const double parV, const double parW) {
    double par = ChClamp(parU, 0.0, 1.0);
    size_t numIntervals = m_path->getNumPoints() - 1;
    double epar = par * numIntervals;
    size_t i = static_cast<size_t>(std::floor(par * numIntervals));
    ChClampValue(i, size_t(0), numIntervals - 1);
    double t = epar - (double)i;

    pos = m_path->eval(i, t);
}

}  // end of namespace geometry
}  // end of namespace chrono
