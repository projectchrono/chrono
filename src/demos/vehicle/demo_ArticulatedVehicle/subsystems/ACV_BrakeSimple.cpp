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
// Authors: Radu Serban
// =============================================================================
//
// Generic simple brake model.
//
// =============================================================================

#include "subsystems/ACV_BrakeSimple.h"

using namespace chrono;
using namespace chrono::vehicle;

const double ACV_BrakeSimple::m_maxtorque = 4000;

ACV_BrakeSimple::ACV_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}
