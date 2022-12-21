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
// HMGenericMWV wheel subsystem
//
// =============================================================================

#include "subsystems/ACV_Wheel.h"

using namespace chrono;
using namespace chrono::vehicle;

// Static variables
const double ACV_Wheel::m_mass = 18.0;
const ChVector<> ACV_Wheel::m_inertia(0.24, 0.42, 0.24);
const double ACV_Wheel::m_radius = 0.3099;
const double ACV_Wheel::m_width = 0.235;

ACV_Wheel::ACV_Wheel(const std::string& name) : ChWheel(name) {}
