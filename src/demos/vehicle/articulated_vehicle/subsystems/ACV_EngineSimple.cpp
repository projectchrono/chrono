// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Generic engine model for a vehicle with a trivial speed-torque curve
//
// =============================================================================

#include "subsystems/ACV_EngineSimple.h"

using namespace chrono;
using namespace chrono::vehicle;

const double ACV_EngineSimple::m_max_power = 110000;
const double ACV_EngineSimple::m_max_torque = 300.0;
const double ACV_EngineSimple::m_max_speed = 4000;

ACV_EngineSimple::ACV_EngineSimple(const std::string& name) : ChEngineSimple(name) {}
