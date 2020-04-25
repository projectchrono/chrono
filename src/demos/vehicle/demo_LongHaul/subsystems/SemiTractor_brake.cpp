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
// Authors: Rainer Gericke
// =============================================================================
//
// Kraz 64431 28t simple brake model.
//
// =============================================================================

#include "subsystems/SemiTractor_brake.h"

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double SemiTractor_brake::m_maxtorque = 10000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SemiTractor_brake::SemiTractor_brake(const std::string& name) : ChBrakeSimple(name) {}
