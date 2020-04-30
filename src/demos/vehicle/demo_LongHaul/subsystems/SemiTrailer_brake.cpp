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
// Krone ProfiLiner SP5 simple brake model.
//
// =============================================================================

#include "subsystems/SemiTrailer_brake.h"

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double SemiTrailer_brake::m_maxtorque = 13000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SemiTrailer_brake::SemiTrailer_brake(const std::string& name) : ChBrakeSimple(name) {}
