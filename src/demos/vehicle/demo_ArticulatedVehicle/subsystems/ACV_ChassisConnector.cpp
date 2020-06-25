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
// Actuated articulation between front and rear chassis
//
// =============================================================================

#include "subsystems/ACV_ChassisConnector.h"

using namespace chrono;
using namespace chrono::vehicle;

const double ACV_ChassisConnector::m_maxangle = CH_C_PI / 6;

ACV_ChassisConnector::ACV_ChassisConnector(const std::string& name) : ChChassisConnectorArticulated(name) {}
