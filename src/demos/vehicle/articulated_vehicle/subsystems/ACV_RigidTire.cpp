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
// Rigid wheel subsystem for the articulated chassis vehicle
//
// =============================================================================

#include "subsystems/ACV_RigidTire.h"

using namespace chrono;
using namespace chrono::vehicle;

// Static variables
const double ACV_RigidTire::m_radius = 0.42;
const double ACV_RigidTire::m_width = 0.3;
const double ACV_RigidTire::m_mass = 35.0;
const ChVector<> ACV_RigidTire::m_inertia(3.0, 6.0, 3.0);

ACV_RigidTire::ACV_RigidTire(const std::string& name) : ChRigidTire(name) {}

void ACV_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}
