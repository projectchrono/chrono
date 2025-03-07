// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Interactive driver for a suspension test rig.
// Independent of keuboard event handler.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriver.h"

namespace chrono {
namespace vehicle {

ChSuspensionTestRigInteractiveDriver::ChSuspensionTestRigInteractiveDriver()
    : m_crt_axle(0), m_displ_delta(1.0 / 100), m_steering_delta(1.0 / 250) {}

double ChSuspensionTestRigInteractiveDriver::GetLeft() {
    return m_displLeft[m_crt_axle];
}

double ChSuspensionTestRigInteractiveDriver::GetRight() {
    return m_displRight[m_crt_axle];
}

void ChSuspensionTestRigInteractiveDriver::NextAxle() {
    m_crt_axle = (m_crt_axle + 1) % m_naxles;
}

void ChSuspensionTestRigInteractiveDriver::PreviousAxle() {
    m_crt_axle = (m_crt_axle == 0) ? m_naxles - 1 : m_crt_axle - 1;
}

void ChSuspensionTestRigInteractiveDriver::IncrementLeft() {
    SetDisplacementLeft(m_crt_axle, m_displLeft[m_crt_axle] + m_displ_delta);
}

void ChSuspensionTestRigInteractiveDriver::DecrementLeft() {
    SetDisplacementLeft(m_crt_axle, m_displLeft[m_crt_axle] - m_displ_delta);
}

void ChSuspensionTestRigInteractiveDriver::IncrementRight() {
    SetDisplacementRight(m_crt_axle, m_displRight[m_crt_axle] + m_displ_delta);
}

void ChSuspensionTestRigInteractiveDriver::DecrementRight() {
    SetDisplacementRight(m_crt_axle, m_displRight[m_crt_axle] - m_displ_delta);
}

void ChSuspensionTestRigInteractiveDriver::IncrementSteering() {
    SetSteering(m_steering + m_steering_delta);
}

void ChSuspensionTestRigInteractiveDriver::DecrementSteering() {
    SetSteering(m_steering - m_steering_delta);
}

}  // end namespace vehicle
}  // end namespace chrono
