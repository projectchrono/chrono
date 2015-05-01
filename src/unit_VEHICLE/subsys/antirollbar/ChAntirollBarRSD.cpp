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
// Base class for an anti-roll bar template modeled two arms connected with a
// revolute spring-damper.
// Derived from ChAntirollBar, but still and abstract base class.
//
// The anti-roll bar subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The subsystem reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// =============================================================================

#include "subsys/antirollbar/ChAntirollBarRSD.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChAntirollBarRSD::ChAntirollBarRSD(const std::string& name)
: ChAntirollBar(name)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                                  const ChVector<>&          location,
                                  ChSharedPtr<ChBody>        susp_body_left,
                                  ChSharedPtr<ChBody>        susp_body_right)
{
  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> suspension_to_abs(location);
  suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  //// TODO
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::LogConstraintViolations()
{
  // Chassis revolute joint
  {
    ChMatrix<> *C = m_revolute_ch->GetC();
    GetLog() << "Chassis revolute          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Central revolute joint
  {
    ChMatrix<> *C = m_revolute->GetC();
    GetLog() << "Central revolute          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Distance constraints (droplinks)
  GetLog() << "Droplink distance (left)  ";
  GetLog() << "  " << m_link_left->GetCurrentDistance() - m_link_left->GetImposedDistance() << "\n";
  GetLog() << "Droplink distance (right) ";
  GetLog() << "  " << m_link_right->GetCurrentDistance() - m_link_right->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::AddVisualizationArm(ChSharedPtr<ChBody>  arm,
                                           const ChVector<>&    pt_1,
                                           const ChVector<>&    pt_2,
                                           const ChVector<>&    pt_3,
                                           double               radius)
{
  //// TODO
}


}  // end namespace chrono
