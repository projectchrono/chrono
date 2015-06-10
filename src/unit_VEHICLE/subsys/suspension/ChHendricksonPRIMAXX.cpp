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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Base class for a Hendrickson PRIMAXX EX suspension.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"

#include "subsys/suspension/ChHendricksonPRIMAXX.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChHendricksonPRIMAXX::m_pointNames[] = {
  "SPINDLE "
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChHendricksonPRIMAXX::ChHendricksonPRIMAXX(const std::string& name)
: ChSuspension(name)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                                  const ChVector<>&          location,
                                  ChSharedPtr<ChBody>        tierod_body)
{
  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> suspension_to_abs(location);
  suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // Transform all points to absolute frame and initialize left side.
  std::vector<ChVector<> > points(NUM_POINTS);

  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    points[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
  }

  InitializeSide(LEFT, chassis, tierod_body, points);

  // Transform all points to absolute frame and initialize right side.
  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    rel_pos.y = -rel_pos.y;
    points[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
  }

  InitializeSide(RIGHT, chassis, tierod_body, points);
}

void ChHendricksonPRIMAXX::InitializeSide(ChVehicleSide                   side,
                                      ChSharedPtr<ChBodyAuxRef>       chassis,
                                      ChSharedPtr<ChBody>             tierod_body,
                                      const std::vector<ChVector<> >& points)
{
  std::string suffix = (side == LEFT) ? "_L" : "_R";

  // Chassis orientation (expressed in absolute frame)
  // Recall that the suspension reference frame is aligned with the chassis.
  ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

  // Create and initialize spindle body (same orientation as the chassis)
  m_spindle[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
  m_spindle[side]->SetPos(points[SPINDLE]);
  m_spindle[side]->SetRot(chassisRot);
  m_spindle[side]->SetMass(getSpindleMass());
  m_spindle[side]->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle(m_spindle[side], getSpindleRadius(), getSpindleWidth());
  chassis->GetSystem()->AddBody(m_spindle[side]);


  //// TODO


  // Create and initialize the axle shaft and its connection to the spindle. Note that the
  // spindle rotates about the Y axis.
  m_axle[side] = ChSharedPtr<ChShaft>(new ChShaft);
  m_axle[side]->SetNameString(m_name + "_axle" + suffix);
  m_axle[side]->SetInertia(getAxleInertia());
  chassis->GetSystem()->Add(m_axle[side]);

  m_axle_to_spindle[side] = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
  m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
  m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
  chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogHardpointLocations(const ChVector<>& ref,
                                                 bool              inches)
{
  double unit = inches ? 1 / 0.0254 : 1.0;

  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

    GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x << "  " << pos.y << "  " << pos.z << "\n";
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogConstraintViolations(ChVehicleSide side)
{
  // Revolute joints
  {
    ChMatrix<>* C = m_revolute[side]->GetC();
    GetLog() << "Spindle revolute      ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  //// TODO
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                                   double          radius,
                                                   double          width)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
  cyl->GetCylinderGeometry().rad = radius;
  spindle->AddAsset(cyl);
}


} // end namespace chrono
