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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Generic 2-axle vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a generic rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "generic/Generic_Vehicle.h"

#include "generic/Generic_SolidAxle.h"
#include "generic/Generic_MultiLink.h"
#include "generic/Generic_DoubleWishbone.h"
#include "generic/Generic_HendricksonPRIMAXX.h"
#include "generic/Generic_MacPhersonStrut.h"

#include "generic/Generic_AntirollBarRSD.h"

#include "generic/Generic_Wheel.h"
#include "generic/Generic_RackPinion.h"
#include "generic/Generic_Driveline2WD.h"
#include "generic/Generic_BrakeSimple.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double     Generic_Vehicle::m_chassisMass = 995.0;                        // chassis sprung mass
const ChVector<> Generic_Vehicle::m_chassisCOM (0, 0, 0);                          // COM location
const ChVector<> Generic_Vehicle::m_chassisInertia(200.0, 500.0, 600.0);  // chassis inertia (roll,pitch,yaw)

const ChCoordsys<> Generic_Vehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_Vehicle::Generic_Vehicle(const bool fixed,
                                 SuspensionType suspType,
                                 VisualizationType wheelVis,
                                 ChMaterialSurfaceBase::ContactMethod contactMethod)
  : ChWheeledVehicle(contactMethod), m_suspType(suspType) {
  // -------------------------------------------
  // Create the chassis body
  // -------------------------------------------
  m_chassis = std::shared_ptr<ChBodyAuxRef>(m_system->NewBodyAuxRef());

  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(m_chassisInertia);
  m_chassis->SetBodyFixed(fixed);

  auto sphere = std::make_shared<ChSphereShape>();
  sphere->GetSphereGeometry().rad = 0.1;
  sphere->Pos = m_chassisCOM;
  m_chassis->AddAsset(sphere);

  m_system->Add(m_chassis);

  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------
  m_suspensions.resize(2);

  switch (m_suspType) {
  case SOLID_AXLE:
      m_suspensions[0] = std::make_shared<Generic_SolidAxle>("FrontSusp");
      m_suspensions[1] = std::make_shared<Generic_SolidAxle>("RearSusp");
    break;
  case MULTI_LINK:
      m_suspensions[0] = std::make_shared<Generic_MultiLink>("FrontSusp");
      m_suspensions[1] = std::make_shared<Generic_MultiLink>("RearSusp");
    break;
  case DOUBLE_WISHBONE:
      m_suspensions[0] = std::make_shared<Generic_DoubleWishbone>("Front suspension");
      m_suspensions[1] = std::make_shared<Generic_DoubleWishbone>("Rear suspension");
    break;
  case HENDRICKSON_PRIMAXX:
      m_suspensions[0] = std::make_shared<Generic_HendricksonPRIMAXX>("Front suspension");
      m_suspensions[1] = std::make_shared<Generic_HendricksonPRIMAXX>("Rear suspension");
    break;
  case MACPHERSON_STRUT:
      m_suspensions[0] = std::make_shared<Generic_MacPhersonStrut>("Front suspension");
      m_suspensions[1] = std::make_shared<Generic_MacPhersonStrut>("Rear suspension");
	break;
  }

  // --------------------------------
  // Create the antirollbar subsystem
  // --------------------------------
  ////if (m_suspensions[0]->IsIndependent()) {
  ////  m_antirollbars.resize(1);
  ////  m_antirollbars[0] = std::make_shared<Generic_AntirollBarRSD>("Antiroll Bar");
  ////}

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------
  m_steerings.resize(1);
  m_steerings[0] = std::make_shared<Generic_RackPinion>("Steering");

  // -----------------
  // Create the wheels
  // -----------------
  m_wheels.resize(4);
  m_wheels[0] = std::make_shared<Generic_Wheel>(wheelVis);
  m_wheels[1] = std::make_shared<Generic_Wheel>(wheelVis);
  m_wheels[2] = std::make_shared<Generic_Wheel>(wheelVis);
  m_wheels[3] = std::make_shared<Generic_Wheel>(wheelVis);

  // --------------------
  // Create the driveline
  // --------------------
  m_driveline = std::make_shared<Generic_Driveline2WD>();

  // -----------------
  // Create the brakes
  // -----------------
  m_brakes.resize(4);
  m_brakes[0] = std::make_shared<Generic_BrakeSimple>();
  m_brakes[1] = std::make_shared<Generic_BrakeSimple>();
  m_brakes[2] = std::make_shared<Generic_BrakeSimple>();
  m_brakes[3] = std::make_shared<Generic_BrakeSimple>();
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_Vehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem (specify the steering subsystem's frame
  // relative to the chassis reference frame).
  ChVector<> offset;
  switch (m_suspType) {
  case SOLID_AXLE:          offset = ChVector<>(2.1, 0, -0.02); break;
  case MULTI_LINK:          offset = ChVector<>(1.25, 0, -0.21); break;
  case DOUBLE_WISHBONE:     offset = ChVector<>(1.25, 0, -0.21); break;
  case HENDRICKSON_PRIMAXX: offset = ChVector<>(1.25, 0, -0.21); break;
  case MACPHERSON_STRUT:    offset = ChVector<>(1.25, 0, -0.21); break;
  }
  m_steerings[0]->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_suspensions[0]->Initialize(m_chassis, ChVector<>(1.6914, 0, 0), m_steerings[0]->GetSteeringLink());
  m_suspensions[1]->Initialize(m_chassis, ChVector<>(-1.6865, 0, 0), m_chassis);

  // Initialize the antiroll bar subsystem.
  ////if (m_antirollbars.size() == 1) {
  ////  m_antirollbars[0]->Initialize(m_chassis,
  ////                               ChVector<>(1.3, 0, 0.0),
  ////                               m_suspensions[0]->GetLeftBody(),
  ////                               m_suspensions[0]->GetRightBody());
  ////}

  // Initialize wheels
  m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
  m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
  m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

  // Initialize the driveline subsystem (RWD)
  std::vector<int> driven_susp(1, 1);
  m_driveline->Initialize(m_chassis, m_suspensions, driven_susp);

  // Initialize the four brakes
  m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
  m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
  m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
  m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Generic_Vehicle::GetSpringForce(const WheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
      return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetSpringForce(wheel_id.side());
  case MULTI_LINK:
      return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetSpringForce(wheel_id.side());
  case DOUBLE_WISHBONE:
      return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetSpringForce(wheel_id.side());
  case MACPHERSON_STRUT:
      return std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[wheel_id.axle()])->GetSpringForce(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetSpringLength(const WheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
      return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetSpringLength(wheel_id.side());
  case MULTI_LINK:
      return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetSpringLength(wheel_id.side());
  case DOUBLE_WISHBONE:
      return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetSpringLength(wheel_id.side());
  case MACPHERSON_STRUT:
      return std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[wheel_id.axle()])->GetSpringLength(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetSpringDeformation(const WheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
      return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetSpringDeformation(wheel_id.side());
  case MULTI_LINK:
      return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetSpringDeformation(wheel_id.side());
  case DOUBLE_WISHBONE:
      return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetSpringDeformation(wheel_id.side());
  case MACPHERSON_STRUT:
      return std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[wheel_id.axle()])->GetSpringDeformation(wheel_id.side());
  default:
    return -1;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Generic_Vehicle::GetShockForce(const WheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
      return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetShockForce(wheel_id.side());
  case MULTI_LINK:
      return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetShockForce(wheel_id.side());
  case DOUBLE_WISHBONE:
      return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetShockForce(wheel_id.side());
  case MACPHERSON_STRUT:
      return std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[wheel_id.axle()])->GetShockForce(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetShockLength(const WheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
      return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetShockLength(wheel_id.side());
  case MULTI_LINK:
      return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetShockLength(wheel_id.side());
  case DOUBLE_WISHBONE:
      return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetShockLength(wheel_id.side());
  case MACPHERSON_STRUT:
      return std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[wheel_id.axle()])->GetShockLength(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetShockVelocity(const WheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
      return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetShockVelocity(wheel_id.side());
  case MULTI_LINK:
      return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetShockVelocity(wheel_id.side());
  case DOUBLE_WISHBONE:
      return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetShockVelocity(wheel_id.side());
  case MACPHERSON_STRUT:
      return std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[wheel_id.axle()])->GetShockVelocity(wheel_id.side());
  default:
    return -1;
  }
}


// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Generic_Vehicle::LogHardpointLocations()
{
  GetLog().SetNumFormat("%7.3f");

  switch (m_suspType) {
  case SOLID_AXLE:
    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChSolidAxle>(m_suspensions[0])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChSolidAxle>(m_suspensions[1])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    break;
  case MULTI_LINK:
    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChMultiLink>(m_suspensions[0])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChMultiLink>(m_suspensions[1])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    break;
  case DOUBLE_WISHBONE:
    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[0])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[1])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    break;
  case MACPHERSON_STRUT:
    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[0])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChMacPhersonStrut>(m_suspensions[1])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    break;
  }

  GetLog() << "\n\n";

  GetLog().SetNumFormat("%g");
}


// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void Generic_Vehicle::DebugLog(int what)
{
  GetLog().SetNumFormat("%10.2f");

  if (what & OUT_SPRINGS)
  {
    GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [inch]       "
      << GetSpringLength(FRONT_LEFT) << "  "
      << GetSpringLength(FRONT_RIGHT) << "  "
      << GetSpringLength(REAR_LEFT) << "  "
      << GetSpringLength(REAR_RIGHT) << "\n";
    GetLog() << "Deformation [inch]  "
      << GetSpringDeformation(FRONT_LEFT) << "  "
      << GetSpringDeformation(FRONT_RIGHT) << "  "
      << GetSpringDeformation(REAR_LEFT) << "  "
      << GetSpringDeformation(REAR_RIGHT) << "\n";
    GetLog() << "Force [lbf]         "
      << GetSpringForce(FRONT_LEFT) << "  "
      << GetSpringForce(FRONT_RIGHT) << "  "
      << GetSpringForce(REAR_LEFT) << "  "
      << GetSpringForce(REAR_RIGHT) << "\n";
  }

  if (what & OUT_SHOCKS)
  {
    GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [inch]       "
      << GetShockLength(FRONT_LEFT) << "  "
      << GetShockLength(FRONT_RIGHT) << "  "
      << GetShockLength(REAR_LEFT) << "  "
      << GetShockLength(REAR_RIGHT) << "\n";
    GetLog() << "Velocity [inch/s]   "
      << GetShockVelocity(FRONT_LEFT) << "  "
      << GetShockVelocity(FRONT_RIGHT) << "  "
      << GetShockVelocity(REAR_LEFT) << "  "
      << GetShockVelocity(REAR_RIGHT) << "\n";
    GetLog() << "Force [lbf]         "
      << GetShockForce(FRONT_LEFT) << "  "
      << GetShockForce(FRONT_RIGHT) << "  "
      << GetShockForce(REAR_LEFT) << "  "
      << GetShockForce(REAR_RIGHT) << "\n";
  }

  if (what & OUT_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations();
  }

  GetLog().SetNumFormat("%g");
}
