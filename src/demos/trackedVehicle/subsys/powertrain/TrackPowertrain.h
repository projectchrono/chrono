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
// Authors: Justin Madsen, Alessandro Tasora, Radu Serban
// =============================================================================
//
// Powertrain model template based on ChShaft objects, for tracked vehicles
//
// =============================================================================

#ifndef TRACKPOWERTRAIN_H
#define TRACKPOWERTRAIN_H

#include "subsys/ChApiSubsys.h"

#include "physics/ChShaftsGear.h" 
#include "physics/ChShaftsGearbox.h"
#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChShaftsClutch.h"
#include "physics/ChShaftsPlanetary.h"
#include "physics/ChShaftsBody.h"
#include "physics/ChShaftsTorsionSpring.h"
#include "physics/ChShaftsTorqueConverter.h"
#include "physics/ChShaftsMotor.h"
#include "physics/ChShaftsTorque.h"
#include "physics/ChShaftsThermalEngine.h"

// Forward reference
class TrackedVehicle;


namespace chrono {


/// Powertrain model template based on ChShaft objects, for tracked vehicles.
class CH_SUBSYS_API TrackPowertrain : public ChShared
{
public:

  enum DriveMode {
    FORWARD,
    NEUTRAL,
    REVERSE
  };

  TrackPowertrain(const std::string& name, 
    const ChVector<>& dir_motor_block = ChVector<>(1,0,0));

  ~TrackPowertrain() {}

  /// To be called after creation, to create all the wrapped ChShaft objects 
  /// and their constraints, torques etc. 
  void Initialize(ChSharedPtr<ChBody>  chassis,
                  ChSharedPtr<ChShaft> driveshaft);

				  
  // ------- Accessors
  /// return current drive mode.
  DriveMode GetDriveMode() { return m_drive_mode; }
				  
  /// Return the current engine speed.
  double GetMotorSpeed() const { return  m_crankshaft->GetPos_dt(); }

  /// Return the current engine torque.
  double GetMotorTorque() const { return  m_engine->GetTorqueReactionOn1(); }

  /// Return the value of slippage in the torque converter.
  double GetTorqueConverterSlippage() const { return m_torqueconverter->GetSlippage(); }

  /// Return the input torque to the torque converter.
  double GetTorqueConverterInputTorque() const { return -m_torqueconverter->GetTorqueReactionOnInput(); }

  /// Return the output torque from the torque converter.
  double GetTorqueConverterOutputTorque() const { return m_torqueconverter->GetTorqueReactionOnOutput(); }

  /// Return the current transmission gear
  int GetCurrentTransmissionGear() const { return m_current_gear; }

  /// Return the ouput torque from the powertrain.
  /// This is the torque that is passed to a vehicle system, thus providing the
  /// interface between the powertrain and vehcicle cosimulation modules.
  /// Since a ShaftsPowertrain is directly connected to the vehicle's driveline,
  /// this function returns 0.
  double GetOutputTorque() const { return 0; }

  // Accessors, from concrete class
  double GetMotorBlockInertia() const      { return m_motorblock_inertia; }
  double GetCrankshaftInertia() const      { return m_crankshaft_inertia; }
  double GetIngearShaftInertia() const     { return m_ingear_shaft_inertia; }
  
  /// Use this function to set the mode of automatic transmission.
  void SetDriveMode(DriveMode mmode);

  /// Use this function to shift from one gear to another.
  /// A zero latency shift is assumed.
  /// Note, index starts from 0.
  void SetSelectedGear(int igear);

  /// Use this to define the gear shift latency, in seconds.
  void SetGearShiftLatency(double ml) {m_gear_shift_latency= ml;}

  /// Use this to get the gear shift latency, in seconds.
  double GetGearShiftLatency(double ml) {return m_gear_shift_latency;}

  /// Update the state of this powertrain system at the current time.
  /// The powertrain system is provided the current driver throttle input, a
  /// value in the range [0,1], and the current angular speed of the transmission
  /// shaft (from the driveline).
  void Update(
    double time,       ///< [in] current time
    double throttle,   ///< [in] current throttle input [0,1]
    double shaft_speed ///< [in] current angular speed of the transmission shaft
    );

  /// Advance the state of this powertrain system by the specified time step.
  /// Since the state of a ShaftsPowertrain is advanced as part of the vehicle
  /// state, this function does nothing.
  void Advance(double step) {}

  // from concrete class
  void SetGearRatios(std::vector<double>& gear_ratios);
  
  void SetEngineTorqueMap(ChSharedPtr<ChFunction_Recorder>& map);
  void SetEngineLossesMap(ChSharedPtr<ChFunction_Recorder>& map);
  void SetTorqueConverterCapacityFactorMap(ChSharedPtr<ChFunction_Recorder>& map);
  void SetTorqeConverterTorqueRatioMap(ChSharedPtr<ChFunction_Recorder>& map);
  
private:

  ChSharedPtr<ChShaftsBody>             m_motorblock_to_body;
  ChSharedPtr<ChShaft>                  m_motorblock;
  ChSharedPtr<ChShaftsThermalEngine>    m_engine;
  ChSharedPtr<ChShaftsThermalEngine>    m_engine_losses;
  ChSharedPtr<ChShaft>                  m_crankshaft;
  ChSharedPtr<ChShaftsTorqueConverter>  m_torqueconverter;
  ChSharedPtr<ChShaft>                  m_shaft_ingear;
  ChSharedPtr<ChShaftsGearbox>          m_gears;

  int m_current_gear;
  std::vector<double> m_gear_ratios;

  ChVector<> m_dir_motor_block;

  double m_last_time_gearshift;
  double m_gear_shift_latency;

  // friend class ChIrrGuiDriver;
  
  // ChPowertrain
  DriveMode m_drive_mode;
  
  // concrete class variables
  // Shaft inertias.
  static const double  m_motorblock_inertia;
  static const double  m_crankshaft_inertia;
  static const double  m_ingear_shaft_inertia;



};

} // end namespace chrono



#endif
