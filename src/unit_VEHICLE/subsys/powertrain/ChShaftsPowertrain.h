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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Powertrain model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_SHAFTS_POWERTRAIN_H
#define CH_SHAFTS_POWERTRAIN_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChPowertrain.h"
#include "subsys/ChVehicle.h"

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

namespace chrono {

// Forward reference
class ChVehicle;


class CH_SUBSYS_API ChShaftsPowertrain : public ChPowertrain
{
public:

  ChShaftsPowertrain(ChVehicle* car,
                     const ChVector<>& dir_motor_block = ChVector<>(1,0,0));

  ~ChShaftsPowertrain() {}

  /// To be called after creation, to create all the wrapped ChShaft objects 
  /// and their constraints, torques etc. 
  void Initialize(ChSharedPtr<ChBody>  chassis,
                  ChSharedPtr<ChShaft> driveshaft);

  /// Return the current engine speed.
  virtual double GetMotorSpeed() const { return  m_crankshaft->GetPos_dt(); }

  /// Return the current engine torque.
  virtual double GetMotorTorque() const { return  m_engine->GetTorqueReactionOn1(); }

  /// Return the value of slippage in the torque converter.
  virtual double GetTorqueConverterSlippage() const { return m_torqueconverter->GetSlippage(); }

  /// Return the input torque to the torque converter.
  virtual double GetTorqueConverterInputTorque() const { return -m_torqueconverter->GetTorqueReactionOnInput(); }

  /// Return the output torque from the torque converter.
  virtual double GetTorqueConverterOutputTorque() const { return m_torqueconverter->GetTorqueReactionOnOutput(); }

  /// Return the current transmission gear
  virtual int GetCurrentTransmissionGear() const { return m_current_gear; }

  /// Use this function to set the mode of automatic transmission.
  virtual void SetDriveMode(ChPowertrain::DriveMode mmode);

  /// Use this function to shift from one gear to another.
  /// A zero latency shift is assumed.
  /// Note, index starts from 0.
  void SetSelectedGear(int igear);

  /// Use this to define the gear shift latency, in seconds.
  void SetGearShiftLatency(double ml) {m_gear_shift_latency= ml;}

  /// Use this to get the gear shift latency, in seconds.
  double GetGearShiftLatency(double ml) {return m_gear_shift_latency;}

  virtual void Update(double time, double throttle);

protected:

  /// Set up the gears, i.e. the transmission ratios of the various gears.
  /// A derived class must populate the vector gear_ratios, using the 0 index
  /// for reverse and 1,2,3,etc. for the forward gears.
  virtual void SetGearRatios(std::vector<double>& gear_ratios) = 0;

  /// Inertias of the component ChShaft objects.
  virtual double GetMotorBlockInertia() const = 0;
  virtual double GetCrankshaftInertia() const = 0;
  virtual double GetIngearShaftInertia() const = 0;

  /// Engine speed-torque map.
  virtual void SetEngineTorqueMap(ChSharedPtr<ChFunction_Recorder>& map) = 0;
  /// Engine speed-torque braking effect because of losses.
  virtual void SetEngineLossesMap(ChSharedPtr<ChFunction_Recorder>& map) = 0;

  /// Torque converter maps:
  /// capacity factor and torque ratio as functions of the speed ratio.
  virtual void SetTorqueConverterCapacityFactorMap(ChSharedPtr<ChFunction_Recorder>& map) = 0;
  virtual void SetTorqeConverterTorqueRatioMap(ChSharedPtr<ChFunction_Recorder>& map) = 0;

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

  friend class ChIrrGuiDriver;
};


} // end namespace chrono


#endif
