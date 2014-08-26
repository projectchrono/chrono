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
// HMMWV9 powertrain model based on ChShaft objects.
//
// =============================================================================

#ifndef HMMWV9_POWERTRAIN_H
#define HMMWV9_POWERTRAIN_H

#include "subsys/powertrain/ChShaftsPowertrain.h"
#include "subsys/ChVehicle.h"


namespace hmmwv9 {

// Forward reference
class HMMWV9_Vehicle;


class HMMWV9_Powertrain : public chrono::ChShaftsPowertrain {
public:

  HMMWV9_Powertrain(chrono::ChVehicle* car);

  ~HMMWV9_Powertrain() {}


  virtual void SetGearRatios(std::vector<double>& gear_ratios);

  virtual double GetMotorBlockInertia() const      { return m_motorblock_inertia; }
  virtual double GetCrankshaftInertia() const      { return m_crankshaft_inertia; }
  virtual double GetIngearShaftInertia() const     { return m_ingear_shaft_inertia; }
  virtual double GetOutgearShaftInertia() const    { return m_outgear_shaft_inertia; }
  virtual double GetDifferentialBoxInertia() const { return m_differentialbox_inertia; }

  virtual double GetConicalGearRatio() const       { return m_conicalgear_ratio; }
  virtual double GetDifferentialRatio() const      { return m_differential_ratio; }

  virtual void SetEngineTorqueMap(chrono::ChSharedPtr<chrono::ChFunction_Recorder>& map);
  virtual void SetTorqueConverterCapacityFactorMap(chrono::ChSharedPtr<chrono::ChFunction_Recorder>& map);
  virtual void SetTorqeConverterTorqueRatioMap(chrono::ChSharedPtr<chrono::ChFunction_Recorder>& map);

private:

  // Shaft inertias.
  static const double  m_motorblock_inertia;
  static const double  m_crankshaft_inertia;
  static const double  m_ingear_shaft_inertia;
  static const double  m_outgear_shaft_inertia;
  static const double  m_differentialbox_inertia;

  // Gear ratios.
  static const double  m_conicalgear_ratio;
  static const double  m_differential_ratio;

};


} // end namespace hmmwv9


#endif
