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
// ChShaft-based powertrain model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef SHAFTS_POWERTRAIN_H
#define SHAFTS_POWERTRAIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/powertrain/ChShaftsPowertrain.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_VEHICLE_API ShaftsPowertrain : public ChShaftsPowertrain
{
public:

  ShaftsPowertrain(const std::string& filename);
  ShaftsPowertrain(const rapidjson::Document& d);
  ~ShaftsPowertrain() {}

  virtual void SetGearRatios(std::vector<double>& gear_ratios);

  virtual double GetMotorBlockInertia() const { return m_motorblock_inertia; }
  virtual double GetCrankshaftInertia() const { return m_crankshaft_inertia; }
  virtual double GetIngearShaftInertia() const { return m_ingear_shaft_inertia; }

  virtual void SetEngineTorqueMap(ChSharedPtr<ChFunction_Recorder>& map) { SetMapData(m_engine_torque, map); }
  virtual void SetEngineLossesMap(ChSharedPtr<ChFunction_Recorder>& map) { SetMapData(m_engine_losses, map); }
  virtual void SetTorqueConverterCapacityFactorMap(ChSharedPtr<ChFunction_Recorder>& map) { SetMapData(m_tc_capacity_factor, map); }
  virtual void SetTorqeConverterTorqueRatioMap(ChSharedPtr<ChFunction_Recorder>& map) { SetMapData(m_tc_torque_ratio, map); }

private:

  struct MapData {
    unsigned int m_n;
    std::vector<double> m_x;
    std::vector<double> m_y;
  };

  void Create(const rapidjson::Document& d);

  void ReadMapData(const rapidjson::Value& a, MapData& map_data);
  void SetMapData(const MapData& map_data, ChSharedPtr<ChFunction_Recorder>& map);

  double m_motorblock_inertia;
  double m_crankshaft_inertia;
  double m_ingear_shaft_inertia;

  double m_rev_gear;
  std::vector<double> m_fwd_gear;

  MapData m_engine_torque;
  MapData m_engine_losses;
  MapData m_tc_capacity_factor;
  MapData m_tc_torque_ratio;
};


} // end namespace chrono


#endif
