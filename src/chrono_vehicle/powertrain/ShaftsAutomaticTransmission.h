// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// ChShaft-based automatic transmission model constructed with data from file
// (JSON format).
//
// =============================================================================

#ifndef SHAFTS_AUTOMATIC_TRANSMISSION_H
#define SHAFTS_AUTOMATIC_TRANSMISSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/powertrain/ChShaftsAutomaticTransmission.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Shafts-based automatic transmission subsystem (specified through JSON file).
class CH_VEHICLE_API ShaftsAutomaticTransmission : public ChShaftsAutomaticTransmission {
  public:
    ShaftsAutomaticTransmission(const std::string& filename);
    ShaftsAutomaticTransmission(const rapidjson::Document& d);
    ~ShaftsAutomaticTransmission() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    virtual double GetTransmissionBlockInertia() const { return m_transmissionblock_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }

    virtual double GetUpshiftRPM() const override { return m_upshift_RPM; }
    virtual double GetDownshiftRPM() const override { return m_downshift_RPM; }

    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) override;

  private:
    struct MapData {
        unsigned int m_n;
        std::vector<double> m_x;
        std::vector<double> m_y;
    };

    virtual void Create(const rapidjson::Document& d) override;

    void ReadMapData(const rapidjson::Value& a, MapData& map_data);
    void SetMapData(const MapData& map_data, double x_factor, double y_factor, std::shared_ptr<ChFunction_Recorder>& map);

    double m_transmissionblock_inertia;
    double m_ingear_shaft_inertia;

    double m_rev_gear;
    std::vector<double> m_fwd_gear;

    double m_upshift_RPM;
    double m_downshift_RPM;

    MapData m_tc_capacity_factor;
    MapData m_tc_torque_ratio;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
