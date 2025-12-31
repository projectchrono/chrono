//
// Created by Rainer Gericke on 06.08.24.
//

#ifndef AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H
#define AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleCVT.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// Automatic transmission model template based on a simple gear-shifting model (specified through JSON file).
class CH_VEHICLE_API AutomaticTransmissionSimpleCVT : public ChAutomaticTransmissionSimpleCVT {
  public:
    AutomaticTransmissionSimpleCVT(const std::string& filename);
    AutomaticTransmissionSimpleCVT(const rapidjson::Document& d);
    ~AutomaticTransmissionSimpleCVT() {}

  private:
    virtual void Create(const rapidjson::Document& d) override;
};

}  // namespace vehicle
}  // namespace chrono

#endif  // AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H
