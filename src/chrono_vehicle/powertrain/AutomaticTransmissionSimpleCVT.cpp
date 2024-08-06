//
// Created by Rainer Gericke on 06.08.24.
//

#include "AutomaticTransmissionSimpleCVT.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

AutomaticTransmissionSimpleCVT::AutomaticTransmissionSimpleCVT(const std::string& filename)
    : ChAutomaticTransmissionSimpleCVT("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

AutomaticTransmissionSimpleCVT::AutomaticTransmissionSimpleCVT(const rapidjson::Document& d)
    : ChAutomaticTransmissionSimpleCVT("") {
    Create(d);
}

void AutomaticTransmissionSimpleCVT::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read transmission data
    double eff = d["Gear Box"]["Efficiency"].GetDouble();
    double ratio_min = d["Gear Box"]["Minimum Gear Ratio"].GetDouble();
    double ratio_max = d["Gear Box"]["Maximum Gear Ratio"].GetDouble();
    double speed_begin = d["Gear Box"]["Driveshaft Speed Begin"].GetDouble();
    double speed_end = d["Gear Box"]["Driveshaft Speed End"].GetDouble();

    eff = std::clamp(eff, 0.8, 1.0);
    SetOperationRange(speed_begin, ratio_min, speed_end, ratio_max, eff);
}

}  // namespace vehicle
}  // namespace chrono