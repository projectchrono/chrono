//
// Created by Rainer Gericke on 18.07.24.
//

#ifndef CHRONO_CH_XYCHART_INFO_H
#define CHRONO_CH_XYCHART_INFO_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChXYChartInfo {
  public:
    ChXYChartInfo();
    ~ChXYChartInfo();
    bool DataComplete();

    std::string title;
    std::vector<float> xdata, ydata;
    std::string xlabel, ylabel;
};

}  // namespace vsg3d
}  // namespace chrono

#endif  // CHRONO_CH_XYCHART_INFO_H