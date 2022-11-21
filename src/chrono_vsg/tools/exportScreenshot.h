#ifndef CH_EXPORT_SCREENSHOT_H
#define CH_EXPORT_SCREENSHOT_H

#include <vsg/all.h>

#include <vsgXchange/all.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include "chrono_vsg/core/ChApiVSG.h"

namespace chrono {
namespace vsg3d {
    CH_VSG_API void exportScreenshot(vsg::ref_ptr<vsg::Window> window, vsg::ref_ptr<vsg::Options> options, std::string& imageFilename);
}
}  // namespace chrono
#endif