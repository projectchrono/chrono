// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHTRTUTILS_H
#define CHTRTUTILS_H

#include <NvUtils.h>
#include <iostream>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_tensorrt
/// @{

/// Logger verbosity enumerator
enum LoggerVerbosity {
    NONE,     ///< print nothing when loading and running
    PARTIAL,  ///< print everything except info
    ALL       ///< print everything -> most verbose
};

/// Inference Logger for TensorRT
class Logger : public nvinfer1::ILogger {
  public:
    Logger(LoggerVerbosity verbose_level = NONE) : m_verbose_level(verbose_level), ILogger() {}
    ~Logger() {}
    /// Logging function. Will be called by the TensorRT parser.
    virtual void log(ILogger::Severity severity, const char* msg) {
        switch (m_verbose_level) {
            case PARTIAL: {
                if (severity != Severity::kINFO)
                    std::cout << msg << std::endl;
            }
            case ALL: {
                std::cout << msg << std::endl;
                break;
            }
            default: {
                break;
            }
        }
    }

  private:
    LoggerVerbosity m_verbose_level;  ///< for storing verbosity level
};

/// destructor for tensorRT pointers
struct TRTDestroyer {
    template <typename T>
    void operator()(T* ptr) {
        if (ptr)
            ptr->destroy();
    }
};
/// @} sensor_tensorrt

}  // namespace sensor
}  // namespace chrono

#endif
