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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHFILTERVISUALIZE_H
#define CHFILTERVISUALIZE_H

#ifdef USE_SENSOR_GLFW
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif

#include "chrono_sensor/filters/ChFilter.h"

#include <iostream>
#include <mutex>

#include <cuda.h>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;
class ChOptixSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, creates a GUI window to visualize the sensor (using GLFW). This visualizes
/// data as an image. Will only work on data that can be interpreted as image data.
class CH_SENSOR_API ChFilterVisualize : public ChFilter {
  public:
    /// Class constructor
    /// @param w Width of the window to create
    /// @param h Height of the window to create
    /// @param name String name of the filter
    /// @param fullscreen whether the window should be spawned in full screen mode
    ChFilterVisualize(int w, int h, std::string name = "ChFilterVisualize", bool fullscreen = false);

    /// Class destructor
    virtual ~ChFilterVisualize();

    /// Apply function. Visualizes data as an image.

    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  protected:
    /// Creates a GLFW window for this filter
    void CreateGlfwWindow(std::string m_name);

#ifdef USE_SENSOR_GLFW
    /// Helper to allow GLFWwindow to be in a unique_ptr
    struct DestroyglfwWin {
        void operator()(GLFWwindow* ptr) { glfwDestroyWindow(ptr); }
    };
#endif

    std::shared_ptr<SensorDeviceR8Buffer> m_bufferR8;
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_bufferRGBA8;
    std::shared_ptr<SensorDeviceSemanticBuffer> m_bufferSemantic;
    std::shared_ptr<SensorDeviceDIBuffer> m_bufferDI;
    std::shared_ptr<SensorDeviceRadarBuffer> m_bufferRadar;

    std::shared_ptr<SensorHostR8Buffer> m_hostR8;
    std::shared_ptr<SensorHostRGBA8Buffer> m_hostRGBA8;
    std::shared_ptr<SensorHostSemanticBuffer> m_hostSemantic;
    std::shared_ptr<SensorHostDIBuffer> m_hostDI;
    std::shared_ptr<SensorHostRadarBuffer> m_hostRadar;

    CUstream m_cuda_stream;  ///< reference to the cuda stream

#ifdef USE_SENSOR_GLFW
    std::unique_ptr<GLFWwindow, DestroyglfwWin> m_window;  ///< pointer to the window
#endif
    unsigned int m_gl_tex_id = 0;                          ///< reference data for the GL context and texture

    /// Helper function for when new window is created
    static void OnNewWindow();

    /// Helper function for when window is closed.
    static void OnCloseWindow();
    static int s_windowCount;       ///< keeps track of the window count
    static std::mutex s_glfwMutex;  ///< mutex to prevent us making two windows at the exact same time

    bool m_window_disabled = false;  ///< for checking if window is not allowed on sysmtem (e.g. headless rendering)
    int m_w;                         ///< width of the window
    int m_h;                         ///< height of the window
    bool m_fullscreen;               ///< toggle for fullscreen mode
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
