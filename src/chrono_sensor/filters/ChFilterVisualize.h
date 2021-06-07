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

// #include "glad.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "chrono_sensor/filters/ChFilter.h"

#include <iostream>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

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
    ChFilterVisualize(int w, int h, std::string name = {});

    /// Class destructor
    virtual ~ChFilterVisualize();

    /// Apply function. Visualizes data as an image.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

  protected:
    /// Creates a GLFW window for this filter
    void CreateGlfwWindow(std::shared_ptr<ChSensor> pSensor);

    /// Makes this filter's GLFW window active so it can be drawn to
    void MakeGlContextActive();

    /// Helper to allow GLFWwindow to be in a unique_ptr
    struct DestroyglfwWin {
        void operator()(GLFWwindow* ptr) { glfwDestroyWindow(ptr); }
    };

    std::unique_ptr<GLFWwindow, DestroyglfwWin> m_window;  ///< pointer to the window
    unsigned int m_gl_tex_id = 0;                          ///< reference data for the GL context and texture

    /// Helper function for when new window is created
    static void OnNewWindow();

    /// Helper function for when window is closed.
    static void OnCloseWindow();
    static int s_windowCount;        ///< keeps track of the window count
    bool m_window_disabled = false;  ///< for checking if window is not allowed on sysmtem (e.g. headless rendering)
    int m_w;                         ///< width of the window
    int m_h;                         ///< height of the window
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
