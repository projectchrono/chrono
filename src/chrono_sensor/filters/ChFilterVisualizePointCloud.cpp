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

#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/ChOptixSensor.h"

namespace chrono {
namespace sensor {

int ChFilterVisualizePointCloud::s_windowCount = 0;

CH_SENSOR_API ChFilterVisualizePointCloud::ChFilterVisualizePointCloud(int w, int h, std::string name)
    : m_w(w), m_h(h), ChFilter(name) {
    ChFilterVisualizePointCloud::OnNewWindow();
}

CH_SENSOR_API ChFilterVisualizePointCloud::~ChFilterVisualizePointCloud() {
    ChFilterVisualizePointCloud::OnCloseWindow();
}

CH_SENSOR_API void ChFilterVisualizePointCloud::Apply(std::shared_ptr<ChSensor> pSensor,
                                                      std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to visualize, the buffer must either be an optix buffer, or a GPU R8 (grayscale) buffer.
    std::shared_ptr<SensorHostXYZIBuffer> pHostXYZIBuffer =
        std::dynamic_pointer_cast<SensorHostXYZIBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceXYZIBuffer> pDeviceXYZIBuffer =
        std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);

    std::shared_ptr<ChOptixSensor> pVis = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);

    if (!pHostXYZIBuffer && !pDeviceXYZIBuffer)
        throw std::runtime_error("This buffer type cannot be visualized.");

    if (!pVis)
        throw std::runtime_error("This sensor type cannot be visualized.");

    if (!m_window && !m_window_disabled) {
        CreateGlfwWindow(pSensor);
        if (m_window)
            glfwSetWindowSize(m_window.get(), 640,
                              480);  // because window size is not just dependent on sensor data size here
    }
    // only render if we have a window
    if (m_window) {
        MakeGlContextActive();

        int window_w, window_h;
        glfwGetWindowSize(m_window.get(), &window_w, &window_h);
        //

        // Set Viewport to window dimensions
        glViewport(0, 0, window_w, window_h);

        // Reset projection matrix stack
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        // Establish clipping volume (left, right, bottom, top, near, far)
        float FOV = 20;
        glOrtho(-FOV, FOV, -FOV, FOV, -FOV, FOV);  // TODO: adjust these based on the sensor or data - vis parameters

        // Reset Model view matrix stack
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        GLfloat x, y, z, angle;  // Storage for coordinates and angles
        // Clear the window with current clearing color
        glClear(GL_COLOR_BUFFER_BIT);

        // Save matrix state and do the rotation
        glPushMatrix();

        glRotatef(-30, 1.0f, 0.0f, 0.0f);
        glRotatef(-45, 0.0f, 1.0f, 0.0f);

        // glColor3f(1.0f, 1.0f, 1.0f);
        // Call only once for all remaining points
        glPointSize(1.0);
        glBegin(GL_POINTS);

        if (pDeviceXYZIBuffer) {
            int data_w = pDeviceXYZIBuffer->Width;
            int data_h = pDeviceXYZIBuffer->Height;
            unsigned int sz = data_w * data_h * sizeof(PixelXYZI);
            auto tmp_buf = std::make_unique<PixelXYZI[]>(sz);

            cudaMemcpy(tmp_buf.get(), pDeviceXYZIBuffer->Buffer.get(), sz, cudaMemcpyDeviceToHost);

            // draw the vertices
            for (int i = 0; i < data_w; i++) {
                for (int j = 0; j < data_h; j++) {
                    if (tmp_buf[i * data_h + j].intensity > 1e-3) {
                        glColor3f(1 - tmp_buf[i * data_h + j].intensity, tmp_buf[i * data_h + j].intensity,
                                  tmp_buf[i * data_h + j].z / 10.0);
                        glVertex3f(-tmp_buf[i * data_h + j].y, tmp_buf[i * data_h + j].z, tmp_buf[i * data_h + j].x);
                    }
                }
            }
        }
        // else if (pHostXYZIBuffer) {
        //     int data_w = pHostXYZIBuffer->Width;
        //     int data_h = pHostXYZIBuffer->Height;
        //     for (int i = 0; i < data_w; i++) {
        //         for (int j = 0; j < data_h; j++) {
        //             // glColor3f(1.0, 1.0, pHostXYZIBuffer->Buffer[i * data_h + j].z / 10.0);
        //             // glVertex3f(-pHostXYZIBuffer->Buffer[i * data_h + j].y, pHostXYZIBuffer->Buffer[i * data_h +
        //             j].z,
        //             //            pHostXYZIBuffer->Buffer[i * data_h + j].x);
        //
        //             if (pHostXYZIBuffer->Buffer[i * data_h + j].intensity > 1e-6) {
        //                 glColor3f(pHostXYZIBuffer->Buffer[i * data_h + j].intensity,
        //                           pHostXYZIBuffer->Buffer[i * data_h + j].intensity,
        //                           pHostXYZIBuffer->Buffer[i * data_h + j].z / 10.0);
        //                 glVertex3f(-pHostXYZIBuffer->Buffer[i * data_h + j].y,
        //                            pHostXYZIBuffer->Buffer[i * data_h + j].z,
        //                            pHostXYZIBuffer->Buffer[i * data_h + j].x);
        //             }
        //         }
        //     }
        // }

        // Done drawing points
        glEnd();

        // Restore transformations
        glPopMatrix();

        // Flush drawing commands
        glFlush();

        glfwSwapBuffers(m_window.get());
        glfwPollEvents();
        // }
    }
}

void ChFilterVisualizePointCloud::CreateGlfwWindow(std::shared_ptr<ChSensor> pSensor) {
    // if we've already made the window, there's nothing to do.
    if (m_window)
        return;

    // to visualize, the sensor must inherit from ChOptixSensor
    std::shared_ptr<ChOptixSensor> pVis = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pVis)
        throw std::runtime_error("Cannot create a window for a sensor that does not implement ChOptixSensor");

    OnNewWindow();
    std::stringstream s;
    s << pSensor->GetName().c_str();
    if (Name().length() > 0)
        s << " - " << Name();
    m_window.reset(glfwCreateWindow(static_cast<GLsizei>(m_w), static_cast<GLsizei>(m_h), s.str().c_str(), NULL, NULL));
    glfwSwapInterval(0);
    if (!m_window) {
        // OnCloseWindow();
        //     throw std::runtime_error("Could not create window");
    }
    if (m_window) {
        glfwMakeContextCurrent(m_window.get());

        // TODO: will only succeed the first time, so perhaps do this more intelligently
        glewInit();

        // Init state
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, 1, 0, 1, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glViewport(0, 0, m_w, m_h);
    } else {
        std::cerr << "WARNING: requested window could not be created by GLFW. Will proceed with no window.\n";
        m_window_disabled = true;
    }
}

void ChFilterVisualizePointCloud::MakeGlContextActive() {
    if (!m_window)
        throw std::runtime_error("Cannot make gl context active because there is no window");
    glfwMakeContextCurrent(m_window.get());
}

void ChFilterVisualizePointCloud::OnNewWindow() {
    if (s_windowCount++ == 0) {
        glfwInit();
    }
}
void ChFilterVisualizePointCloud::OnCloseWindow() {
    if (--s_windowCount == 0)
        glfwTerminate();
}

}  // namespace sensor
}  // namespace chrono
