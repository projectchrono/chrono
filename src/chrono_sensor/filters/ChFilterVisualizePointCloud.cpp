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

CH_SENSOR_API ChFilterVisualizePointCloud::ChFilterVisualizePointCloud(int w, int h, float zoom, std::string name)
    : m_zoom(zoom), ChFilterVisualize(w, h, name) {}

CH_SENSOR_API ChFilterVisualizePointCloud::~ChFilterVisualizePointCloud() {}

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
        float FOV = 20 * m_zoom;
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

        // Call only once for all remaining points
        glPointSize(1.0);
        glBegin(GL_POINTS);

        if (pDeviceXYZIBuffer) {
            int data_w = pDeviceXYZIBuffer->Width;
            int data_h = pDeviceXYZIBuffer->Height;
            unsigned int sz = data_w * data_h * sizeof(PixelXYZI);
            auto tmp_buf = std::make_unique<PixelXYZI[]>(sz);

            cudaMemcpy(tmp_buf.get(), pDeviceXYZIBuffer->Buffer.get(), sz, cudaMemcpyDeviceToHost);

            // draw the vertices, color them by the intensity of the lidar point (red=0, green=1)
            for (int i = 0; i < data_w; i++) {
                for (int j = 0; j < data_h; j++) {
                    if (tmp_buf[i * data_h + j].intensity > 1e-6) {
                        float inten = tmp_buf[i * data_h + j].intensity;
                        glColor3f(1 - inten, inten, 0);
                        glVertex3f(-tmp_buf[i * data_h + j].y, tmp_buf[i * data_h + j].z, tmp_buf[i * data_h + j].x);
                    }
                }
            }
        }

        // Done drawing points
        glEnd();

        // Restore transformations
        glPopMatrix();

        // Flush drawing commands
        glFlush();

        glfwSwapBuffers(m_window.get());
        glfwPollEvents();
    }
}

}  // namespace sensor
}  // namespace chrono
