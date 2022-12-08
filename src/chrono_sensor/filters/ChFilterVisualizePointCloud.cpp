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
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterVisualizePointCloud::ChFilterVisualizePointCloud(int w, int h, float zoom, std::string name)
    : m_zoom(zoom), ChFilterVisualize(w, h, name) {}

CH_SENSOR_API ChFilterVisualizePointCloud::~ChFilterVisualizePointCloud() {}

CH_SENSOR_API void ChFilterVisualizePointCloud::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                           std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    auto pOptixSen = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pOptixSen) {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }
    m_cuda_stream = pOptixSen->GetCudaStream();
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    m_host_buffer = chrono_types::make_shared<SensorHostXYZIBuffer>();
    std::shared_ptr<PixelXYZI[]> b(
        cudaHostMallocHelper<PixelXYZI>(m_buffer_in->Width * m_buffer_in->Height * (m_buffer_in->Dual_return + 1)),
        cudaHostFreeHelper<PixelXYZI>);
    m_host_buffer->Buffer = std::move(b);
    m_host_buffer->Width = m_buffer_in->Width;
    m_host_buffer->Height = m_buffer_in->Height;

#ifndef USE_SENSOR_GLFW
    std::cerr << "WARNING: Chrono::SENSOR not built with GLFW support. Will proceed with no window.\n";
#endif
}
CH_SENSOR_API void ChFilterVisualizePointCloud::Apply() {
#ifdef USE_SENSOR_GLFW
    if (!m_window && !m_window_disabled) {
        CreateGlfwWindow(Name());
    }
    // only render if we have a window
    if (m_window) {
        // copy buffer to host
        cudaMemcpyAsync(m_host_buffer->Buffer.get(), m_buffer_in->Buffer.get(),
                        m_buffer_in->Beam_return_count * sizeof(PixelXYZI), cudaMemcpyDeviceToHost);
        // lock the glfw mutex because from here on out, we don't want to be interrupted
        std::lock_guard<std::mutex> lck(s_glfwMutex);
        // visualize data

        glfwMakeContextCurrent(m_window.get());

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
        glOrtho(-FOV, FOV, -FOV, FOV, -FOV,
                FOV);  // TODO: adjust these based on the sensor or data - vis parameters

        // Reset Model view matrix stack
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Clear the window with current clearing color
        glClear(GL_COLOR_BUFFER_BIT);

        // Save matrix state and do the rotation
        glPushMatrix();

        glRotatef(-30, 1.0f, 0.0f, 0.0f);
        glRotatef(-45, 0.0f, 1.0f, 0.0f);

        // Call only once for all remaining points
        glPointSize(1.0);
        glBegin(GL_POINTS);

        // display the points, synchronizing the stream first
        cudaStreamSynchronize(m_cuda_stream);
        // draw the vertices, color them by the intensity of the lidar point (red=0, green=1)
        for (unsigned int i = 0; i < m_buffer_in->Beam_return_count; i++) {
            float inten = m_host_buffer->Buffer[i].intensity;
            glColor3f(1 - inten, inten, 0);
            glVertex3f(-m_host_buffer->Buffer[i].y, m_host_buffer->Buffer[i].z, m_host_buffer->Buffer[i].x);
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
#endif
}

}  // namespace sensor
}  // namespace chrono
