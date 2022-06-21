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

#define PROFILE false

#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterRadarVisualizeCluster::ChFilterRadarVisualizeCluster(int w, int h, float zoom, std::string name)
    : m_zoom(zoom), ChFilterVisualize(w, h, name) {}

CH_SENSOR_API ChFilterRadarVisualizeCluster::~ChFilterRadarVisualizeCluster() {}

CH_SENSOR_API void ChFilterRadarVisualizeCluster::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                             std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    auto pOptixSen = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pOptixSen)
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    m_cuda_stream = pOptixSen->GetCudaStream();
    m_radar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor);
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarXYZBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
#ifndef USE_SENSOR_GLFW
    std::cerr << "WARNING: Chrono::SENSOR not built with GLFW support. Will proceed with no window.\n";
#endif
}

CH_SENSOR_API void ChFilterRadarVisualizeCluster::Apply() {
#ifdef USE_SENSOR_GLFW
    if (!m_window && !m_window_disabled) {
        CreateGlfwWindow(Name());
        float hfov = m_radar->GetHFOV();
        float vfov = m_radar->GetVFOV();
        if (m_window) {
            glfwSetWindowSize(m_window.get(), 960, 960 / hfov * vfov);
        }
    }
    // only render if we have a window
    if (m_window) {
        // copy buffer to host

        // lock the glfw mutex because from here on out, we do not want to be interrupted
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

        float hfov = m_radar->GetHFOV();
        float vfov = m_radar->GetVFOV();
        float near = m_radar->GetClipNear();
        float far = m_radar->GetMaxDistance();
        float right = tan(hfov / 2) * near;
        float left = -right;
        float top = tan(vfov / 2) * near;
        float bottom = -top;

        glFrustum(left, right, bottom, top, near, far);

        // Reset Model view matrix stack
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Clear the window with current clearing color
        glClear(GL_COLOR_BUFFER_BIT);

        // Save matrix state and do the rotation
        glPushMatrix();

        // Call only once for all remaining points
        glPointSize(1.0);
        glBegin(GL_POINTS);

        // display the points, synchronoizing the streamf first
        cudaStreamSynchronize(m_cuda_stream);
        // draw the vertices, color them by clusterID

        if (m_buffer_in->Num_clusters > 0) {
            for (int i = 0; i < m_buffer_in->Beam_return_count; i++) {
                float inten = 1 / (float)m_buffer_in->Num_clusters;
                glColor3f(1 - inten * m_buffer_in->Buffer[i].objectId, inten * m_buffer_in->Buffer[i].objectId,
                          inten * 0.5 * m_buffer_in->Buffer[i].objectId);
                //                glColor3f(1, 1, 1);
                glVertex3f(-m_buffer_in->Buffer[i].y, m_buffer_in->Buffer[i].z, -m_buffer_in->Buffer[i].x);
//                printf("%f %f %f\n",m_buffer_in->Buffer[i].xyz[0],m_buffer_in->Buffer[i].xyz[1],m_buffer_in->Buffer[i].xyz[2]);
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
#endif
}

}  // namespace sensor
}  // namespace chrono
