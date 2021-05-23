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

#define PROFILE true

#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterRadarVisualizeCluster::ChFilterRadarVisualizeCluster(int w, int h, float zoom, std::string name)
    : m_zoom(zoom), ChFilterVisualize(w, h, name) {}

CH_SENSOR_API ChFilterRadarVisualizeCluster::~ChFilterRadarVisualizeCluster() {}

CH_SENSOR_API void ChFilterRadarVisualizeCluster::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                        std::shared_ptr<SensorBuffer>& bufferInOut){
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    auto pOptixSen =  std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pOptixSen)
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    m_cuda_stream = pOptixSen->GetCudaStream();
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceProcessedRadarBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
}

CH_SENSOR_API void ChFilterRadarVisualizeCluster::Apply() {
    if (!m_window && !m_window_disabled) {
        CreateGlfwWindow(Name());
        if (m_window)
            glfwSetWindowSize(m_window.get(), 640,
                              480);  // because window size is not just dependent on sensor data size here
    }
    // only render if we have a window
    if (m_window) {
        //copy buffer to host

        // lock the glfw mutex because from here on out, we do not want to be interrupted
        std::lock_guard<std::mutex> lck(s_glfwMutex);
        //visualize data
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
        glOrtho(-FOV, FOV, -FOV, FOV, -FOV, FOV);  // TODO: adjust these based on the sensor or data - vis parameters

        // Reset Model view matrix stack
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Clear the window with current clearing color
        glClear(GL_COLOR_BUFFER_BIT);

        // Save matrix state and do the rotation
        glPushMatrix();

        glRotatef(-30, 1.0f, 0.0f, 0.0f);

        // Call only once for all remaining points
        glPointSize(1.0);
        glBegin(GL_POINTS);

        // display the points, synchronoizing the streamf first
        cudaStreamSynchronize(m_cuda_stream);
        // draw the vertices, color them by clusterID 

        if (m_buffer_in->Num_clusters > 0){
            for (int i = 0; i < m_buffer_in->Beam_return_count; i++){
                float inten = 1 / (float)m_buffer_in->Num_clusters;
                glColor3f(1 - inten * m_buffer_in->Buffer[i].objectID, inten * m_buffer_in->Buffer[i].objectID, inten * 0.5 * m_buffer_in->Buffer[i].objectID);
//                glColor3f(1, 1, 1);
                glVertex3f(-m_buffer_in->Buffer[i].y, m_buffer_in->Buffer[i].z, m_buffer_in->Buffer[i].x);
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
