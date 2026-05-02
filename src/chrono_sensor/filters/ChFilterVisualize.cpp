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

#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

int ChFilterVisualize::s_windowCount = 0;
std::mutex ChFilterVisualize::s_glfwMutex;

CH_SENSOR_API ChFilterVisualize::ChFilterVisualize(int w, int h, std::string name, bool fullscreen)
    : m_w(w), m_h(h), m_fullscreen(fullscreen), ChFilter(name) {}

CH_SENSOR_API ChFilterVisualize::~ChFilterVisualize() {
    ChFilterVisualize::OnCloseWindow();
}

CH_SENSOR_API void ChFilterVisualize::Apply() {
#ifdef USE_SENSOR_GLFW
    if (!m_window && !m_window_disabled) {
        CreateGlfwWindow(Name());
    }
    if (m_window) {
        // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        // do all memcpy
        cudaStreamSynchronize(m_cuda_stream);
        if (m_bufferR8) {
            cudaMemcpyAsync(m_hostR8->Buffer.get(), m_bufferR8->Buffer.get(),
                            m_bufferR8->Width * m_bufferR8->Height * sizeof(char), cudaMemcpyDeviceToHost,
                            m_cuda_stream);
        } else if (m_bufferRGBA8) {
            cudaMemcpyAsync(m_hostRGBA8->Buffer.get(), m_bufferRGBA8->Buffer.get(),
                            m_hostRGBA8->Width * m_hostRGBA8->Height * sizeof(PixelRGBA8), cudaMemcpyDeviceToHost,
                            m_cuda_stream);
        } else if (m_bufferRGBA16) {
            cudaMemcpyAsync(m_hostRGBA16->Buffer.get(), m_bufferRGBA16->Buffer.get(),
                            m_hostRGBA16->Width * m_hostRGBA16->Height * sizeof(PixelRGBA16), cudaMemcpyDeviceToHost,
                            m_cuda_stream);
        } else if (m_bufferRGBDHalf4) {
            cudaMemcpyAsync(m_hostRGBDHalf4->Buffer.get(), m_bufferRGBDHalf4->Buffer.get(),
                            m_hostRGBDHalf4->Width * m_hostRGBDHalf4->Height * sizeof(PixelRGBDHalf4), cudaMemcpyDeviceToHost,
                            m_cuda_stream);
            // throw std::runtime_error("RGBD cannot be directly visualized");
        } else if (m_bufferSemantic) {
            cudaMemcpyAsync(m_hostSemantic->Buffer.get(), m_bufferSemantic->Buffer.get(),
                            m_hostSemantic->Width * m_hostSemantic->Height * sizeof(PixelSemantic),
                            cudaMemcpyDeviceToHost, m_cuda_stream);
        } else if(m_bufferDepth) {
             cudaMemcpyAsync(m_hostDepth->Buffer.get(), m_bufferDepth->Buffer.get(),
                            m_hostDepth->Width * m_hostDepth->Height * sizeof(PixelDepth),
                            cudaMemcpyDeviceToHost, m_cuda_stream);
        } else if (m_bufferNormal) {
             cudaMemcpyAsync(m_hostNormal->Buffer.get(), m_bufferNormal->Buffer.get(),
                            m_hostNormal->Width * m_hostNormal->Height * sizeof(PixelNormal),
                            cudaMemcpyDeviceToHost, m_cuda_stream);
        } else if (m_bufferDI) {
            cudaMemcpyAsync(m_hostDI->Buffer.get(), m_bufferDI->Buffer.get(),
                            m_hostDI->Width * m_hostDI->Height * sizeof(PixelDI), cudaMemcpyDeviceToHost,
                            m_cuda_stream);
        } else if (m_bufferRadar) {
            cudaMemcpyAsync(m_hostRadar->Buffer.get(), m_bufferRadar->Buffer.get(),
                            m_hostRadar->Width * m_hostRadar->Height * sizeof(RadarReturn), cudaMemcpyDeviceToHost,
                            m_cuda_stream);
        } else {
            throw std::runtime_error("No buffer incoming for visualization");
        }

        // lock the glfw mutex because from here on out, we don't want to be interrupted
        std::lock_guard<std::mutex> lck(s_glfwMutex);

        // do window prep stuff
        glfwMakeContextCurrent(m_window.get());
        if (!m_gl_tex_id) {
            glGenTextures(1, &m_gl_tex_id);
            glBindTexture(GL_TEXTURE_2D, m_gl_tex_id);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

            // GL_CLAMP_TO_EDGE for linear filtering, not relevant for nearest.
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
        glBindTexture(GL_TEXTURE_2D, m_gl_tex_id);

        // Set Viewport to window dimensions
        int window_w, window_h;
        glfwGetWindowSize(m_window.get(), &window_w, &window_h);
        glViewport(0, 0, window_w, window_h);

        // update the textures, making sure data has finished memcpy first
        cudaStreamSynchronize(m_cuda_stream);
        if (m_bufferR8) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, m_hostR8->Width, m_hostR8->Height, 0, GL_RED, GL_UNSIGNED_BYTE,
                         m_hostR8->Buffer.get());
        } else if (m_bufferRGBA8) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_hostRGBA8->Width, m_hostRGBA8->Height, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, m_hostRGBA8->Buffer.get());
        } else if (m_bufferRGBA16) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_hostRGBA16->Width, m_hostRGBA16->Height, 0, GL_RGBA,
                         GL_UNSIGNED_SHORT, m_hostRGBA16->Buffer.get()); // ???
        } else if (m_bufferRGBDHalf4) {
            throw std::runtime_error("RGBD cannot be directly visualized");
        } else if (m_bufferSemantic) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16, m_hostSemantic->Width, m_hostSemantic->Height, 0, GL_RG,
                         GL_UNSIGNED_SHORT, m_hostSemantic->Buffer.get());
        } else if(m_bufferDepth) {
             glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, m_hostDepth->Width, m_hostDepth->Height, 0, GL_RED, GL_FLOAT,
                         m_hostDepth->Buffer.get());
        } else if (m_bufferNormal) {
             glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, m_hostNormal->Width, m_hostNormal->Height, 0, GL_RGBA, GL_FLOAT,
                          m_hostNormal->Buffer.get());
        } else if (m_bufferDI) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG32F, m_hostDI->Width, m_hostDI->Height, 0, GL_RG, GL_FLOAT,
                         m_hostDI->Buffer.get());
        } else if (m_bufferRadar) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG32F, m_hostRadar->Width, m_hostRadar->Height, 0, GL_RG, GL_FLOAT,
                         m_hostRadar->Buffer.get());
        } else {
            throw std::runtime_error("No buffer incoming for visualization");
        }

        // update the window

        // 1:1 texel to pixel mapping with glOrtho(0, 1, 0, 1, -1, 1) setup:
        // The quad coordinates go from lower left corner of the lower left pixel
        // to the upper right corner of the upper right pixel.
        // Same for the texel coordinates.

        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex2f(0.0f, 0.0f);
        glTexCoord2f(1.0f, 0.0f);
        glVertex2f(1.0f, 0.0f);
        glTexCoord2f(1.0f, 1.0f);
        glVertex2f(1.0f, 1.0f);
        glTexCoord2f(0.0f, 1.0f);
        glVertex2f(0.0f, 1.0f);
        glEnd();
        glDisable(GL_TEXTURE_2D);

        glfwSwapBuffers(m_window.get());
        glfwPollEvents();

        // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "Render to window time: " << wall_time.count() << std::endl;
    }
#endif
}
CH_SENSOR_API void ChFilterVisualize::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                 std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    auto pOptixSen = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pOptixSen) {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }
    m_cuda_stream = pOptixSen->GetCudaStream();
    m_bufferR8 = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    m_bufferRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_bufferRGBA16 = std::dynamic_pointer_cast<SensorDeviceRGBA16Buffer>(bufferInOut);
    m_bufferRGBDHalf4 = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut);
    m_bufferSemantic = std::dynamic_pointer_cast<SensorDeviceSemanticBuffer>(bufferInOut);
    m_bufferDI = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut);
    m_bufferRadar = std::dynamic_pointer_cast<SensorDeviceRadarBuffer>(bufferInOut);
    m_bufferDepth = std::dynamic_pointer_cast<SensorDeviceDepthBuffer>(bufferInOut);
    m_bufferNormal = std::dynamic_pointer_cast<SensorDeviceNormalBuffer>(bufferInOut);

    if (m_bufferR8) {
        m_hostR8 = chrono_types::make_shared<SensorHostR8Buffer>();
        std::shared_ptr<char[]> b(cudaHostMallocHelper<char>(m_bufferR8->Width * m_bufferR8->Height),
                                  cudaHostFreeHelper<char>);
        m_hostR8->Buffer = std::move(b);
        m_hostR8->Width = m_bufferR8->Width;
        m_hostR8->Height = m_bufferR8->Height;

    } else if (m_bufferRGBA8) {
        m_hostRGBA8 = chrono_types::make_shared<SensorHostRGBA8Buffer>();
        std::shared_ptr<PixelRGBA8[]> b(cudaHostMallocHelper<PixelRGBA8>(m_bufferRGBA8->Width * m_bufferRGBA8->Height),
                                        cudaHostFreeHelper<PixelRGBA8>);
        m_hostRGBA8->Buffer = std::move(b);
        m_hostRGBA8->Width = m_bufferRGBA8->Width;
        m_hostRGBA8->Height = m_bufferRGBA8->Height;
    } else if (m_bufferRGBA16) {
        m_hostRGBA16 = chrono_types::make_shared<SensorHostRGBA16Buffer>();
        std::shared_ptr<PixelRGBA16[]> b(
            cudaHostMallocHelper<PixelRGBA16>(m_bufferRGBA16->Width * m_bufferRGBA16->Height),
            cudaHostFreeHelper<PixelRGBA16>
        );
        m_hostRGBA16->Buffer = std::move(b);
        m_hostRGBA16->Width = m_bufferRGBA16->Width;
        m_hostRGBA16->Height = m_bufferRGBA16->Height;

    } else if (m_bufferRGBDHalf4) {
        m_hostRGBDHalf4 = chrono_types::make_shared<SensorHostRGBDHalf4Buffer>();
        std::shared_ptr<PixelRGBDHalf4[]> b(
            cudaHostMallocHelper<PixelRGBDHalf4>(m_bufferRGBDHalf4->Width * m_bufferRGBDHalf4->Height),
            cudaHostFreeHelper<PixelRGBDHalf4>
        );
        m_hostRGBDHalf4->Buffer = std::move(b);
        m_hostRGBDHalf4->Width = m_bufferRGBDHalf4->Width;
        m_hostRGBDHalf4->Height = m_bufferRGBDHalf4->Height;
        
    } else if (m_bufferSemantic) {
        m_hostSemantic = chrono_types::make_shared<SensorHostSemanticBuffer>();
        std::shared_ptr<PixelSemantic[]> b(
            cudaHostMallocHelper<PixelSemantic>(m_bufferSemantic->Width * m_bufferSemantic->Height),
            cudaHostFreeHelper<PixelSemantic>);
        m_hostSemantic->Buffer = std::move(b);
        m_hostSemantic->Width = m_bufferSemantic->Width;
        m_hostSemantic->Height = m_bufferSemantic->Height;

    } else if (m_bufferDepth) {
        m_hostDepth = chrono_types::make_shared<SensorHostDepthBuffer>();
         std::shared_ptr<PixelDepth[]> b(
            cudaHostMallocHelper<PixelDepth>(m_bufferDepth->Width * m_bufferDepth->Height),
            cudaHostFreeHelper<PixelDepth>);
        m_hostDepth->Buffer = std::move(b);
        m_hostDepth->Width = m_bufferDepth->Width;
        m_hostDepth->Height = m_bufferDepth->Height;

    } else if (m_bufferNormal) {
        m_hostNormal = chrono_types::make_shared<SensorHostNormalBuffer>();
         std::shared_ptr<PixelNormal[]> b(
            cudaHostMallocHelper<PixelNormal>(m_bufferNormal->Width * m_bufferNormal->Height),
            cudaHostFreeHelper<PixelNormal>);
        m_hostNormal->Buffer = std::move(b);
        m_hostNormal->Width = m_bufferNormal->Width;
        m_hostNormal->Height = m_bufferNormal->Height;

    } else if (m_bufferDI) {
        m_hostDI = chrono_types::make_shared<SensorHostDIBuffer>();
        std::shared_ptr<PixelDI[]> b(cudaHostMallocHelper<PixelDI>(m_bufferDI->Width * m_bufferDI->Height),
                                     cudaHostFreeHelper<PixelDI>);
        m_hostDI->Buffer = std::move(b);
        m_hostDI->Width = m_bufferDI->Width;
        m_hostDI->Height = m_bufferDI->Height;
    } else if (m_bufferRadar) {
        m_hostRadar = chrono_types::make_shared<SensorHostRadarBuffer>();
        std::shared_ptr<RadarReturn[]> b(
            cudaHostMallocHelper<RadarReturn>(m_bufferRadar->Width * m_bufferRadar->Height),
            cudaHostFreeHelper<RadarReturn>);
        m_hostRadar->Buffer = std::move(b);
        m_hostRadar->Width = m_bufferRadar->Width;
        m_hostRadar->Height = m_bufferRadar->Height;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

#ifndef USE_SENSOR_GLFW
    std::cerr << "WARNING: Chrono::SENSOR not built with GLFW support. Will proceed with no window.\n";
#endif
}

CH_SENSOR_API void ChFilterVisualize::CreateGlfwWindow(std::string window_name) {
    // if we've already made the window, there's nothing to do.
#ifdef USE_SENSOR_GLFW
    if (m_window)
        return;

    OnNewWindow();  // OnNewWindow will need to lock inside itself

    std::lock_guard<std::mutex> lck(s_glfwMutex);

    if (m_fullscreen)
        m_window.reset(glfwCreateWindow(static_cast<GLsizei>(m_w), static_cast<GLsizei>(m_h), window_name.c_str(),
                                        glfwGetPrimaryMonitor(), NULL));
    else
        m_window.reset(
            glfwCreateWindow(static_cast<GLsizei>(m_w), static_cast<GLsizei>(m_h), window_name.c_str(), NULL, NULL));
    if (m_window) {
        glfwMakeContextCurrent(m_window.get());
        glfwSwapInterval(0);  // disable vsync as we are "fast as possible"
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
#endif
}

CH_SENSOR_API void ChFilterVisualize::OnNewWindow() {
#ifdef USE_SENSOR_GLFW
    std::lock_guard<std::mutex> l(s_glfwMutex);
    if (s_windowCount++ == 0) {
        glfwInit();
    }
#endif
}
CH_SENSOR_API void ChFilterVisualize::OnCloseWindow() {
#ifdef USE_SENSOR_GLFW
    std::lock_guard<std::mutex> l(s_glfwMutex);
    if (--s_windowCount == 0) {
        glfwTerminate();
    }
#endif
}

}  // namespace sensor
}  // namespace chrono
