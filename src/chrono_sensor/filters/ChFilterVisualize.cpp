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

#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace chrono {
namespace sensor {

namespace {

uint8_t ToByte(float v) {
    v = std::max(0.f, std::min(1.f, v));
    return static_cast<uint8_t>(255.f * v + 0.5f);
}

PixelRGBA8 MakeRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    PixelRGBA8 p;
    p.R = r;
    p.G = g;
    p.B = b;
    p.A = a;
    return p;
}

}  // namespace

int ChFilterVisualize::s_windowCount = 0;
std::mutex ChFilterVisualize::s_glfwMutex;

CH_SENSOR_API ChFilterVisualize::ChFilterVisualize(int w, int h, std::string name, bool fullscreen)
    : ChFilter(name), m_w(w), m_h(h), m_fullscreen(fullscreen) {}

CH_SENSOR_API ChFilterVisualize::~ChFilterVisualize() {
#ifdef USE_SENSOR_GLFW
    if (m_window)
        OnCloseWindow();
#endif
}

void ChFilterVisualize::OnNewWindow() {
#ifdef USE_SENSOR_GLFW
    std::lock_guard<std::mutex> lock(s_glfwMutex);
    if (s_windowCount++ == 0) {
        if (!glfwInit()) {
            --s_windowCount;
            std::cerr << "WARNING: GLFW initialization failed. Chrono::Sensor visualization window disabled.\n";
        }
    }
#endif
}

void ChFilterVisualize::OnCloseWindow() {
#ifdef USE_SENSOR_GLFW
    std::lock_guard<std::mutex> lock(s_glfwMutex);
    if (s_windowCount > 0 && --s_windowCount == 0)
        glfwTerminate();
#endif
}

void ChFilterVisualize::CreateGlfwWindow(std::string window_name) {
#ifdef USE_SENSOR_GLFW
    if (m_window || m_window_disabled)
        return;

    OnNewWindow();

    std::lock_guard<std::mutex> lock(s_glfwMutex);
    if (s_windowCount <= 0) {
        m_window_disabled = true;
        return;
    }

    GLFWmonitor* monitor = m_fullscreen ? glfwGetPrimaryMonitor() : nullptr;
    m_window.reset(glfwCreateWindow(static_cast<GLsizei>(m_w), static_cast<GLsizei>(m_h), window_name.c_str(), monitor,
                                    nullptr));
    if (!m_window) {
        std::cerr << "WARNING: requested GLFW window could not be created. Chrono::Sensor will continue without this "
                     "visualization window.\n";
        m_window_disabled = true;
        if (s_windowCount > 0 && --s_windowCount == 0)
            glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(m_window.get());
    glfwSwapInterval(0);

    GLenum glew_status = glewInit();
    if (glew_status != GLEW_OK) {
        std::cerr << "WARNING: GLEW initialization failed for Chrono::Sensor visualization window: "
                  << reinterpret_cast<const char*>(glewGetErrorString(glew_status)) << "\n";
    }

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glViewport(0, 0, m_w, m_h);

    if (!m_gl_tex_id)
        glGenTextures(1, &m_gl_tex_id);
#else
    (void)window_name;
    if (!m_window_disabled)
        std::cerr << "WARNING: Chrono::Sensor not built with GLFW support. Will proceed with no visualization window.\n";
    m_window_disabled = true;
#endif
}

CH_SENSOR_API void ChFilterVisualize::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                 std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    // In Vulkan-only builds the SensorDevice* aliases are host-visible buffers. Keep the member names identical to
    // the OptiX implementation so the rest of the filter graph still sees the same buffer catalogue.
    m_bufferR8 = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    m_bufferRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_bufferRGBA16 = std::dynamic_pointer_cast<SensorDeviceRGBA16Buffer>(bufferInOut);
    m_bufferRGBDHalf4 = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut);
    m_bufferSemantic = std::dynamic_pointer_cast<SensorDeviceSemanticBuffer>(bufferInOut);
    m_bufferDepth = std::dynamic_pointer_cast<SensorDeviceDepthBuffer>(bufferInOut);
    m_bufferNormal = std::dynamic_pointer_cast<SensorDeviceNormalBuffer>(bufferInOut);
    m_bufferDI = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut);
    m_bufferRadar = std::dynamic_pointer_cast<SensorDeviceRadarBuffer>(bufferInOut);

    if (!m_bufferR8 && !m_bufferRGBA8 && !m_bufferRGBA16 && !m_bufferRGBDHalf4 && !m_bufferSemantic &&
        !m_bufferDepth && !m_bufferNormal && !m_bufferDI && !m_bufferRadar) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
#ifndef USE_SENSOR_GLFW
    if (!m_window_disabled)
        std::cerr << "WARNING: Chrono::Sensor not built with GLFW support. Will proceed with no visualization window.\n";
    m_window_disabled = true;
#endif
}

CH_SENSOR_API void ChFilterVisualize::Apply() {
#ifdef USE_SENSOR_GLFW
    if (!m_window && !m_window_disabled)
        CreateGlfwWindow(Name());

    if (m_window_disabled || !m_window)
        return;

    if (glfwWindowShouldClose(m_window.get()))
        return;

    glfwMakeContextCurrent(m_window.get());

    int window_w, window_h;
    glfwGetWindowSize(m_window.get(), &window_w, &window_h);
    glViewport(0, 0, window_w, window_h);

    glBindTexture(GL_TEXTURE_2D, m_gl_tex_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    std::vector<PixelRGBA8> scratch;
    const void* data = nullptr;
    GLint internal_format = GL_RGBA;
    GLenum format = GL_RGBA;
    GLenum type = GL_UNSIGNED_BYTE;
    GLsizei width = 0;
    GLsizei height = 0;

    if (m_bufferRGBA8 && m_bufferRGBA8->Buffer) {
        data = m_bufferRGBA8->Buffer.get();
        width = static_cast<GLsizei>(m_bufferRGBA8->Width);
        height = static_cast<GLsizei>(m_bufferRGBA8->Height);
    } else if (m_bufferR8 && m_bufferR8->Buffer) {
        const size_t count = static_cast<size_t>(m_bufferR8->Width) * static_cast<size_t>(m_bufferR8->Height);
        scratch.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const uint8_t g = static_cast<uint8_t>(m_bufferR8->Buffer[i]);
            scratch[i] = MakeRGBA(g, g, g);
        }
        data = scratch.data();
        width = static_cast<GLsizei>(m_bufferR8->Width);
        height = static_cast<GLsizei>(m_bufferR8->Height);
    } else if (m_bufferRGBA16 && m_bufferRGBA16->Buffer) {
        data = m_bufferRGBA16->Buffer.get();
        width = static_cast<GLsizei>(m_bufferRGBA16->Width);
        height = static_cast<GLsizei>(m_bufferRGBA16->Height);
        type = GL_UNSIGNED_SHORT;
    } else if (m_bufferRGBDHalf4 && m_bufferRGBDHalf4->Buffer) {
        const size_t count = static_cast<size_t>(m_bufferRGBDHalf4->Width) * static_cast<size_t>(m_bufferRGBDHalf4->Height);
        scratch.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const auto& p = m_bufferRGBDHalf4->Buffer[i];
            scratch[i] = MakeRGBA(ToByte(p.R), ToByte(p.G), ToByte(p.B));
        }
        data = scratch.data();
        width = static_cast<GLsizei>(m_bufferRGBDHalf4->Width);
        height = static_cast<GLsizei>(m_bufferRGBDHalf4->Height);
    } else if (m_bufferSemantic && m_bufferSemantic->Buffer) {
        const size_t count = static_cast<size_t>(m_bufferSemantic->Width) * static_cast<size_t>(m_bufferSemantic->Height);
        scratch.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const auto& s = m_bufferSemantic->Buffer[i];
            // Keep the underlying buffer semantic-compatible, but display it as a deterministic false-color image.
            const uint8_t r = static_cast<uint8_t>((s.class_id * 53u + s.instance_id * 97u) & 0xffu);
            const uint8_t g = static_cast<uint8_t>((s.class_id * 101u + s.instance_id * 17u) & 0xffu);
            const uint8_t b = static_cast<uint8_t>((s.class_id * 29u + s.instance_id * 193u) & 0xffu);
            scratch[i] = MakeRGBA(r, g, b);
        }
        data = scratch.data();
        width = static_cast<GLsizei>(m_bufferSemantic->Width);
        height = static_cast<GLsizei>(m_bufferSemantic->Height);
    } else if (m_bufferDepth && m_bufferDepth->Buffer) {
        const size_t count = static_cast<size_t>(m_bufferDepth->Width) * static_cast<size_t>(m_bufferDepth->Height);
        float min_depth = std::numeric_limits<float>::max();
        float max_depth = 0.f;
        for (size_t i = 0; i < count; ++i) {
            const float d = m_bufferDepth->Buffer[i].depth;
            if (std::isfinite(d)) {
                min_depth = std::min(min_depth, d);
                max_depth = std::max(max_depth, d);
            }
        }
        if (max_depth <= min_depth)
            max_depth = min_depth + 1.f;
        scratch.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const float d = m_bufferDepth->Buffer[i].depth;
            const float t = std::isfinite(d) ? 1.f - (d - min_depth) / (max_depth - min_depth) : 0.f;
            const uint8_t g = ToByte(t);
            scratch[i] = MakeRGBA(g, g, g);
        }
        data = scratch.data();
        width = static_cast<GLsizei>(m_bufferDepth->Width);
        height = static_cast<GLsizei>(m_bufferDepth->Height);
    } else if (m_bufferNormal && m_bufferNormal->Buffer) {
        const size_t count = static_cast<size_t>(m_bufferNormal->Width) * static_cast<size_t>(m_bufferNormal->Height);
        scratch.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const auto& n = m_bufferNormal->Buffer[i];
            scratch[i] = MakeRGBA(ToByte(n.normal_x * 0.5f + 0.5f), ToByte(n.normal_y * 0.5f + 0.5f),
                                  ToByte(n.normal_z * 0.5f + 0.5f));
        }
        data = scratch.data();
        width = static_cast<GLsizei>(m_bufferNormal->Width);
        height = static_cast<GLsizei>(m_bufferNormal->Height);
    } else if (m_bufferDI && m_bufferDI->Buffer) {
        const size_t count = static_cast<size_t>(m_bufferDI->Width) * static_cast<size_t>(m_bufferDI->Height);
        float max_range = 0.f;
        for (size_t i = 0; i < count; ++i)
            max_range = std::max(max_range, m_bufferDI->Buffer[i].range);
        if (max_range <= 0.f)
            max_range = 1.f;
        scratch.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const auto& p = m_bufferDI->Buffer[i];
            scratch[i] = MakeRGBA(ToByte(1.f - p.range / max_range), ToByte(p.intensity), 0);
        }
        data = scratch.data();
        width = static_cast<GLsizei>(m_bufferDI->Width);
        height = static_cast<GLsizei>(m_bufferDI->Height);
    } else if (m_bufferRadar && m_bufferRadar->Buffer) {
        const size_t count = static_cast<size_t>(m_bufferRadar->Width) * static_cast<size_t>(m_bufferRadar->Height);
        float max_range = 0.f;
        float max_amp = 0.f;
        for (size_t i = 0; i < count; ++i) {
            max_range = std::max(max_range, m_bufferRadar->Buffer[i].range);
            max_amp = std::max(max_amp, m_bufferRadar->Buffer[i].amplitude);
        }
        if (max_range <= 0.f)
            max_range = 1.f;
        if (max_amp <= 0.f)
            max_amp = 1.f;
        scratch.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const auto& p = m_bufferRadar->Buffer[i];
            scratch[i] = MakeRGBA(ToByte(1.f - p.range / max_range), ToByte(p.amplitude / max_amp), 0);
        }
        data = scratch.data();
        width = static_cast<GLsizei>(m_bufferRadar->Width);
        height = static_cast<GLsizei>(m_bufferRadar->Height);
    }

    if (!data)
        return;

    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, format, type, data);

    glClear(GL_COLOR_BUFFER_BIT);
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
#endif
}

}  // namespace sensor
}  // namespace chrono

#else

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

#endif
