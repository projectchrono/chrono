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
#include "chrono_sensor/ChOptixSensor.h"

namespace chrono {
namespace sensor {

int ChFilterVisualize::s_windowCount = 0;

CH_SENSOR_API ChFilterVisualize::ChFilterVisualize(int w, int h, std::string name) : m_w(w), m_h(h), ChFilter(name) {
    ChFilterVisualize::OnNewWindow();
}

CH_SENSOR_API ChFilterVisualize::~ChFilterVisualize() {
    ChFilterVisualize::OnCloseWindow();
}

CH_SENSOR_API void ChFilterVisualize::Apply(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to visualize, the buffer must either be an optix buffer, or a GPU R8 (grayscale) buffer.
    std::shared_ptr<SensorOptixBuffer> pOptix = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR8 = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceDIBuffer> pDI = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut);

    if (!pOptix && !pR8 && !pRGBA8 && !pDI)
        throw std::runtime_error("This buffer type cannot be visualized");

    if (!m_window && !m_window_disabled) {
        CreateGlfwWindow(pSensor);
    }
    if (m_window) {
        MakeGlContextActive();

        // make a texture (first time through)
        if (!m_gl_tex_id) {
            glGenTextures(1, &m_gl_tex_id);
            glBindTexture(GL_TEXTURE_2D, m_gl_tex_id);

            // Change these to GL_LINEAR for super- or sub-sampling
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

            // GL_CLAMP_TO_EDGE for linear filtering, not relevant for nearest.
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }

        glBindTexture(GL_TEXTURE_2D, m_gl_tex_id);

        if (pOptix) {
            // Query buffer information
            RTsize buffer_width_rts, buffer_height_rts;
            pOptix->Buffer->getSize(buffer_width_rts, buffer_height_rts);
            uint32_t width = static_cast<int>(buffer_width_rts);
            uint32_t height = static_cast<int>(buffer_height_rts);
            RTformat buffer_format = pOptix->Buffer->getFormat();
            GLvoid* imageData = pOptix->Buffer->map(0, RT_BUFFER_MAP_READ);

            // TODO: not sure what's going on here...should the glPixelStorei line use elmt_size? or what is elmt_size
            // queried for?
            RTsize elmt_size = pOptix->Buffer->getElementSize();
            glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

            if (buffer_format == RT_FORMAT_UNSIGNED_BYTE4) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData);
            } else if (buffer_format == RT_FORMAT_FLOAT2) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RG32F, width, height, 0, GL_RG, GL_FLOAT, imageData);
            } else {
                throw std::runtime_error("Unknown Optix buffer format.");
            }

            pOptix->Buffer->unmap();

        } else if (pR8) {  // grayscal image in GPU memory
            // TODO: there must be a better way than copying the grayscale GPU buffer to host and then setting
            // texture...
            char* buf = new char[pR8->Width * pR8->Height];
            cudaMemcpy(buf, pR8->Buffer.get(), pR8->Width * pR8->Height, cudaMemcpyDeviceToHost);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, pR8->Width, pR8->Height, 0, GL_RED, GL_UNSIGNED_BYTE, buf);
            delete buf;
        } else if (pRGBA8) {  // RGBA8 image in GPU memory
            // TODO: there must be a better way than copying the grayscale GPU buffer to host and then setting
            // texture...
            glViewport(0, 0, pRGBA8->Width, pRGBA8->Height);
            char* buf = new char[pRGBA8->Width * pRGBA8->Height * sizeof(PixelRGBA8)];
            cudaMemcpy(buf, pRGBA8->Buffer.get(), pRGBA8->Width * pRGBA8->Height * sizeof(PixelRGBA8),
                       cudaMemcpyDeviceToHost);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, pRGBA8->Width, pRGBA8->Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
            delete buf;

        }

        else if (pDI) {  // grayscal image in GPU memory
            // TODO: there must be a better way than copying the grayscale GPU buffer to host and then setting
            // texture...
            float* buf = new float[pDI->Width * pDI->Height * 2];
            cudaMemcpy(buf, pDI->Buffer.get(), pDI->Width * pDI->Height * 2 * sizeof(float), cudaMemcpyDeviceToHost);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG32F, pDI->Width, pDI->Height, 0, GL_RG, GL_FLOAT, buf);
            delete buf;
        }

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
    }
}

void ChFilterVisualize::CreateGlfwWindow(std::shared_ptr<ChSensor> pSensor) {
    // if we've already made the window, there's nothing to do.
    if (m_window)
        return;

    // to visualize, the sensor *must* inherit from ICanBeVisualized
    std::shared_ptr<ChOptixSensor> pVis = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pVis)
        throw std::runtime_error("Cannot create a window for a sensor that does not inherit from ChOptixSensor");

    // unsigned int width = pVis->m_width;
    // unsigned int height = pVis->m_height;

    OnNewWindow();
    std::stringstream s;
    s << pSensor->GetName().c_str();
    if (Name().length() > 0)
        s << " - " << Name();
    m_window.reset(glfwCreateWindow(static_cast<GLsizei>(m_w), static_cast<GLsizei>(m_h), s.str().c_str(), NULL, NULL));

    if (!m_window) {
        // OnCloseWindow();
        //     throw std::runtime_error("Could not create window");
    }
    if (m_window) {
        glfwMakeContextCurrent(m_window.get());

        // disable vsync
        glfwSwapInterval(0);

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

void ChFilterVisualize::MakeGlContextActive() {
    if (!m_window)
        throw std::runtime_error("Cannot make gl context active because there is no window");
    glfwMakeContextCurrent(m_window.get());
}

void ChFilterVisualize::OnNewWindow() {
    if (s_windowCount++ == 0) {
        glfwInit();
    }
}
void ChFilterVisualize::OnCloseWindow() {
    if (--s_windowCount == 0)
        glfwTerminate();
}

}  // namespace sensor
}  // namespace chrono
