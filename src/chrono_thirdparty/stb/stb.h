// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Simple C++ wrapper around STB functions
//
// =============================================================================

#ifndef STB_WRAPPER_H
#define STB_WRAPPER_H

#include <cassert>
#include <limits>
#include <string>

#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/stb/stb_image_write.h"

///  Simple C++ wrapper around image functions in the STB library.
class STB {
  public:
    STB() : m_nx(0), m_ny(0), m_num_channels(0), m_avail_channels(0), m_range(0), m_data(nullptr) {}
    ~STB() { delete m_data; }

    /// Read image from specified file, loading the requested number of channels.
    /// Passing req_comp=0 (default) will result in using the number of channels defined in the input file.
    /// Note that we always read the image as 16-bit (8-bit images are automatically converted).
    bool ReadFromFile(const std::string& filename, int req_comp = 0) {
        assert(req_comp >= 0 && req_comp <= 4);
        m_data = stbi_load_16(filename.c_str(), &m_nx, &m_ny, &m_avail_channels, req_comp);
        m_num_channels = (req_comp == 0) ? m_avail_channels : req_comp;
        m_range = USHRT_MAX;
        return m_data != nullptr;
    }

    /// Return the image width (in pixels).
    int GetWidth() const {return m_nx;}
    /// Return the image height (in pixels).
    int GetHeight() const {return m_ny;}
    /// Return the number of channels loaded.
    int GetNumChannels() const {return m_num_channels;}
    /// Return the number of channels available in the input file.
    int GetNumAvailableChannels() const {return m_avail_channels;}
    /// Return the range of the image data (for 16-bit, this is USHRT_MAX=65535).
    int GetRange() const {return m_range;}

    /// Return the red value at the specified pixel. If no red channel, return -1.
    /// Recall that images are loaded from the top-left corner.
    unsigned short Red(int i, int j) const {
        if (m_num_channels < 3)
            return -1;
        return *(m_data + m_num_channels * (m_nx * j + i) + 0);
    }

    /// Return the green value at the specified pixel. If no green channel, return -1.
    /// Recall that images are loaded from the top-left corner.
    unsigned short Green(int i, int j) const {
        if (m_num_channels < 3)
            return -1;
        return *(m_data + m_num_channels * (m_nx * j + i) + 1);
    }

    /// Return the blue value at the specified pixel. If no blue channel, return -1.
    /// Recall that images are loaded from the top-left corner.
    unsigned short Blue(int i, int j) const {
        if (m_num_channels < 3)
            return -1;
        return *(m_data + m_num_channels * (m_nx * j + i) + 2);
    }

    /// Return the alpha value at the specified pixel. If no alpha channel, return -1.
    /// Recall that images are loaded from the top-left corner.
    unsigned short Alpha(int i, int j) const {
        if (m_num_channels == 1 || m_num_channels == 3)
            return -1;
        return *(m_data + m_num_channels * (m_nx * j + i) + (m_num_channels - 1));
    }

    /// Return the gray value at the specified pixel.
    /// If the data has an explicit gray channel (num. channels = 1 or 2), return that value.
    /// Otherwise, convert RGB to YUV.
    /// Recall that images are loaded from the top-left corner.
    unsigned short Gray(int i, int j) const {
        if (m_num_channels < 3)
            return *(m_data + m_num_channels * (m_nx * j + i) + 0);
        auto r = *(m_data + m_num_channels * (m_nx * j + i) + 0);
        auto g = *(m_data + m_num_channels * (m_nx * j + i) + 1);
        auto b = *(m_data + m_num_channels * (m_nx * j + i) + 2);
        return static_cast<unsigned short>(0.299 * r + 0.587 * g + 0.114 * b);  // RGB -> YUV conversion
    }

  private:
    int m_nx;                ///< image width
    int m_ny;                ///< image height
    int m_avail_channels;    ///< number of available channels
    int m_num_channels;      ///< number of channels
    int m_range;             ///< data range
    unsigned short* m_data;  ///< image data
};

#endif