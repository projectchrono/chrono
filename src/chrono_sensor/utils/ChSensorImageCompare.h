// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut
// =============================================================================
//
// Header-only numeric image-comparison helpers for Chrono::Sensor.
//
// These provide a scalar, per-pixel comparison between two rendered images so a
// candidate renderer (for example the experimental Vulkan RT backend) can be
// checked against a reference renderer (for example the OptiX backend) beyond a
// visual side-by-side. Metrics are reported in 8-bit code units [0,255].
//
// The utility is intentionally dependency-light (only ChSensorBuffer types) and
// header-only, so any sensor demo or test can include it without extra build
// wiring. Convention throughout: "candidate" is the image under test and
// "reference" is the ground truth; the signed bias is (candidate - reference).
//
// =============================================================================

#ifndef CH_SENSOR_IMAGE_COMPARE_H
#define CH_SENSOR_IMAGE_COMPARE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"

// These helpers live directly in chrono::sensor, matching every other header in this directory
// (ChSensorUtils.h, ChGPSUtils.h, ChSensorUtilsJSON.h, CudaMallocHelper.h). An earlier version
// nested them in a chrono::sensor::utils namespace, which made an unqualified `utils::` ambiguous
// against chrono::utils in any translation unit that pulls in both chrono/utils/ChBodyGeometry.h
// and this header. That is not hypothetical: ChSystemNSC.h now reaches ChBodyGeometry.h
// transitively through ChOutput.h and ChUtilsYAML.h, so a demo saying `using namespace chrono;`
// plus `using namespace chrono::sensor;` failed to compile. Following the directory's existing
// convention avoids the collision entirely.
namespace chrono {
namespace sensor {

#if defined(CHRONO_HAS_OPTIX) || defined(CHRONO_HAS_VULKAN_RT)

/// Scalar comparison metrics between two RGBA8 images.
/// All error quantities are expressed in 8-bit code units on [0,255].
/// Array indices 0..3 correspond to the R, G, B, A channels.
struct ImageCompareResult {
    bool valid = false;             ///< false if buffers are null or sizes disagree
    unsigned int width = 0;         ///< image width (pixels)
    unsigned int height = 0;        ///< image height (pixels)
    unsigned int num_pixels = 0;    ///< width * height
    unsigned int threshold = 0;     ///< per-channel abs-diff threshold used for frac_over_threshold

    double rmse[4] = {0, 0, 0, 0};     ///< per-channel root-mean-square error
    double mae[4] = {0, 0, 0, 0};      ///< per-channel mean absolute error
    double max_abs[4] = {0, 0, 0, 0};  ///< per-channel maximum absolute error
    double bias[4] = {0, 0, 0, 0};     ///< per-channel signed mean error (candidate - reference)

    double rmse_rgb = 0.0;     ///< RMSE pooled over the R,G,B channels
    double mae_rgb = 0.0;      ///< MAE pooled over the R,G,B channels
    double max_abs_rgb = 0.0;  ///< maximum absolute error over the R,G,B channels
    double bias_rgb = 0.0;     ///< signed mean error pooled over the R,G,B channels
    double psnr_rgb = 0.0;     ///< peak signal-to-noise ratio over R,G,B (dB; +inf if identical)

    double frac_over_threshold = 0.0;  ///< fraction of pixels whose max(|dR|,|dG|,|dB|) exceeds threshold
};

/// Compare two RGBA8 buffers pixel-by-pixel and return scalar error metrics.
/// @param candidate image under test (for example the Vulkan RT render)
/// @param reference ground-truth image (for example the OptiX render)
/// @param threshold per-channel absolute difference (code units) above which a pixel is counted as differing
inline ImageCompareResult CompareRGBA8(const UserRGBA8BufferPtr& candidate,
                                       const UserRGBA8BufferPtr& reference,
                                       unsigned int threshold = 2) {
    ImageCompareResult r;
    r.threshold = threshold;

    if (!candidate || !reference || !candidate->Buffer || !reference->Buffer)
        return r;
    if (candidate->Width != reference->Width || candidate->Height != reference->Height)
        return r;
    if (candidate->Width == 0 || candidate->Height == 0)
        return r;

    r.width = candidate->Width;
    r.height = candidate->Height;
    r.num_pixels = r.width * r.height;

    double sse[4] = {0, 0, 0, 0};    // sum of squared error
    double sad[4] = {0, 0, 0, 0};    // sum of absolute difference
    double sbias[4] = {0, 0, 0, 0};  // sum of signed difference
    double maxabs[4] = {0, 0, 0, 0};
    unsigned long long over = 0;

    const PixelRGBA8* pa = candidate->Buffer.get();
    const PixelRGBA8* pb = reference->Buffer.get();

    for (unsigned int i = 0; i < r.num_pixels; ++i) {
        const int d[4] = {int(pa[i].R) - int(pb[i].R), int(pa[i].G) - int(pb[i].G),
                          int(pa[i].B) - int(pb[i].B), int(pa[i].A) - int(pb[i].A)};
        int maxrgb = 0;
        for (int c = 0; c < 4; ++c) {
            const double ad = std::abs((double)d[c]);
            sse[c] += (double)d[c] * d[c];
            sad[c] += ad;
            sbias[c] += (double)d[c];
            if (ad > maxabs[c])
                maxabs[c] = ad;
            if (c < 3 && std::abs(d[c]) > maxrgb)
                maxrgb = std::abs(d[c]);
        }
        if ((unsigned int)maxrgb > threshold)
            ++over;
    }

    const double n = (double)r.num_pixels;
    double sse_rgb = 0, sad_rgb = 0, sbias_rgb = 0;
    for (int c = 0; c < 4; ++c) {
        r.rmse[c] = std::sqrt(sse[c] / n);
        r.mae[c] = sad[c] / n;
        r.bias[c] = sbias[c] / n;
        r.max_abs[c] = maxabs[c];
        if (c < 3) {
            sse_rgb += sse[c];
            sad_rgb += sad[c];
            sbias_rgb += sbias[c];
            r.max_abs_rgb = std::max(r.max_abs_rgb, maxabs[c]);
        }
    }
    const double mse_rgb = sse_rgb / (3.0 * n);
    r.rmse_rgb = std::sqrt(mse_rgb);
    r.mae_rgb = sad_rgb / (3.0 * n);
    r.bias_rgb = sbias_rgb / (3.0 * n);
    r.psnr_rgb = (mse_rgb <= 0.0) ? std::numeric_limits<double>::infinity()
                                  : 10.0 * std::log10(255.0 * 255.0 / mse_rgb);
    r.frac_over_threshold = (double)over / n;
    r.valid = true;
    return r;
}

/// Pretty-print a comparison result to a stream.
inline void PrintImageCompareResult(std::ostream& os,
                                    const ImageCompareResult& r,
                                    const std::string& label = {}) {
    os << "==== Image comparison";
    if (!label.empty())
        os << " [" << label << "]";
    os << " (candidate vs reference) ====\n";
    if (!r.valid) {
        os << "  INVALID: null buffers or size mismatch\n";
        return;
    }
    const std::ios_base::fmtflags saved = os.flags();
    os << "  resolution : " << r.width << " x " << r.height << " (" << r.num_pixels << " px)\n";
    os << std::fixed << std::setprecision(4);
    os << "  RGB        : RMSE=" << r.rmse_rgb << "  MAE=" << r.mae_rgb << "  maxabs=" << r.max_abs_rgb
       << "  bias=" << r.bias_rgb << "  PSNR=" << r.psnr_rgb << " dB\n";
    const char* ch = "RGBA";
    for (int c = 0; c < 4; ++c)
        os << "    " << ch[c] << "       : RMSE=" << r.rmse[c] << "  MAE=" << r.mae[c] << "  maxabs=" << r.max_abs[c]
           << "  bias=" << r.bias[c] << "\n";
    os << std::setprecision(2);
    os << "  pixels with max|dRGB| > " << r.threshold << " : " << (100.0 * r.frac_over_threshold) << " %\n";
    os << "  (8-bit code units [0,255]; bias = candidate - reference)\n";
    os.flags(saved);
}

/// Build an amplified absolute-difference image, |candidate - reference| * gain (clamped to 255),
/// with opaque alpha, for saving/inspection. Returns nullptr on null buffers or a size mismatch.
/// A gain > 1 makes small errors visible; use gain = 1 for a faithful magnitude map.
inline UserRGBA8BufferPtr MakeAbsDiffRGBA8(const UserRGBA8BufferPtr& candidate,
                                           const UserRGBA8BufferPtr& reference,
                                           float gain = 1.0f) {
    if (!candidate || !reference || !candidate->Buffer || !reference->Buffer)
        return nullptr;
    if (candidate->Width != reference->Width || candidate->Height != reference->Height)
        return nullptr;

    const unsigned int w = candidate->Width;
    const unsigned int h = candidate->Height;
    const unsigned int n = w * h;

    auto out = std::make_shared<SensorHostRGBA8Buffer>();
    out->Width = w;
    out->Height = h;
    out->TimeStamp = candidate->TimeStamp;
    out->LaunchedCount = candidate->LaunchedCount;
    out->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[n]);

    const PixelRGBA8* pa = candidate->Buffer.get();
    const PixelRGBA8* pb = reference->Buffer.get();
    auto clamp8 = [](float v) -> uint8_t {
        v = v < 0.f ? 0.f : (v > 255.f ? 255.f : v);
        return (uint8_t)(v + 0.5f);
    };
    for (unsigned int i = 0; i < n; ++i) {
        out->Buffer[i].R = clamp8(gain * std::abs(int(pa[i].R) - int(pb[i].R)));
        out->Buffer[i].G = clamp8(gain * std::abs(int(pa[i].G) - int(pb[i].G)));
        out->Buffer[i].B = clamp8(gain * std::abs(int(pa[i].B) - int(pb[i].B)));
        out->Buffer[i].A = 255;
    }
    return out;
}

#endif  // CHRONO_HAS_OPTIX || CHRONO_HAS_VULKAN_RT

}  // namespace sensor
}  // namespace chrono

#endif  // CH_SENSOR_IMAGE_COMPARE_H
