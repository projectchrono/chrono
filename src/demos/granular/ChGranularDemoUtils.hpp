#pragma once
#include "chrono/utils/ChUtilsSamplers.h"

template <typename T>
std::vector<ChVector<T>> PDLayerSampler_BOX(ChVector<T> center, ChVector<T> hdims, T diam, T padding_factor = 1.02) {
    T fill_bottom = center.z() - hdims.z();
    T fill_top = center.z() + hdims.z();

    // set center to bottom
    center.z() = fill_bottom;
    // 2D layer
    hdims.z() = 0;

    chrono::utils::PDSampler<T> sampler(diam * padding_factor);
    std::vector<ChVector<T>> points_full;
    while (center.z() < fill_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        points_full.insert(points_full.end(), points.begin(), points.end());
        center.z() += diam * padding_factor;
    }
    return points_full;
}
