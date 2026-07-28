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
// Authors: Han Wang, Asher Elmquist
// =============================================================================
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterRadarProcess.h"
#include "chrono_sensor/utils/Dbscan.h"

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/cuda/radarprocess.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>
#if PROFILE
#include <chrono>
#include <iostream>
#endif

namespace chrono {
namespace sensor {

namespace {

RadarXYZReturn RadarToXYZ(const RadarReturn& in) {
    const float proj_xy = in.range * std::cos(in.elevation);
    RadarXYZReturn out{};
    out.x = proj_xy * std::cos(in.azimuth);
    out.y = proj_xy * std::sin(in.azimuth);
    out.z = in.range * std::sin(in.elevation);
    out.vel_x = in.doppler_velocity[0];
    out.vel_y = in.doppler_velocity[1];
    out.vel_z = in.doppler_velocity[2];
    out.amplitude = in.amplitude;
    out.objectId = in.objectId;
    return out;
}

std::vector<RadarXYZReturn> ClusterRadarReturns(std::vector<RadarXYZReturn>& buf,
                                                SensorDeviceRadarXYZBuffer& out_buffer,
                                                int& scan_number) {
    auto bins = std::vector<std::vector<RadarXYZReturn>>();
    for (RadarXYZReturn point : buf) {
        if (point.amplitude > 0) {
            const size_t object_id = static_cast<size_t>(std::max(0.f, point.objectId));
            while (bins.size() <= object_id)
                bins.push_back(std::vector<RadarXYZReturn>());
            bins[object_id].push_back(point);
        }
    }

    for (std::vector<RadarXYZReturn>& bin : bins) {
        auto rng = std::default_random_engine{};
        if (bin.size() > 10000) {
            std::shuffle(std::begin(bin), std::end(bin), rng);
            bin = std::vector<RadarXYZReturn>(bin.begin(), bin.begin() + static_cast<int>(bin.size() * 0.1));
        }
    }

    out_buffer.Beam_return_count = 0;
    auto processed_buffer = std::vector<RadarXYZReturn>(out_buffer.Width * out_buffer.Height);
    for (std::vector<RadarXYZReturn> bin : bins) {
        for (RadarXYZReturn point : bin) {
            buf[out_buffer.Beam_return_count] = point;
            out_buffer.Beam_return_count += 1;
        }
    }

    std::vector<vec3f> points;
    for (int i = 0; i < out_buffer.Beam_return_count; i++) {
        processed_buffer[i] = buf[i];
        points.push_back(vec3f{processed_buffer[i].x, processed_buffer[i].y, processed_buffer[i].z});
    }

    int minimum_points = 1;
    float epsilon = 1.f;

#if PROFILE
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "DBSCAN initiated with " << points.size() << " points" << std::endl;
#endif
    auto dbscan = DBSCAN();
    dbscan.Run(&points, epsilon, minimum_points);
#if PROFILE
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    std::cout << "DBScan time = " << milli << "ms" << std::endl;
#endif

    auto clusters = dbscan.getClusters();

    out_buffer.avg_velocity.clear();
    out_buffer.centroids.clear();
    out_buffer.amplitudes.clear();

    for (size_t i = 0; i < clusters.size(); i++) {
        std::array<float, 3> temp = {0, 0, 0};
        out_buffer.avg_velocity.push_back(temp);
        out_buffer.centroids.push_back(temp);
        out_buffer.amplitudes.push_back(0);
    }

    std::vector<RadarXYZReturn> valid_returns;
    for (size_t i = 0; i < clusters.size(); i++) {
        for (size_t j = 0; j < clusters[i].size(); j++) {
            int idx = clusters[i][j];
            processed_buffer[idx].objectId = static_cast<float>(i + 1);
            valid_returns.push_back(processed_buffer[idx]);
            out_buffer.centroids[i][0] += processed_buffer[idx].x;
            out_buffer.centroids[i][1] += processed_buffer[idx].y;
            out_buffer.centroids[i][2] += processed_buffer[idx].z;
            out_buffer.avg_velocity[i][0] += processed_buffer[idx].vel_x;
            out_buffer.avg_velocity[i][1] += processed_buffer[idx].vel_y;
            out_buffer.avg_velocity[i][2] += processed_buffer[idx].vel_z;
            out_buffer.amplitudes[i] += processed_buffer[idx].amplitude;
        }
    }

    for (size_t i = 0; i < out_buffer.avg_velocity.size(); i++) {
        const float inv_cluster_size = clusters[i].empty() ? 0.f : 1.f / static_cast<float>(clusters[i].size());
        out_buffer.avg_velocity[i][0] *= inv_cluster_size;
        out_buffer.avg_velocity[i][1] *= inv_cluster_size;
        out_buffer.avg_velocity[i][2] *= inv_cluster_size;
        out_buffer.centroids[i][0] *= inv_cluster_size;
        out_buffer.centroids[i][1] *= inv_cluster_size;
        out_buffer.centroids[i][2] *= inv_cluster_size;
    }

    int num_valid_returns = static_cast<int>(valid_returns.size());
    out_buffer.invalid_returns = out_buffer.Beam_return_count - num_valid_returns;
    out_buffer.Beam_return_count = num_valid_returns;
    out_buffer.Num_clusters = static_cast<int>(clusters.size());

#if PROFILE
    printf("Scan %i\n", scan_number++);
#endif
    return valid_returns;
}

}  // namespace

ChFilterRadarProcess::ChFilterRadarProcess(std::string name) : ChFilter(name) {}

void ChFilterRadarProcess::Initialize(std::shared_ptr<ChSensor> pSensor,
                                      std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pRadar->GetCudaStream();
#endif
        m_hFOV = static_cast<float>(pRadar->GetHFOV());
        m_vFOV = static_cast<float>(pRadar->GetVFOV());
        m_radar = pRadar;
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_out = chrono_types::make_shared<SensorDeviceRadarXYZBuffer>();
#ifdef CHRONO_HAS_OPTIX
    std::shared_ptr<RadarXYZReturn[]> b(cudaHostMallocHelper<RadarXYZReturn>(m_buffer_in->Width * m_buffer_in->Height),
                                        cudaHostFreeHelper<RadarXYZReturn>);
#else
    std::shared_ptr<RadarXYZReturn[]> b(new RadarXYZReturn[static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height]);
#endif
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    bufferInOut = m_buffer_out;
}

void ChFilterRadarProcess::Apply() {
#ifdef CHRONO_HAS_OPTIX
    cuda_radar_pointcloud_from_angles(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                      (int)m_buffer_in->Height, m_hFOV, m_vFOV, m_cuda_stream);

    auto buf = std::vector<RadarXYZReturn>(m_buffer_out->Width * m_buffer_out->Height);
    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(),
                    m_buffer_out->Width * m_buffer_out->Height * sizeof(RadarXYZReturn), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
#else
    auto buf = std::vector<RadarXYZReturn>(m_buffer_out->Width * m_buffer_out->Height);
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    for (size_t i = 0; i < count; ++i)
        buf[i] = RadarToXYZ(m_buffer_in->Buffer[i]);
#endif

    auto valid_returns = ClusterRadarReturns(buf, *m_buffer_out, m_scan_number);
    if (!valid_returns.empty())
        std::memcpy(m_buffer_out->Buffer.get(), valid_returns.data(),
                    valid_returns.size() * sizeof(RadarXYZReturn));

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}

}  // namespace sensor
}  // namespace chrono