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
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/radarprocess.cuh"
#include "chrono_sensor/utils/Dbscan.h"
#include <random>

namespace chrono {
namespace sensor {

ChFilterRadarProcess::ChFilterRadarProcess(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterRadarProcess::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                    std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    // The sensor must be a radar
    if (auto pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
        m_cuda_stream = pRadar->GetCudaStream();
        m_hFOV = pRadar->GetHFOV();
        m_vFOV = pRadar->GetVFOV();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }
    m_radar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor);
    m_buffer_out = chrono_types::make_shared<SensorDeviceRadarXYZBuffer>();
    std::shared_ptr<RadarXYZReturn[]> b(
        cudaHostMallocHelper<RadarXYZReturn>(m_buffer_in->Width * m_buffer_in->Height),
        cudaHostFreeHelper<RadarXYZReturn>);
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    bufferInOut = m_buffer_out;
}


CH_SENSOR_API void ChFilterRadarProcess::Apply() {
    // converts azimuth and elevation to XYZ Coordinates in device
    cuda_radar_pointcloud_from_angles(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), 
                                      (int)m_buffer_in->Width, (int)m_buffer_in->Height, m_hFOV, m_vFOV,
                                      m_cuda_stream);

    // Transfer pointcloud to host
    auto buf = std::vector<RadarXYZReturn>(m_buffer_out->Width * m_buffer_out->Height);
    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(),
                    m_buffer_out->Width * m_buffer_out->Height * sizeof(RadarXYZReturn), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);


    // sort returns to bins by objectId
    auto bins = std::vector<std::vector<RadarXYZReturn>>();
    for (RadarXYZReturn point : buf){
        // remove rays with no returns
        if (point.amplitude > 0){
            // tries to add the return to the bin, if bin doesnt exist, add the bin
            while (bins.size() <= point.objectId){
                bins.push_back(std::vector<RadarXYZReturn>());
            }
            bins[point.objectId].push_back(point);
        }
    }

#if PROFILE
    
    printf("number of bins: %i\n", bins.size());
    for (int i = 0; i < bins.size(); i++){
        printf("bin %i has %i points\n", i, bins[i].size());
    }

#endif

    // Down sample each bin to 30 points if necessary
    for (std::vector<RadarXYZReturn>& bin : bins){
        auto rng = std::default_random_engine{};
        if (bin.size() > 10000){
            std::shuffle(std::begin(bin), std::end(bin), rng);
            bin = std::vector<RadarXYZReturn>(bin.begin(), bin.begin() + (int)(bin.size() * 0.1));
        }
    }

    // buffer to store clustered radar data
    m_buffer_out->Beam_return_count = 0;
    auto processed_buffer = std::vector<RadarXYZReturn>(m_buffer_out->Width * m_buffer_out->Height);

    // cluster each bin 
    for (std::vector<RadarXYZReturn> bin : bins){
        for (RadarXYZReturn point : bin){
            buf[m_buffer_out->Beam_return_count] = point;
            m_buffer_out->Beam_return_count+=1;
        }
    }

    std::vector<vec3f> points;
    for (int i = 0; i < m_buffer_out->Beam_return_count; i++) {
        processed_buffer[i] = buf[i];
        points.push_back(vec3f{processed_buffer[i].x,
                               processed_buffer[i].y,
                               processed_buffer[i].z});
    }

    int minimum_points = 1;
    float epsilon = 1;

#if PROFILE
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "DBSCAN initiated with " << points.size() << " points" << std::endl;

    auto dbscan = DBSCAN();
    dbscan.Run(&points, epsilon, minimum_points);

    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    std::cout << "DBScan time = " << milli << "ms" << std::endl;
#else
    auto dbscan = DBSCAN();
    dbscan.Run(&points, epsilon, minimum_points);
#endif

    // Grab the clustered points from DBSCAN
    auto clusters = dbscan.getClusters();

    // vectors are populated with last scans values, clear them out
    m_buffer_out->avg_velocity.clear();
    m_buffer_out->centroids.clear();
    m_buffer_out->amplitudes.clear();

    // initialize values for each cluster
    for (int i = 0; i < clusters.size(); i++) {
        std::array<float,3> temp = {0,0,0};
        m_buffer_out->avg_velocity.push_back(temp);
        m_buffer_out->centroids.push_back(temp);
        m_buffer_out->amplitudes.push_back(0);
    }

    // summing positions, velocities, intensities to caculate average
    std::vector<RadarXYZReturn> valid_returns;
    for (int i = 0; i < clusters.size(); i++) {
        for (int j = 0; j < clusters[i].size(); j++) {
            // we are adding 1 so cluster ID starts at 1 instead of 0
            int idx = clusters[i][j];
            processed_buffer[idx].objectId = i + 1;
            valid_returns.push_back(processed_buffer[idx]);
            // adding velocity and xyz and then dividing by size in next for loop
            m_buffer_out->centroids[i][0] += processed_buffer[idx].x;
            m_buffer_out->centroids[i][1] += processed_buffer[idx].y;
            m_buffer_out->centroids[i][2] += processed_buffer[idx].z;

            m_buffer_out->avg_velocity[i][0] += processed_buffer[idx].vel_x;
            m_buffer_out->avg_velocity[i][1] += processed_buffer[idx].vel_y;
            m_buffer_out->avg_velocity[i][2] += processed_buffer[idx].vel_z;

            m_buffer_out->amplitudes[i] += processed_buffer[idx].amplitude;
        }
    }

    // averaging positions and velocities. NOTE: We are not averaging intensity
    for (int i = 0; i < m_buffer_out->avg_velocity.size(); i++) {
        m_buffer_out->avg_velocity[i][0] = m_buffer_out->avg_velocity[i][0] / (clusters[i].size());
        m_buffer_out->avg_velocity[i][1] = m_buffer_out->avg_velocity[i][1] / (clusters[i].size());
        m_buffer_out->avg_velocity[i][2] = m_buffer_out->avg_velocity[i][2] / (clusters[i].size());
        m_buffer_out->centroids[i][0] = m_buffer_out->centroids[i][0] / (clusters[i].size());
        m_buffer_out->centroids[i][1] = m_buffer_out->centroids[i][1] / (clusters[i].size());
        m_buffer_out->centroids[i][2] = m_buffer_out->centroids[i][2] / (clusters[i].size());
    }

    m_buffer_out->invalid_returns = m_buffer_out->Beam_return_count - valid_returns.size();
    m_buffer_out->Beam_return_count = valid_returns.size();
    m_buffer_out->Num_clusters = clusters.size();

//    for (int i = 0; i < clusters.size(); i++){
//        valid_returns.data()[i].xyz[0] = m_buffer_out->centroids[i][0];
//        valid_returns.data()[i].xyz[1] = m_buffer_out->centroids[i][1];
//        valid_returns.data()[i].xyz[2] = m_buffer_out->centroids[i][2];
//        valid_returns.data()[i].vel[0] = m_buffer_out->avg_velocity[i][0];
//        valid_returns.data()[i].vel[1] = m_buffer_out->avg_velocity[i][1];
//        valid_returns.data()[i].vel[2] = m_buffer_out->avg_velocity[i][2];
//        valid_returns.data()[i].intensity = m_buffer_out->intensity[i];
//    }
//    m_buffer_out->Beam_return_count = clusters.size();
//    printf("number of clusters: %f\n", clusters.size());
    memcpy(m_buffer_out->Buffer.get(), valid_returns.data(),
           m_buffer_out->Beam_return_count * sizeof(RadarXYZReturn));

#if PROFILE
    printf("Scan %i\n", m_scan_number);
    m_scan_number++;
    int total_returns = m_buffer_out->Beam_return_count + m_buffer_out->invalid_returns;
    printf("Total Number of returns: %i |  Number of clustered returns: %i | Number of clusters: %i\n", total_returns,
           m_buffer_out->Beam_return_count, clusters.size());

    // note that we are starting with i = 1 because there are no clusters with id of 0
    for (int i = 1; i <= clusters.size(); i++) {
        int count = 0;
        float3 avg_vel = {0, 0, 0};
        for (int j = 0; j < m_buffer_out->Beam_return_count; j++) {
            if (m_buffer_out->Buffer[j].objectId == i) {
                count++;
            }
        }
        printf("Cluster %i: %i returns\n", i, count);
        printf("velocity %f %f %f\n", m_buffer_out->avg_velocity[i - 1][0], m_buffer_out->avg_velocity[i - 1][1],
               m_buffer_out->avg_velocity[i - 1][2]);
        printf("centroid %f %f %f\n", m_buffer_out->centroids[i - 1][0], m_buffer_out->centroids[i - 1][1],
               m_buffer_out->centroids[i - 1][2]);
        printf("intensity %f\n", m_buffer_out->intensity[i - 1]);
        printf("-------\n");
    }
    printf("--------------------------------------------------------\n");
#endif
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}

}  // namespace sensor
}  // namespace chrono