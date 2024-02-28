
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu M. Batagoda
// =============================================================================
//
// =============================================================================

#ifdef USE_SENSOR_NVDB

#include <cuda.h>
#include <cuda_runtime.h>

//#include <nanovdb/NanoVDB.h>
//#include <openvdb/openvdb.h>

//#include <nanovdb/util/GridHandle.h>

//#include <openvdb/tools/LevelSetUtil.h>
//#include <openvdb/tools/ParticlesToLevelSet.h>
//#include <openvdb/tools/LevelSetSphere.h> // replace with your own dependencies for generating the OpenVDB grid
//#include <nanovdb/util/CreateNanoGrid.h> // converter from OpenVDB to NanoVDB (includes NanoVDB.h and GridManager.h)
//#include <nanovdb/util/cuda/CudaDeviceBuffer.h>

#include <vector>


#include "cuda_utils.cuh"

namespace chrono {
namespace sensor {



class MyParticleList {
  protected:
    struct MyParticle {
        openvdb::Vec3R p, v;
        openvdb::Real r;
    };
    openvdb::Real mRadiusScale;
    openvdb::Real mVelocityScale;
    std::vector<MyParticle> mParticleList;

  public:
    typedef openvdb::Vec3R PosType;

    MyParticleList(openvdb::Real rScale = 1, openvdb::Real vScale = 1) : mRadiusScale(rScale), mVelocityScale(vScale) {}
    void add(const openvdb::Vec3R& p, const openvdb::Real& r, const openvdb::Vec3R& v = openvdb::Vec3R(0, 0, 0)) {
        MyParticle pa;
        pa.p = p;
        pa.r = r;
        pa.v = v;
        mParticleList.push_back(pa);
    }
    /// @return coordinate bbox in the space of the specified transfrom
    openvdb::CoordBBox getBBox(const openvdb::GridBase& grid) {
        openvdb::CoordBBox bbox;
        openvdb::Coord &min = bbox.min(), &max = bbox.max();
        openvdb::Vec3R pos;
        openvdb::Real rad, invDx = 1 / grid.voxelSize()[0];
        for (size_t n = 0, e = this->size(); n < e; ++n) {
            this->getPosRad(n, pos, rad);
            const openvdb::Vec3d xyz = grid.worldToIndex(pos);
            const openvdb::Real r = rad * invDx;
            for (int i = 0; i < 3; ++i) {
                min[i] = openvdb::math::Min(min[i], openvdb::math::Floor(xyz[i] - r));
                max[i] = openvdb::math::Max(max[i], openvdb::math::Ceil(xyz[i] + r));
            }
        }
        return bbox;
    }
    // typedef int AttributeType;
    // The methods below are only required for the unit-tests
    openvdb::Vec3R pos(int n) const { return mParticleList[n].p; }
    openvdb::Vec3R vel(int n) const { return mVelocityScale * mParticleList[n].v; }
    openvdb::Real radius(int n) const { return mRadiusScale * mParticleList[n].r; }

    //////////////////////////////////////////////////////////////////////////////
    /// The methods below are the only ones required by tools::ParticleToLevelSet
    /// @note We return by value since the radius and velocities are modified
    /// by the scaling factors! Also these methods are all assumed to
    /// be thread-safe.

    /// Return the total number of particles in list.
    ///  Always required!
    size_t size() const { return mParticleList.size(); }

    /// Get the world space position of n'th particle.
    /// Required by ParticledToLevelSet::rasterizeSphere(*this,radius).
    void getPos(size_t n, openvdb::Vec3R& pos) const { pos = mParticleList[n].p; }

    void getPosRad(size_t n, openvdb::Vec3R& pos, openvdb::Real& rad) const {
        pos = mParticleList[n].p;
        rad = mRadiusScale * mParticleList[n].r;
    }
    void getPosRadVel(size_t n, openvdb::Vec3R& pos, openvdb::Real& rad, openvdb::Vec3R& vel) const {
        pos = mParticleList[n].p;
        rad = mRadiusScale * mParticleList[n].r;
        vel = mVelocityScale * mParticleList[n].v;
    }
    // The method below is only required for attribute transfer
    void getAtt(size_t n, openvdb::Index32& att) const { att = openvdb::Index32(n); }
};


__global__ void convertFloatBufferToNVDBVec3F(float* input, nanovdb::Vec3f* output, int n) {
   int idx = blockIdx.x * blockDim.x + threadIdx.x;
   if (idx < n) {
       output[idx] = nanovdb::Vec3f(input[6*idx], input[6*idx + 1], input[6*idx + 2]);
   }
}

void cuda_convert_float_buffer_to_NVDBVec3f(void* input, void* output, int n) {
   int blockSize = 256;
   int gridSize = (n + blockSize - 1) / blockSize;
   convertFloatBufferToNVDBVec3F<<<gridSize, blockSize>>>((float*)input, (nanovdb::Vec3f*)output, n);
   cudaError_t err = cudaGetLastError();
   if (err != cudaSuccess) {
       fprintf(stderr, "CUDA kernel launch error: %s\n", cudaGetErrorString(err));
       // Handle the error...
   }
   err = cudaDeviceSynchronize();
   if (err != cudaSuccess) {
       fprintf(stderr, "CUDA kernel execution error: %s\n", cudaGetErrorString(err));
       // Handle the error...
   }
}

nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> createNanoVDBGridHandle(void* h_points_buffer, int n) {


    // Loop over the particles and add them to MyParticleList
    MyParticleList pa;
    for (int i = 0; i < n; i++) {
        pa.add(openvdb::Vec3R(((float*)h_points_buffer)[6 * i], ((float*)h_points_buffer)[6 * i + 1], ((float*)h_points_buffer)[6 * i + 2]), 0.01, openvdb::Vec3R(((float*)h_points_buffer)[6 * i + 3], ((float*)h_points_buffer)[6 * i] + 4, ((float*)h_points_buffer)[6 * i] + 5));
    }

    //auto sdf = openvdb::createLevelSet<openvdb::FloatGrid>();
    //openvdb::v11_0::tools::particlesToSdf<openvdb::FloatGrid, MyParticleList>(pa, *sdf);

    //auto srcGrid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(100.0f, openvdb::Vec3f(0.0f), 1.0f);
    //auto handle = nanovdb::createNanoGrid<openvdb::FloatGrid, float, nanovdb::CudaDeviceBuffer>(*srcGrid);

    float* d_points_float_buffer = nullptr;
    cudaMalloc((void**)&d_points_float_buffer, sizeof(float) * 6 * n);
    cudaMemcpy(d_points_float_buffer, h_points_buffer, sizeof(float) * 6 * n, cudaMemcpyHostToDevice);


    //// Loop over the particles and add them to 
    using BuildT = nanovdb::Point;//uint32_t;
    using Vec3T = nanovdb::Vec3f;



    nanovdb::Vec3f* d_points = nullptr;
    cudaMalloc((void**)&d_points, sizeof(nanovdb::Vec3f) * n);
 
    cuda_convert_float_buffer_to_NVDBVec3f(d_points_float_buffer, d_points, n);


    const double voxelSize = 0.01;
    printf("Creating NanoVDB Grid with %d points!\n", n);
    nanovdb::CudaPointsToGrid<BuildT> converter(voxelSize);  // unit map
    converter.setPointType(nanovdb::PointType::World32);
    converter.setVerbose();
    auto handle = converter.getHandle(d_points, n);
   
   

    //nanovdb::NanoGrid<BuildT>* grid = handle.deviceGrid<BuildT>();
    //handle.deviceDownload();
    //nanovdb::NanoGrid<BuildT>* grid_h = handle.grid<BuildT>();
    //auto* tree = grid_h->treePtr();
    ////const uint32_t maxPointsPerVoxel = converter.maxPointsPerVoxel();
    ////const uint32_t maxPointsPerLeaf = converter.maxPointsPerLeaf();
    //// save NanoVBD grid

  
    //printf("############### VDB GRID INFORMATION ################\n");
    //printf("Grid Size: %d\n", grid_h->gridSize());
    //printf("Grid Class: %s\n", nanovdb::toStr(handle.gridMetaData()->gridClass()));
    //printf("Grid Type: %s\n", nanovdb::toStr(handle.gridType(0)));
    ////printf("Point Count: %d\n", (int)grid_h->pointCount());
    //printf("Upper Internal Nodes: %d\n", grid_h->tree().nodeCount(2));
    //printf("Lower Internal Nodes: %d\n", grid_h->tree().nodeCount(1));
    //printf("Leaf Nodes: %d\n", grid_h->tree().nodeCount(0));
    //printf("Active Voxels: %d\n", grid_h->activeVoxelCount());
    //printf("Size of nanovdb::Point: %d", sizeof(BuildT));
  
    ////printf("maxPointsPerVoxel: %d\n maxPointsPerLeaf: %d\n", maxPointsPerVoxel, maxPointsPerLeaf);

    //float wBBoxDimZ = (float)grid_h->worldBBox().dim()[2] * 2;
    //nanovdb::Vec3<float> wBBoxCenter =
    //    nanovdb::Vec3<float>(grid_h->worldBBox().min() + grid_h->worldBBox().dim() * 0.5f);
    //nanovdb::CoordBBox treeIndexBbox = grid_h->tree().bbox();

    //std::cout << "WorldBbox Center: (" << wBBoxCenter[0] << "," << wBBoxCenter[1] << "," << wBBoxCenter[2] << ")" << std::endl;
    //std::cout << "WorldBBox Dimensions: [" << grid_h->worldBBox().dim()[0] << "," << grid_h->worldBBox().dim()[1] << "," << grid_h->worldBBox().dim()[2] << "]" << std::endl;
    //std::cout << "WolrdBBox Bounds: "
    //          << "[" << grid_h->worldBBox().min()[0] << "," << grid_h->worldBBox().min()[1] << "," << grid_h->worldBBox().min()[2]
    //           << "] -> [" << grid_h->worldBBox().max()[0] << "," << grid_h->worldBBox().max()[1] << "," << grid_h->worldBBox().max()[2]
    //           << "]" << std::endl;

    //std::cout << "Bounds: "
    //          << "[" << treeIndexBbox.min()[0] << "," << treeIndexBbox.min()[1] << "," << treeIndexBbox.min()[2]
    //          << "] -> [" << treeIndexBbox.max()[0] << "," << treeIndexBbox.max()[1] << "," << treeIndexBbox.max()[2]
    //          << "]" << std::endl;

    ////try {
    ////    nanovdb::io::writeGrid("realSlope.nvdb", handle);  // Write the NanoVDB grid to file and throw if writing fails
    ////} catch (const std::exception& e) {
    ////    std::cerr << "An exception occurred: \"" << e.what() << "\"" << std::endl;
    ////}
    //printf("############### END #############\n");




   return handle;
}

}  // namespace sensor
}  // namespace chrono
#endif

