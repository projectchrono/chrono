#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"

#ifdef USE_SENSOR_NVDB

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/Ray.h>
#include <nanovdb/util/HDDA.h>

#include <iostream>

template <typename RayT, typename AccT>
inline __hostdev__ bool ZeroCrossingPoint(RayT& ray, AccT& acc, nanovdb::Coord& ijk, typename AccT::ValueType& v, float& t) {
    if (!ray.clip(acc.root().bbox()) || ray.t1() > 1e20)
        return false;  // clip ray to bbox
    static const float Delta = 1.0001f;
    //printf("OK1\n");
    ijk = nanovdb::RoundDown<nanovdb::Coord>(ray.start());  // first hit of bbox
    nanovdb::HDDA<RayT, nanovdb::Coord> hdda(ray, acc.getDim(ijk, ray));
    const auto v0 = acc.getValue(ijk);
    
    while (hdda.step()) {
        ijk = nanovdb::RoundDown<nanovdb::Coord>(ray(hdda.time() + Delta));
        hdda.update(ray, acc.getDim(ijk, ray));
        bool b1 = (hdda.dim() > 1);
        bool b2 = (!acc.isActive(ijk));
        //printf("v0: %d | hadd.dim() > 1? %d | !acc.isActive(ijk)? %d\n", v0, b1, b2);
        if (hdda.dim() > 1 || !acc.isActive(ijk))
            continue;                                        // either a tile value or an inactive voxel
       
        while (hdda.step() && acc.isActive(hdda.voxel())) {  // in the narrow band
            v = acc.getValue(hdda.voxel());
            //printf("v: %d, v0: %d\n", v, v0);
            if (v * v0 < 0) {  // zero crossing
                ijk = hdda.voxel();
                t = hdda.time();
                return true;
            }
        }
    }
    return false;
}

extern "C" __global__ void __intersection__nvdb_vol_intersect() {
    const float3 ray_orig = optixGetObjectRayOrigin();
    const float3 ray_dir = optixGetObjectRayDirection();
    const float ray_tmin = optixGetRayTmin();
    const float ray_tmax = optixGetRayTmax();
    
    // Print ray info
    //printf("ray_orig: %f %f %f\n", ray_orig.x, ray_orig.y, ray_orig.z);
    //printf("ray_dir: %f %f %f\n", ray_dir.x, ray_dir.y, ray_dir.z);

    /// NanoVDB stuff

    using Vec3T = nanovdb::Vec3f;
    using BuildT = nanovdb::Point;

    // convert to nanovdb types
    const Vec3T ray_orig_v(ray_orig.x, ray_orig.y, ray_orig.z);
    const Vec3T ray_dir_v(ray_dir.x, ray_dir.y, ray_dir.z);

    //nanovdb::NanoGrid<nanovdb::Point>* grid = params.handle_ptr;
    nanovdb::NanoGrid<BuildT>* grid = params.handle_ptr;
   
    // Get grid accessor
    
    nanovdb::DefaultReadAccessor<BuildT> acc = grid->tree().getAccessor();
    //nanovdb::PointAccessor<Vec3T, BuildT> pacc(*grid);

    //nanovdb::DefaultReadAccessor<BuildT>::ValueType v1;

    
    //printf("OK1\n");
    // Get ray origin in grid space
    const Vec3T eye = grid->worldToIndex(ray_orig_v);
    const Vec3T dir = grid->worldToIndex(ray_dir_v);

   // printf("OK12\n");
    // check if ray intersects grid
    nanovdb::Ray<float> iRay(eye, dir, ray_tmin, ray_tmax);
    float t0;
    nanovdb::Coord ijk;
    nanovdb::DefaultReadAccessor<BuildT>::ValueType v;

    // Implement ZeroCrossing for PointType
    
    if (ZeroCrossingPoint(iRay, acc, ijk, v, t0)) {
        float wT0 = t0 * float(grid->voxelSize()[0]);
        printf("ray intersects grid\n");
        printf("ijk: %d %d %d\n", ijk[0], ijk[1], ijk[2]);
        printf("v: %f\n", v);
        printf("wT0: %d\n", wT0);
    } else {
        //printf("ray does not intersect grid\n");
    }
}

#endif
