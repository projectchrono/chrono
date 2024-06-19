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
// Utility classes for various space sampling methods.
//
// ChSampler
//  - base class
//  - implements the top-level methods for sampling a given volume
//  - supported domains: box, rectangle, cylinder, disk, sphere
//
// ChPDSampler
//  - implements Poisson Disk sampler - uniform random distribution with
//    guaranteed minimum distance between any two sample points.
//
// ChGridSampler
//  - uniform grid
//
// ChHCPSampler
// - A class to generate points in a hexagonally close packed structure.
// =============================================================================

#ifndef CH_UTILS_SAMPLERS_H
#define CH_UTILS_SAMPLERS_H

#include <cmath>
#include <list>
#include <random>
#include <utility>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChUtils.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// Construct a single random engine (on first use)
//
// Note that this object is never destructed (but this is OK)
// -----------------------------------------------------------------------------
inline std::default_random_engine& rengine() {
    static std::default_random_engine* re = new std::default_random_engine;
    return *re;
}

// -----------------------------------------------------------------------------
// sampleTruncatedDist
//
// Utility function for generating samples from a truncated normal distribution.
// -----------------------------------------------------------------------------
template <typename T>
inline T sampleTruncatedDist(std::normal_distribution<T>& distribution, T minVal, T maxVal) {
    T val;

    do {
        val = distribution(rengine());
    } while (val < minVal || val > maxVal);

    return val;
}

// -----------------------------------------------------------------------------
// Type definitions
// -----------------------------------------------------------------------------

// Until Visual Studio supports alias templates (VS2013 does) if we want to
// "share" a typedef across different template classes, we use this structure.
template <typename T>
struct Types {
    typedef std::vector<ChVector3<T>> PointVector;
    typedef std::list<ChVector3<T>> PointList;
};

// Convenience shortcuts
typedef Types<double>::PointVector PointVectorD;
typedef Types<float>::PointVector PointVectorF;

/// @addtogroup chrono_utils
/// @{

/// Volumetric sampling method.
enum class SamplingType {
    REGULAR_GRID,  ///< Regular (equidistant) grid
    POISSON_DISK,  ///< Poisson Disk sampling
    HCP_PACK       ///< Hexagonally Close Packing
};

/// Base class for different types of point samplers.
template <typename T>
class ChSampler {
  public:
    static_assert(std::is_floating_point<T>::value,
                  "class chrono::utils::ChSampler can only be instantiated with floating point types");

    typedef typename Types<T>::PointVector PointVector;

    virtual ~ChSampler() {}

    /// Return points sampled from the specified box volume.
    PointVector SampleBox(const ChVector3<T>& center, const ChVector3<T>& halfDim) {
        m_center = center;
        m_size = halfDim;
        SetFuzz();
        return Sample(BOX);
    }

    /// Return points sampled from the specified spherical volume.
    PointVector SampleSphere(const ChVector3<T>& center, T radius) {
        m_center = center;
        m_size = ChVector3<T>(radius, radius, radius);
        SetFuzz();
        return Sample(SPHERE);
    }

    /// Return points sampled from the specified X-aligned cylindrical volume.
    PointVector SampleCylinderX(const ChVector3<T>& center, T radius, T halfHeight) {
        m_center = center;
        m_size = ChVector3<T>(halfHeight, radius, radius);
        SetFuzz();
        return Sample(CYLINDER_X);
    }

    /// Return points sampled from the specified Y-aligned cylindrical volume.
    PointVector SampleCylinderY(const ChVector3<T>& center, T radius, T halfHeight) {
        m_center = center;
        m_size = ChVector3<T>(radius, halfHeight, radius);
        SetFuzz();
        return Sample(CYLINDER_Y);
    }

    /// Return points sampled from the specified Z-aligned cylindrical volume.
    PointVector SampleCylinderZ(const ChVector3<T>& center, T radius, T halfHeight) {
        m_center = center;
        m_size = ChVector3<T>(radius, radius, halfHeight);
        SetFuzz();
        return Sample(CYLINDER_Z);
    }

    /// Get the current value of the suggested minimum separation.
    virtual T GetSeparation() const { return m_separation; }

    /// Change the suggested minimum separation for subsequent calls to Sample.
    virtual void SetSeparation(T separation) { m_separation = separation; }

  protected:
    enum VolumeType { BOX, SPHERE, CYLINDER_X, CYLINDER_Y, CYLINDER_Z };

    ChSampler(T separation) : m_separation(separation) {}

    /// Worker function for sampling the given domain.
    /// Implemented by concrete samplers.
    virtual PointVector Sample(VolumeType t) = 0;

    /// Utility function to check if a point is inside the sampling volume.
    bool accept(VolumeType t, const ChVector3<T>& p) const {
        ChVector3<T> vec = p - m_center;

        switch (t) {
            case BOX:
                return (std::abs(vec.x()) <= m_size.x() + m_fuzz) && (std::abs(vec.y()) <= m_size.y() + m_fuzz) &&
                       (std::abs(vec.z()) <= m_size.z() + m_fuzz);
            case SPHERE:
                return (vec.Length() <= m_size.x() + m_fuzz);
            case CYLINDER_X:
                return (vec.y() * vec.y() + vec.z() * vec.z() <= (m_size.y() + m_fuzz) * (m_size.y() + m_fuzz)) &&
                       (std::abs(vec.x()) <= m_size.x() + m_fuzz);
            case CYLINDER_Y:
                return (vec.z() * vec.z() + vec.x() * vec.x() <= (m_size.z() + m_fuzz) * (m_size.z() + m_fuzz)) &&
                       (std::abs(vec.y()) <= m_size.y() + m_fuzz);
            case CYLINDER_Z:
                return (vec.x() * vec.x() + vec.y() * vec.y() <= (m_size.x() + m_fuzz) * (m_size.x() + m_fuzz)) &&
                       (std::abs(vec.z()) <= m_size.z() + m_fuzz);
            default:
                return false;
        }
    }

    T m_separation;         ///< inter-particle separation
    T m_fuzz;               ///< fuzz value to account for roundoff error
    ChVector3<T> m_center;  ///< center of the sampling volume
    ChVector3<T> m_size;    ///< half dimensions of the bounding box of the sampling volume

  private:
    void SetFuzz() { m_fuzz = (m_size.x() < 1) ? (T)1e-6 * m_size.x() : (T)1e-6; }
};

/// Simple 3D grid utility class for use by the Poisson Disk sampler.
template <typename Point = ChVector3d>
class ChPDGrid {
  public:
    typedef std::pair<Point, bool> Content;

    ChPDGrid() {}

    int GetDimX() const { return m_dimX; }
    int GetDimY() const { return m_dimY; }
    int GetDimZ() const { return m_dimZ; }

    void Resize(int dimX, int dimY, int dimZ) {
        m_dimX = dimX;
        m_dimY = dimY;
        m_dimZ = dimZ;
        m_data.resize(dimX * dimY * dimZ, Content(Point(0, 0, 0), true));
    }

    void SetCellPoint(int i, int j, int k, const Point& p) {
        int ii = index(i, j, k);
        m_data[ii].first = p;
        m_data[ii].second = false;
    }

    const Point& GetCellPoint(int i, int j, int k) const { return m_data[index(i, j, k)].first; }

    bool IsCellEmpty(int i, int j, int k) const {
        if (i < 0 || i >= m_dimX || j < 0 || j >= m_dimY || k < 0 || k >= m_dimZ)
            return true;

        return m_data[index(i, j, k)].second;
    }

    Content& operator()(int i, int j, int k) { return m_data[index(i, j, k)]; }
    const Content& operator()(int i, int j, int k) const { return m_data[index(i, j, k)]; }

  private:
    int index(int i, int j, int k) const { return i * m_dimY * m_dimZ + j * m_dimZ + k; }

    int m_dimX;
    int m_dimY;
    int m_dimZ;
    std::vector<Content> m_data;
};

template <class T>
constexpr T Pi = T(3.1415926535897932385L);

/// Sampler for 3D domains (box, sphere, or cylinder) using Poisson Disk Sampling. The sampler produces a set of points
/// uniformly distributed in the specified domain such that no two points are closer than a specified distance.
///
/// 2D domains can also be sampled (rectangle or circle), by setting the size of the domain in the z direction to 0.
///
/// Based on "Fast Poisson Disk Sampling in Arbitrary Dimensions" by Robert Bridson \n
/// http://people.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
template <typename T = double>
class ChPDSampler : public ChSampler<T> {
  public:
    typedef typename Types<T>::PointVector PointVector;
    typedef typename Types<T>::PointList PointList;
    typedef typename ChSampler<T>::VolumeType VolumeType;

    /// Construct a Poisson Disk sampler with specified minimum distance.
    ChPDSampler(T separation, int pointsPerIteration = m_ppi_default)
        : ChSampler<T>(separation), m_ppi(pointsPerIteration), m_realDist(0.0, 1.0) {
        m_gridLoc.resize(3);
        m_realDist.reset();
        rengine().seed(0);
    }

    /// Set the current state of the random-number engine (default: 0).
    void SetRandomEngineSeed(unsigned int seed) { rengine().seed(seed); }

  private:
    enum Direction2D { NONE, X_DIR, Y_DIR, Z_DIR };

    /// Worker function for sampling the given domain.
    virtual PointVector Sample(VolumeType t) override {
        PointVector out_points;

        // Check 2D/3D. If the size in one direction (e.g. z) is less than the
        // minimum distance, we switch to a 2D sampling. All sample points will
        // have p.z() = m_center.z()
        if (this->m_size.z() < this->m_separation) {
            m_2D = Z_DIR;
            m_cellSize = this->m_separation / std::sqrt((T)2);
            this->m_size.z() = 0;
        } else if (this->m_size.y() < this->m_separation) {
            m_2D = Y_DIR;
            m_cellSize = this->m_separation / std::sqrt((T)2);
            this->m_size.y() = 0;
        } else if (this->m_size.x() < this->m_separation) {
            m_2D = X_DIR;
            m_cellSize = this->m_separation / std::sqrt((T)2);
            this->m_size.x() = 0;
        } else {
            m_2D = NONE;
            m_cellSize = this->m_separation / std::sqrt((T)3);
        }

        m_bl = this->m_center - this->m_size;
        m_tr = this->m_center + this->m_size;

        m_grid.Resize((int)(2 * this->m_size.x() / m_cellSize) + 1, (int)(2 * this->m_size.y() / m_cellSize) + 1,
                      (int)(2 * this->m_size.z() / m_cellSize) + 1);

        // Add the first output point (and initialize active list)
        AddFirstPoint(t, out_points);

        // As long as there are active points...
        while (m_active.size() != 0) {
            // ... select one of them at random
            std::uniform_int_distribution<int> intDist(0, (int)m_active.size() - 1);

            typename PointList::iterator point = m_active.begin();
            std::advance(point, intDist(rengine()));

            // ... attempt to add points near the active one
            bool found = false;

            for (int k = 0; k < m_ppi; k++)
                found |= AddNextPoint(t, *point, out_points);

            // ... if not possible, remove the current active point
            if (!found)
                m_active.erase(point);
        }

        return out_points;
    }

    /// Add the first point in the volume (selected randomly).
    void AddFirstPoint(VolumeType t, PointVector& out_points) {
        ChVector3<T> p;

        // Generate a random point in the domain
        do {
            p.x() = m_bl.x() + m_realDist(rengine()) * 2 * this->m_size.x();
            p.y() = m_bl.y() + m_realDist(rengine()) * 2 * this->m_size.y();
            p.z() = m_bl.z() + m_realDist(rengine()) * 2 * this->m_size.z();
        } while (!this->accept(t, p));

        // Place the point in the grid, add it to the active list, and add it
        // to output.
        MapToGrid(p);

        m_grid.SetCellPoint(m_gridLoc[0], m_gridLoc[1], m_gridLoc[2], p);
        m_active.push_back(p);
        out_points.push_back(p);
    }

    /// Attempt to add a new point, close to the specified one.
    bool AddNextPoint(VolumeType t, const ChVector3<T>& point, PointVector& out_points) {
        // Generate a random candidate point in the neighborhood of the
        // specified point.
        ChVector3<T> q = GenerateRandomNeighbor(point);

        // Check if point is in the domain.
        if (!this->accept(t, q))
            return false;

        // Check distance from candidate point to any existing point in the grid
        // (note that we only need to check 5x5x5 surrounding grid cells).
        MapToGrid(q);

        for (int i = m_gridLoc[0] - 2; i < m_gridLoc[0] + 3; i++) {
            for (int j = m_gridLoc[1] - 2; j < m_gridLoc[1] + 3; j++) {
                for (int k = m_gridLoc[2] - 2; k < m_gridLoc[2] + 3; k++) {
                    if (m_grid.IsCellEmpty(i, j, k))
                        continue;
                    ChVector3<T> dist = q - m_grid.GetCellPoint(i, j, k);
                    if (dist.Length2() < this->m_separation * this->m_separation)
                        return false;
                }
            }
        }

        // The candidate point is acceptable.
        // Place it in the grid, add it to the active list, and add it to the
        // output.
        m_grid.SetCellPoint(m_gridLoc[0], m_gridLoc[1], m_gridLoc[2], q);
        m_active.push_back(q);
        out_points.push_back(q);

        return true;
    }

    /// Return a random point in spherical anulus between sep and 2*sep centered at given point.
    ChVector3<T> GenerateRandomNeighbor(const ChVector3<T>& point) {
        T x, y, z;

        switch (m_2D) {
            case Z_DIR: {
                T radius = this->m_separation * (1 + m_realDist(rengine()));
                T angle = 2 * Pi<T> * m_realDist(rengine());
                x = point.x() + radius * std::cos(angle);
                y = point.y() + radius * std::sin(angle);
                z = this->m_center.z();
            } break;
            case Y_DIR: {
                T radius = this->m_separation * (1 + m_realDist(rengine()));
                T angle = 2 * Pi<T> * m_realDist(rengine());
                x = point.x() + radius * std::cos(angle);
                y = this->m_center.y();
                z = point.z() + radius * std::sin(angle);
            } break;
            case X_DIR: {
                T radius = this->m_separation * (1 + m_realDist(rengine()));
                T angle = 2 * Pi<T> * m_realDist(rengine());
                x = this->m_center.x();
                y = point.y() + radius * std::cos(angle);
                z = point.z() + radius * std::sin(angle);
            } break;
            default:
            case NONE: {
                T radius = this->m_separation * (1 + m_realDist(rengine()));
                T angle1 = 2 * Pi<T> * m_realDist(rengine());
                T angle2 = 2 * Pi<T> * m_realDist(rengine());
                x = point.x() + radius * std::cos(angle1) * std::sin(angle2);
                y = point.y() + radius * std::sin(angle1) * std::sin(angle2);
                z = point.z() + radius * std::cos(angle2);
            } break;
        }

        return ChVector3<T>(x, y, z);
    }

    /// Map point location to a 3D grid location.
    void MapToGrid(ChVector3<T> point) {
        m_gridLoc[0] = (int)((point.x() - m_bl.x()) / m_cellSize);
        m_gridLoc[1] = (int)((point.y() - m_bl.y()) / m_cellSize);
        m_gridLoc[2] = (int)((point.z() - m_bl.z()) / m_cellSize);
    }

    ChPDGrid<ChVector3<T>> m_grid;
    PointList m_active;

    Direction2D m_2D;   ///< 2D or 3D sampling
    ChVector3<T> m_bl;  ///< bottom-left corner of sampling domain
    ChVector3<T> m_tr;  ///< top-right corner of sampling domain      REMOVE?
    T m_cellSize;       ///< grid cell size

    std::vector<int> m_gridLoc;
    int m_ppi;  ///< maximum points per iteration

    /// Generate real numbers uniformly distributed in (0,1)
    std::uniform_real_distribution<T> m_realDist;

    static const int m_ppi_default = 30;
};

/// Poisson Disk sampler for sampling a 3D box in layers.
/// The computational efficiency of PD sampling degrades as points are added, especially for large volumes.
/// This class provides an alternative sampling method where PD sampling is done in 2D layers, separated by a specified
/// distance (padding_factor * diam). This significantly improves computational efficiency of the sampling but at the
/// cost of discarding the PD uniform distribution properties in the direction orthogonal to the layers.
template <typename T>
std::vector<ChVector3<T>> ChPDLayerSamplerBox(ChVector3<T> center,      ///< Center of axis-aligned box to fill
                                              ChVector3<T> hdims,       ///< Half-dimensions along the x, y, and z axes
                                              T diam,                   ///< Particle diameter
                                              T padding_factor = 1.02,  ///< Multiplier on particle diameter for spacing
                                              bool verbose = false      ///< Output progress during generation
) {
    T fill_bottom = center.z() - hdims.z();
    T fill_top = center.z() + hdims.z();

    // set center to bottom
    center.z() = fill_bottom;
    // 2D layer
    hdims.z() = 0;

    chrono::utils::ChPDSampler<T> sampler(diam * padding_factor);
    std::vector<ChVector3<T>> points_full;
    while (center.z() < fill_top) {
        if (verbose) {
            std::cout << "Create layer at " << center.z() << std::endl;
        }
        auto points = sampler.SampleBox(center, hdims);
        points_full.insert(points_full.end(), points.begin(), points.end());
        center.z() += diam * padding_factor;
    }
    return points_full;
}

/// Sampler for 3D volumes using a regular (equidistant) grid.
/// The grid spacing can be different in the 3 global X, Y, and Z directions.
template <typename T = double>
class ChGridSampler : public ChSampler<T> {
  public:
    typedef typename Types<T>::PointVector PointVector;
    typedef typename ChSampler<T>::VolumeType VolumeType;

    ChGridSampler(T separation) : ChSampler<T>(separation), m_sep3D(separation, separation, separation) {}
    ChGridSampler(const ChVector3<T>& separation) : ChSampler<T>(separation.x()), m_sep3D(separation) {}

    /// Change the minimum separation for subsequent calls to Sample.
    virtual void SetSeparation(T separation) override { m_sep3D = ChVector3<T>(separation, separation, separation); }

  private:
    /// Worker function for sampling the given domain.
    virtual PointVector Sample(VolumeType t) override {
        PointVector out_points;

        ChVector3<int> n;    // number of divisions in each direction
        ChVector3<T> sep3D;  // adjusted separation in each direction
        for (int i = 0; i < 3; i++) {
            auto d = 2 * this->m_size[i];
            n[i] = std::round(d / this->m_sep3D[i]);
            sep3D[i] = d / n[i];
        }

        ChVector3<T> bl = this->m_center - this->m_size;
        for (int ix = 0; ix <= n[0]; ix++) {
            for (int iy = 0; iy <= n[1]; iy++) {
                for (int iz = 0; iz <= n[2]; iz++) {
                    auto p = bl + ChVector3<T>(ix * sep3D.x(), iy * sep3D.y(), iz * sep3D.z());
                    if (this->accept(t, p))
                        out_points.push_back(p);
                }
            }
        }        

        return out_points;
    }

    ChVector3<T> m_sep3D;
};

/// Sampler for 3D volumes using a Hexagonally Close Packed structure.
template <typename T = double>
class ChHCPSampler : public ChSampler<T> {
  public:
    typedef typename Types<T>::PointVector PointVector;
    typedef typename ChSampler<T>::VolumeType VolumeType;

    ChHCPSampler(T separation) : ChSampler<T>(separation) {}

  private:
    /// Worker function for sampling the given domain.
    virtual PointVector Sample(VolumeType t) override {
        PointVector out_points;

        ChVector3<T> bl = this->m_center - this->m_size;  // start corner of sampling domain

        T dx = this->m_separation;                              // distance between two points in X direction
        T dy = this->m_separation * (T)(std::sqrt(3.0) / 2);    // distance between two rows in Y direction
        T dz = this->m_separation * (T)(std::sqrt(2.0 / 3.0));  // distance between two layers in Z direction

        int nx = (int)(2 * this->m_size.x() / dx) + 1;
        int ny = (int)(2 * this->m_size.y() / dy) + 1;
        int nz = (int)(2 * this->m_size.z() / dz) + 1;

        for (int k = 0; k < nz; k++) {
            // Y offsets for alternate layers
            T offset_y = (k % 2 == 0) ? 0 : dy / 3;
            for (int j = 0; j < ny; j++) {
                // X offset for current row and layer
                T offset_x = ((j + k) % 2 == 0) ? 0 : dx / 2;
                for (int i = 0; i < nx; i++) {
                    ChVector3<T> p = bl + ChVector3<T>(offset_x + i * dx, offset_y + j * dy, k * dz);
                    if (this->accept(t, p))
                        out_points.push_back(p);
                }
            }
        }

        return out_points;
    }
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
