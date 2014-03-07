#ifndef CH_UTILS_SAMPLERS_H
#define CH_UTILS_SAMPLERS_H

#include <random>
#include <cmath>
#include <vector>
#include <list>
#include <utility>
#include <time.h>

#include "core/ChVector.h"


namespace chrono {
namespace utils {


const double Pi = 3.1415926535897932384626433832795;

enum SamplingType {
	REGULAR_GRID,
	POISSON_DISK
};


// -------------------------------------------------------------------------------
// Type definitions
// -------------------------------------------------------------------------------

// Until Visual Studio supports alias templates (VS2013 does) if we want to
// "share" a typedef across different template classes, we use this structure.
template <typename T>
struct Types {
	typedef std::vector<ChVector<T> >  PointVector;
	typedef std::list<ChVector<T> >    PointList;
};

// Convenience shrotcuts
typedef Types<double>::PointVector  PointVectorD;
typedef Types<float>::PointVector   PointVectorF;


// -------------------------------------------------------------------------------
// Sampler
//
// Base class for different types of point samplers
// -------------------------------------------------------------------------------

template <typename T>
class Sampler {
public:
	typedef typename Types<T>::PointVector PointVector;

	PointVector SampleBox(const ChVector<T>& center, const ChVector<T>& halfDim)
	{
		m_center = center;
		m_size = halfDim;
		return Sample(BOX);
	}

	PointVector SampleSphere(const ChVector<T>& center, T radius)
	{
		m_center = center;
		m_size = ChVector<T>(radius, radius, radius);
		return Sample(SPHERE);
	}

	PointVector SampleCylinder(const ChVector<T>& center, T radius, T halfHeight)
	{
		m_center = center;
		m_size = ChVector<T>(radius, radius, halfHeight);
		return Sample(CYLINDER);
	}

protected:
	enum VolumeType {
		BOX,
		SPHERE,
		CYLINDER
	};

	virtual PointVector Sample(VolumeType t) = 0;

	// Utility function to check if a point is inside the sampling volume
	bool accept(VolumeType t, const ChVector<T>& p)
	{
		ChVector<T> vec = p - m_center;
		T fuzz = 1e-6;

		switch (t) {
		case BOX:
			return (std::abs(vec.x) <= m_size.x + fuzz) &&
			       (std::abs(vec.y) <= m_size.y + fuzz) &&
			       (std::abs(vec.z) <= m_size.z + fuzz);
		case SPHERE:
			return (vec.Length2() <= m_size.x * m_size.x);
		case CYLINDER:
			return (vec.x * vec.x + vec.y * vec.y <= m_size.x * m_size.x) &&
			       (std::abs(vec.z) <= m_size.z + fuzz);
		default:
			return false;
		}
	}

	ChVector<T> m_center;     ///< center of the sampling volume
	ChVector<T> m_size;       ///< half dimensions of the bounding box of the sampling volume
};


// -------------------------------------------------------------------------------
// PDGrid
//
// Simple 3D grid utility class for use by the Poisson Disk sampler.
// -------------------------------------------------------------------------------

template <typename Point = ChVector<double> >
class PDGrid {
public:
	typedef std::pair<Point, bool> Content;

	PDGrid() {}

	int GetDimX() const {return m_dimX;}
	int GetDimY() const {return m_dimY;}
	int GetDimZ() const {return m_dimZ;}

	void Resize(int dimX, int dimY, int dimZ) {
		m_dimX = dimX;
		m_dimY = dimY;
		m_dimZ = dimZ;
		m_data.resize(dimX * dimY * dimZ, Content(Point(0,0,0), true));
	}

	void SetCellPoint(int i, int j, int k, const Point& p) {
		int ii = index(i, j, k);
		m_data[ii].first = p;
		m_data[ii].second = false;
	}

	const Point& GetCellPoint(int i, int j, int k) const
	{
		return m_data[index(i,j,k)].first;
	}

	bool IsCellEmpty(int i, int j, int k) const
	{
		if (i < 0 || i >= m_dimX || j < 0 || j >= m_dimY || k < 0 || k >= m_dimZ)
			return true;

		return m_data[index(i,j,k)].second;
	}

	Content&       operator()(int i, int j, int k)        {return m_data[index(i,j,k)];}
	const Content& operator()(int i, int j, int k) const  {return m_data[index(i,j,k)];}

private:
	int  index(int i, int j, int k) const  {return i * m_dimY * m_dimZ + j * m_dimZ + k;}

	int                  m_dimX;
	int                  m_dimY;
	int                  m_dimZ;
	std::vector<Content> m_data;
};


// -------------------------------------------------------------------------------
// PDSampler
//
// A class to sample 3D domains (box, sphere, or cylinder) using Poisson Disk
// Sampling. The sampler produces a set of points uniformly distributed in the
// specified domain such that no two points are closer than a specified distance.
//
// 2D domains can also be sampled (rectangle or circle), by setting the size of
// the domain in the z direction to 0.
//
// Based on "Fast Poisson Disk Sampling in Arbitrary Dimensions" by Robert Bridson
// http://people.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
// -------------------------------------------------------------------------------

template <typename T = double>
class PDSampler : public Sampler<T> {
public:
	typedef typename Types<T>::PointVector  PointVector;
	typedef typename Types<T>::PointList    PointList;
	typedef typename Sampler<T>::VolumeType VolumeType;

	PDSampler(T   minDist,
	          int pointsPerIteration = m_ppi_default)
	:	m_minDist(minDist),
		m_ppi(pointsPerIteration),
		m_realDist(0.0, 1.0)
	{
		m_gridLoc.resize(3);
	}

private:
	// This is the worker function for sampling the given domain.
	virtual PointVector Sample(VolumeType t)
	{
		PointVector out_points;

		// Check 2D/3D. If the size in the z direction is less than the minimum distance,
		// we switch to a 2D sampling. All sample points will have p.z = m_center.z
		assert(m_size.x > m_minDist);
		assert(m_size.y > m_minDist);

		if (m_size.z < m_minDist) {
			m_2D = true;
			m_cellSize = m_minDist / std::sqrt(2.0);
			m_size.z = 0;
		} else {
			m_2D = false;
			m_cellSize = m_minDist / std::sqrt(3.0);
		}


		m_bl = m_center - m_size;
		m_tr = m_center + m_size;

		m_grid.Resize((int) (2 * m_size.x / m_cellSize) + 1,
		              (int) (2 * m_size.y / m_cellSize) + 1,
		              (int) (2 * m_size.z / m_cellSize) + 1);

		// Add the first output point (and initialize active list)
		AddFirstPoint(t, out_points);

		// As long as there are active points...
		while (m_active.size() != 0) {
			// ... select one of them at random
			std::uniform_int_distribution<int> intDist(0, m_active.size()-1);

			typename PointList::iterator point = m_active.begin();
			std::advance(point, intDist(m_generator));

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

	// This function adds the first point in the volume (randomly)
	void AddFirstPoint(VolumeType t, PointVector& out_points)
	{
		ChVector<T> p;

		// Generate a random point in the domain
		do {
			p.x = m_bl.x + m_realDist(m_generator) * 2 * m_size.x;
			p.y = m_bl.y + m_realDist(m_generator) * 2 * m_size.y;
			p.z = m_bl.z + m_realDist(m_generator) * 2 * m_size.z;
		} while (!accept(t, p));

		// Place the point in the grid, add it to the active list, and add it to output.
		MapToGrid(p);

		m_grid.SetCellPoint(m_gridLoc[0], m_gridLoc[1], m_gridLoc[2], p);
		m_active.push_back(p);
		out_points.push_back(p);
	}

	// Attempt to add a new point close to the specified one.
	bool AddNextPoint(VolumeType t, const ChVector<T>& point, PointVector& out_points)
	{
		// Generate a random candidate point in the neighborhood of the specified point.
		ChVector<T> q = GenerateRandomNeighbor(point);

		// Check if point is in the domain.
		if (!accept(t, q))
			return false;

		// Check distance from candidate point to any existing point in the grid
		// (note that we only need to check 5x5x5 surrounding grid cells).
		MapToGrid(q);

		for (int i = m_gridLoc[0] - 2; i < m_gridLoc[0] + 3; i++) {
			for (int j = m_gridLoc[1] - 2; j < m_gridLoc[1] + 3; j++) {
				for (int k = m_gridLoc[2] - 2; k < m_gridLoc[2] + 3; k++) {
					if (m_grid.IsCellEmpty(i, j, k))
						continue;
					ChVector<T> dist = q - m_grid.GetCellPoint(i, j, k);
					if (dist.Length2() < m_minDist * m_minDist)
						return false;
				}
			}
		}

		// The candidate point is acceptable.
		// Place it in the grid, add it to the active list, and add it to the output.
		m_grid.SetCellPoint(m_gridLoc[0], m_gridLoc[1], m_gridLoc[2], q);
		m_active.push_back(q);
		out_points.push_back(q);

		return true;
	}

	// Return random point in spherical anulus between minDist and 2*minDist
	// centered at given point
	ChVector<T> GenerateRandomNeighbor(const ChVector<T>& point)
	{
		T x, y, z;

		if (m_2D) {
			T radius = m_minDist * (1 + m_realDist(m_generator));
			T angle = 2 * Pi * m_realDist(m_generator);
			x = point.x + radius * std::cos(angle);
			y = point.y + radius * std::sin(angle);
			z = m_center.z;
		} else {
			T radius = m_minDist * (1 + m_realDist(m_generator));
			T angle1 = 2 * Pi * m_realDist(m_generator);
			T angle2 = 2 * Pi * m_realDist(m_generator);
			x = point.x + radius * std::cos(angle1) * std::sin(angle2);
			y = point.y + radius * std::sin(angle1) * std::sin(angle2);
			z = point.z + radius * std::cos(angle2);
		}

		return ChVector<T>(x, y, z);
	}

	// Map point location to a 3D grid location
	void MapToGrid(ChVector<T> point)
	{
		m_gridLoc[0] = (int) ((point.x - m_bl.x) / m_cellSize);
		m_gridLoc[1] = (int) ((point.y - m_bl.y) / m_cellSize);
		m_gridLoc[2] = (int) ((point.z - m_bl.z) / m_cellSize);
	}


	PDGrid<ChVector<T> >  m_grid;
	PointList             m_active;

	bool        m_2D;          ///< 2D or 3D sampling
	ChVector<T> m_bl;          ///< bottom-left corner of sampling domain
	ChVector<T> m_tr;          ///< top-right corner of sampling domain      REMOVE?
	T           m_cellSize;    ///< grid cell size
	T           m_minDist;     ///< minimum distance between generated points

	std::vector<int> m_gridLoc;
	int m_ppi;                 ///< maximum points per iteration

	/// Generate real numbers uniformly distributed in (0,1)
	std::default_random_engine         m_generator;
	std::uniform_real_distribution<T>  m_realDist;

	static const int m_ppi_default = 30;
};


// -------------------------------------------------------------------------------
// GridSampler
//
// A class to generate points in a regular grid within a 3D domain. Grid spacing
// can be different in the three directions.
// -------------------------------------------------------------------------------

template <typename T = double>
class GridSampler : public Sampler<T> {
public:
	typedef typename Types<T>::PointVector  PointVector;
	typedef typename Sampler<T>::VolumeType VolumeType;

	GridSampler(T spacing) : m_spacing(spacing, spacing, spacing) {}
	GridSampler(const ChVector<T>& spacing) : m_spacing(spacing) {}

private:
	virtual PointVector Sample(VolumeType t)
	{
		PointVector out_points;

		ChVector<T> bl = m_center - m_size;

		int nx = (int) (2 * m_size.x / m_spacing.x) + 1;
		int ny = (int) (2 * m_size.y / m_spacing.y) + 1;
		int nz = (int) (2 * m_size.z / m_spacing.z) + 1;

		for (int i = 0; i < nx; i++) {
			for (int j = 0; j < ny; j++) {
				for (int k = 0; k < nz; k++) {
					ChVector<T> p = bl + ChVector<T>(i*m_spacing.x, j*m_spacing.y, k*m_spacing.z);
					if (accept(t, p)) 
						out_points.push_back(p);
				}
			}
		}

		return out_points;
	}

	ChVector<T> m_spacing;
};



} // end namespace utils
} // end namespace chrono


#endif
