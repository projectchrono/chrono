#ifndef CH_UTILS_GENERATE_H
#define CH_UTILS_GENERATE_H

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
// Grid2D
//
// Simple 2D grid utility class for use by the Poisson Disk sampler.
// -------------------------------------------------------------------------------
template <typename Point = ChVector<double> >
class Grid2D {
public:
	typedef std::pair<Point, bool> Content;

	Grid2D() {}
	~Grid2D() {}

	int GetDimX() const {return m_dimX;}
	int GetDimY() const {return m_dimY;}

	void Resize(int dimX, int dimY) {
		m_dimX = dimX;
		m_dimY = dimY;
		m_data.resize(dimX * dimY, Content(Point(0,0,0), true));
	}

	void SetCellPoint(int i, int j, const Point& p) {
		int ii = index(i, j);
		m_data[ii].first = p;
		m_data[ii].second = false;
	}

	const Point& GetCellPoint(int i, int j) const
	{
		return m_data[index(i,j)].first;
	}

	bool IsCellEmpty(int i, int j) const
	{
		if (i < 0 || i >= m_dimX || j < 0 || j >= m_dimY)
			return true;

		return m_data[index(i,j)].second;
	}

	Content&       operator()(int i, int j)        {return m_data[index(i,j)];}
	const Content& operator()(int i, int j) const  {return m_data[index(i,j)];}

private:
	int  index(int i, int j) const  {return i * m_dimY + j;}

	int                  m_dimX;
	int                  m_dimY;
	std::vector<Content> m_data;
};


// -------------------------------------------------------------------------------
// PDSampler2D
//
// A class to sample 2D domains (rectangle or circle) using Poisson Disk Sampling.
// The sampler produces a set of points uniformly distributed in the specified
// domain such that no two points are closer than a specified distance.
//
// Based on "Fast Poisson Disk Sampling in Arbitrary Dimensions" by Robert Bridson
// http://people.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
// -------------------------------------------------------------------------------

template <typename T = double>
class PDSampler2D {
public:
	typedef typename Types<T>::PointVector PointVector;
	typedef typename Types<T>::PointList   PointList;

	typedef std::pair<int, int>            GridLocation;

	PDSampler2D(T   minDist,
	            int pointsPerIteration = m_ppi_default)
	:	m_minDist(minDist),
		m_cellSize(minDist / std::sqrt(2.0)),
		m_ppi(pointsPerIteration),
		m_realDist(0.0, 1.0)
	{
	}

	PointVector SampleCircle(const ChVector<T>& center, T radius)
	{
		m_center = center;
		m_size = ChVector<T>(2 * radius, 2 * radius, 0);
		m_maxDist2 = radius * radius;
		return Sample();
	}

	PointVector SampleRectangle(const ChVector<T>& center, const ChVector<T>& halfDim)
	{
		m_center = center;
		m_size = halfDim * 2;
		m_maxDist2 = -1;
		return Sample();
	}

	PointVector Sample()
	{
		m_height = m_center.z;
		m_bl = m_center - m_size / 2;
		m_tr = m_center + m_size / 2;

		m_grid.Resize((int) (m_size.x / m_cellSize) + 1, (int) (m_size.y / m_cellSize) + 1);

		PointVector out_points;

		// Add the first output point (and initialize active list)
		AddFirstPoint(out_points);

		// As long as there are active points...
		while (m_active.size() != 0) {
			// ... select one of them at random
			std::uniform_int_distribution<int> intDist(0, m_active.size()-1);

			PointList::iterator point = m_active.begin();
			std::advance(point, intDist(m_generator));

			// ... attempt to add points near the active one
			bool found = false;

			for (int k = 0; k < m_ppi; k++)
				found |= AddNextPoint(*point, out_points);

			// ... if not possible, remove the current active point
			if (!found)
				m_active.erase(point);
		}

		return out_points;
	}

	void AddFirstPoint(PointVector& out_points)
	{
		ChVector<T> p;

		// Generate a random point in the domain
		while (true) {
			p.x = m_bl.x + m_realDist(m_generator) * m_size.x;
			p.y = m_bl.y + m_realDist(m_generator) * m_size.y;
			p.z = m_height;

			if (m_maxDist2 < 0)
				break;

			ChVector<T> dist = p - m_center;
			if (dist.Length2() <= m_maxDist2)
				break;
		}

		// Place the point in the grid, add it to the active list, and add it to output.
		GridLocation index = MapToGrid(p);

		m_grid.SetCellPoint(index.first, index.second, p);
		m_active.push_back(p);
		out_points.push_back(p);
	}

	bool AddNextPoint(const ChVector<T>& point, PointVector& out_points)
	{
		// Generate a random candidate point in the neighborhood of the specified point.
		ChVector<T> q = GenerateRandomNeighbor(point);

		// Check if point is in the domain
		if (q.x < m_bl.x || q.x > m_tr.x || q.y < m_bl.y || q.y > m_tr.y)
			return false;

		if (m_maxDist2 > 0) {
			ChVector<T> dist = q - m_center;
			if (dist.Length2() > m_maxDist2)
				return false;
		}

		// Find grid location for this point
		GridLocation index = MapToGrid(q);

		// Check distance from candidate point to any existing point in the grid
		// (note that we only need to check 5x5 surrounding grid cells)
		for (int i = index.first - 2; i < index.first + 3; i++) {
			for (int j = index.second - 2; j < index.second + 3; j++) {
				if (m_grid.IsCellEmpty(i, j))
					continue;
				ChVector<T> dist = q - m_grid.GetCellPoint(i, j);
				if (dist.Length2() < m_minDist * m_minDist)
					return false;
			}
		}

		// The candidate point is acceptable.
		// Place it in the grid, add it to the active list, and add it to the output.
		m_grid.SetCellPoint(index.first, index.second, q);
		m_active.push_back(q);
		out_points.push_back(q);

		return true;
	}

	// Return random point in circular anulus between minDist and 2*minDist
	ChVector<T> GenerateRandomNeighbor(const ChVector<T>& center)
	{
		T radius = m_minDist * (1 + m_realDist(m_generator));
		T angle = 2 * Pi * m_realDist(m_generator);
		return ChVector<T>(center.x + radius * std::cos(angle), center.y + radius * std::sin(angle), m_height);
	}

	// Map point location to a 2D grid location
	GridLocation MapToGrid(ChVector<T> point)
	{
		return GridLocation((int) ((point.x - m_bl.x) / m_cellSize),
		                    (int) ((point.y - m_bl.y) / m_cellSize));
	}

private:

	Grid2D<ChVector<T> >  m_grid;
	PointList             m_active;

	ChVector<T> m_bl;          ///< bottom-left corner of sampling domain
	ChVector<T> m_tr;          ///< top-right corner of sampling domain
	ChVector<T> m_size;        ///< dimensions of sampling domain (length and width)
	ChVector<T> m_center;      ///< center of sampling domain
	T           m_height;      ///< height (z coordinate) of sampling domain
	T           m_cellSize;    ///< grid cell size
	T           m_minDist;     ///< minimum distance between generated points
	T           m_maxDist2;    ///< squared maximum distance from center

	int m_ppi;                 ///< maximum points per iteration

	/// Generate real numbers uniformly distributed in (0,1)
	std::default_random_engine         m_generator;
	std::uniform_real_distribution<T>  m_realDist;

	static const int m_ppi_default = 30;
};


// -------------------------------------------------------------------------------
// GridSampler2D
//
// A class to generate points in a regular grid within a 2D domain (rectangle or
// circle). Grid spacing can be different in the two directions.
// -------------------------------------------------------------------------------

template <typename T = double>
class GridSampler2D {
public:
	typedef typename Types<T>::PointVector PointVector;

	GridSampler2D(T delX,
	              T delY)
	:	m_delX(delX),
		m_delY(delY)
	{
	}

	PointVector SampleCircle(const ChVector<T>& center, T radius)
	{
		m_center = center;
		m_size = ChVector<T>(2 * radius, 2 * radius, 0);
		m_maxDist2 = radius * radius;
		return Sample();
	}

	PointVector SampleRectangle(const ChVector<T>& center, const ChVector<T>& halfDim)
	{
		m_center = center;
		m_size = halfDim * 2;
		m_maxDist2 = -1;
		return Sample();
	}

	PointVector Sample()
	{
		T height = m_center.z;
		ChVector<T> bl = m_center - m_size / 2;

		PointVector out_points;

		int nx = (int) (m_size.x / m_delX) + 1;
		int ny = (int) (m_size.y / m_delY) + 1;

		for (int i = 0; i < nx; i++) {
			for (int j = 0; j < ny; j++) {
				ChVector<T> p = bl + ChVector<T>(i*m_delX, j*m_delY, height);
				if (m_maxDist2 > 0) {
					ChVector<T> dist = p - m_center;
					if (dist.Length2() > m_maxDist2)
						continue;
				}
				out_points.push_back(p);
			}
		}

		return out_points;
	}

private:
	T  m_delX;
	T  m_delY;
	T  m_maxDist2;
	ChVector<T> m_center;
	ChVector<T> m_size;
};



} // end namespace utils
} // end namespace chrono


#endif
