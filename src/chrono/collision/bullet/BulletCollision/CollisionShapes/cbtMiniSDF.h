#ifndef MINISDF_H
#define MINISDF_H

#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtAabbUtil2.h"
#include "LinearMath/cbtAlignedObjectArray.h"

struct cbtMultiIndex
{
	unsigned int ijk[3];
};

struct cbtAlignedBox3d
{
	cbtVector3 m_min;
	cbtVector3 m_max;

	const cbtVector3& min() const
	{
		return m_min;
	}

	const cbtVector3& max() const
	{
		return m_max;
	}

	bool contains(const cbtVector3& x) const
	{
		return TestPointAgainstAabb2(m_min, m_max, x);
	}

	cbtAlignedBox3d(const cbtVector3& mn, const cbtVector3& mx)
		: m_min(mn),
		  m_max(mx)
	{
	}

	cbtAlignedBox3d()
	{
	}
};

struct cbtShapeMatrix
{
	double m_vec[32];

	inline double& operator[](int i)
	{
		return m_vec[i];
	}

	inline const double& operator[](int i) const
	{
		return m_vec[i];
	}
};

struct cbtShapeGradients
{
	cbtVector3 m_vec[32];

	void topRowsDivide(int row, double denom)
	{
		for (int i = 0; i < row; i++)
		{
			m_vec[i] /= denom;
		}
	}

	void bottomRowsMul(int row, double val)
	{
		for (int i = 32 - row; i < 32; i++)
		{
			m_vec[i] *= val;
		}
	}

	inline cbtScalar& operator()(int i, int j)
	{
		return m_vec[i][j];
	}
};

struct cbtCell32
{
	unsigned int m_cells[32];
};

struct cbtMiniSDF
{
	cbtAlignedBox3d m_domain;
	unsigned int m_resolution[3];
	cbtVector3 m_cell_size;
	cbtVector3 m_inv_cell_size;
	std::size_t m_n_cells;
	std::size_t m_n_fields;
	bool m_isValid;

	cbtAlignedObjectArray<cbtAlignedObjectArray<double> > m_nodes;
	cbtAlignedObjectArray<cbtAlignedObjectArray<cbtCell32> > m_cells;
	cbtAlignedObjectArray<cbtAlignedObjectArray<unsigned int> > m_cell_map;

	cbtMiniSDF()
		: m_isValid(false)
	{
	}
	bool load(const char* data, int size);
	bool isValid() const
	{
		return m_isValid;
	}
	unsigned int multiToSingleIndex(cbtMultiIndex const& ijk) const;

	cbtAlignedBox3d subdomain(cbtMultiIndex const& ijk) const;

	cbtMultiIndex singleToMultiIndex(unsigned int l) const;

	cbtAlignedBox3d subdomain(unsigned int l) const;

	cbtShapeMatrix
	shape_function_(cbtVector3 const& xi, cbtShapeGradients* gradient = 0) const;

	bool interpolate(unsigned int field_id, double& dist, cbtVector3 const& x, cbtVector3* gradient) const;
};

#endif  //MINISDF_H
