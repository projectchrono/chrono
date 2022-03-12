#include "cbtMiniSDF.h"

//
//Based on code from DiscreGrid, https://github.com/InteractiveComputerGraphics/Discregrid
//example:
//GenerateSDF.exe -r "32 32 32" -d "-1.6 -1.6 -.6 1.6 1.6 .6" concave_box.obj
//The MIT License (MIT)
//
//Copyright (c) 2017 Dan Koschier
//

#include <limits.h>
#include <string.h>  //memcpy

struct cbtSdfDataStream
{
	const char* m_data;
	int m_size;

	int m_currentOffset;

	cbtSdfDataStream(const char* data, int size)
		: m_data(data),
		  m_size(size),
		  m_currentOffset(0)
	{
	}

	template <class T>
	bool read(T& val)
	{
		int bytes = sizeof(T);
		if (m_currentOffset + bytes <= m_size)
		{
			char* dest = (char*)&val;
			memcpy(dest, &m_data[m_currentOffset], bytes);
			m_currentOffset += bytes;
			return true;
		}
		cbtAssert(0);
		return false;
	}
};

bool cbtMiniSDF::load(const char* data, int size)
{
	int fileSize = -1;

	cbtSdfDataStream ds(data, size);
	{
		double buf[6];
		ds.read(buf);
		m_domain.m_min[0] = buf[0];
		m_domain.m_min[1] = buf[1];
		m_domain.m_min[2] = buf[2];
		m_domain.m_min[3] = 0;
		m_domain.m_max[0] = buf[3];
		m_domain.m_max[1] = buf[4];
		m_domain.m_max[2] = buf[5];
		m_domain.m_max[3] = 0;
	}
	{
		unsigned int buf2[3];
		ds.read(buf2);
		m_resolution[0] = buf2[0];
		m_resolution[1] = buf2[1];
		m_resolution[2] = buf2[2];
	}
	{
		double buf[3];
		ds.read(buf);
		m_cell_size[0] = buf[0];
		m_cell_size[1] = buf[1];
		m_cell_size[2] = buf[2];
	}
	{
		double buf[3];
		ds.read(buf);
		m_inv_cell_size[0] = buf[0];
		m_inv_cell_size[1] = buf[1];
		m_inv_cell_size[2] = buf[2];
	}
	{
		unsigned long long int cells;
		ds.read(cells);
		m_n_cells = cells;
	}
	{
		unsigned long long int fields;
		ds.read(fields);
		m_n_fields = fields;
	}

	unsigned long long int nodes0;
	std::size_t n_nodes0;
	ds.read(nodes0);
	n_nodes0 = nodes0;
	if (n_nodes0 > 1024 * 1024 * 1024)
	{
		return m_isValid;
	}
	m_nodes.resize(n_nodes0);
	for (unsigned int i = 0; i < n_nodes0; i++)
	{
		unsigned long long int n_nodes1;
		ds.read(n_nodes1);
		cbtAlignedObjectArray<double>& nodes = m_nodes[i];
		nodes.resize(n_nodes1);
		for (int j = 0; j < nodes.size(); j++)
		{
			double& node = nodes[j];
			ds.read(node);
		}
	}

	unsigned long long int n_cells0;
	ds.read(n_cells0);
	m_cells.resize(n_cells0);
	for (int i = 0; i < n_cells0; i++)
	{
		unsigned long long int n_cells1;
		cbtAlignedObjectArray<cbtCell32>& cells = m_cells[i];
		ds.read(n_cells1);
		cells.resize(n_cells1);
		for (int j = 0; j < n_cells1; j++)
		{
			cbtCell32& cell = cells[j];
			ds.read(cell);
		}
	}

	{
		unsigned long long int n_cell_maps0;
		ds.read(n_cell_maps0);

		m_cell_map.resize(n_cell_maps0);
		for (int i = 0; i < n_cell_maps0; i++)
		{
			unsigned long long int n_cell_maps1;
			cbtAlignedObjectArray<unsigned int>& cell_maps = m_cell_map[i];
			ds.read(n_cell_maps1);
			cell_maps.resize(n_cell_maps1);
			for (int j = 0; j < n_cell_maps1; j++)
			{
				unsigned int& cell_map = cell_maps[j];
				ds.read(cell_map);
			}
		}
	}

	m_isValid = (ds.m_currentOffset == ds.m_size);
	return m_isValid;
}

unsigned int cbtMiniSDF::multiToSingleIndex(cbtMultiIndex const& ijk) const
{
	return m_resolution[1] * m_resolution[0] * ijk.ijk[2] + m_resolution[0] * ijk.ijk[1] + ijk.ijk[0];
}

cbtAlignedBox3d
cbtMiniSDF::subdomain(cbtMultiIndex const& ijk) const
{
	cbtAssert(m_isValid);
	cbtVector3 tmp;
	tmp.m_floats[0] = m_cell_size[0] * (double)ijk.ijk[0];
	tmp.m_floats[1] = m_cell_size[1] * (double)ijk.ijk[1];
	tmp.m_floats[2] = m_cell_size[2] * (double)ijk.ijk[2];

	cbtVector3 origin = m_domain.min() + tmp;

	cbtAlignedBox3d box = cbtAlignedBox3d(origin, origin + m_cell_size);
	return box;
}

cbtMultiIndex
cbtMiniSDF::singleToMultiIndex(unsigned int l) const
{
	cbtAssert(m_isValid);
	unsigned int n01 = m_resolution[0] * m_resolution[1];
	unsigned int k = l / n01;
	unsigned int temp = l % n01;
	unsigned int j = temp / m_resolution[0];
	unsigned int i = temp % m_resolution[0];
	cbtMultiIndex mi;
	mi.ijk[0] = i;
	mi.ijk[1] = j;
	mi.ijk[2] = k;
	return mi;
}

cbtAlignedBox3d
cbtMiniSDF::subdomain(unsigned int l) const
{
	cbtAssert(m_isValid);
	return subdomain(singleToMultiIndex(l));
}

cbtShapeMatrix
cbtMiniSDF::shape_function_(cbtVector3 const& xi, cbtShapeGradients* gradient) const
{
	cbtAssert(m_isValid);
	cbtShapeMatrix res;

	cbtScalar x = xi[0];
	cbtScalar y = xi[1];
	cbtScalar z = xi[2];

	cbtScalar x2 = x * x;
	cbtScalar y2 = y * y;
	cbtScalar z2 = z * z;

	cbtScalar _1mx = 1.0 - x;
	cbtScalar _1my = 1.0 - y;
	cbtScalar _1mz = 1.0 - z;

	cbtScalar _1px = 1.0 + x;
	cbtScalar _1py = 1.0 + y;
	cbtScalar _1pz = 1.0 + z;

	cbtScalar _1m3x = 1.0 - 3.0 * x;
	cbtScalar _1m3y = 1.0 - 3.0 * y;
	cbtScalar _1m3z = 1.0 - 3.0 * z;

	cbtScalar _1p3x = 1.0 + 3.0 * x;
	cbtScalar _1p3y = 1.0 + 3.0 * y;
	cbtScalar _1p3z = 1.0 + 3.0 * z;

	cbtScalar _1mxt1my = _1mx * _1my;
	cbtScalar _1mxt1py = _1mx * _1py;
	cbtScalar _1pxt1my = _1px * _1my;
	cbtScalar _1pxt1py = _1px * _1py;

	cbtScalar _1mxt1mz = _1mx * _1mz;
	cbtScalar _1mxt1pz = _1mx * _1pz;
	cbtScalar _1pxt1mz = _1px * _1mz;
	cbtScalar _1pxt1pz = _1px * _1pz;

	cbtScalar _1myt1mz = _1my * _1mz;
	cbtScalar _1myt1pz = _1my * _1pz;
	cbtScalar _1pyt1mz = _1py * _1mz;
	cbtScalar _1pyt1pz = _1py * _1pz;

	cbtScalar _1mx2 = 1.0 - x2;
	cbtScalar _1my2 = 1.0 - y2;
	cbtScalar _1mz2 = 1.0 - z2;

	// Corner nodes.
	cbtScalar fac = 1.0 / 64.0 * (9.0 * (x2 + y2 + z2) - 19.0);
	res[0] = fac * _1mxt1my * _1mz;
	res[1] = fac * _1pxt1my * _1mz;
	res[2] = fac * _1mxt1py * _1mz;
	res[3] = fac * _1pxt1py * _1mz;
	res[4] = fac * _1mxt1my * _1pz;
	res[5] = fac * _1pxt1my * _1pz;
	res[6] = fac * _1mxt1py * _1pz;
	res[7] = fac * _1pxt1py * _1pz;

	// Edge nodes.

	fac = 9.0 / 64.0 * _1mx2;
	cbtScalar fact1m3x = fac * _1m3x;
	cbtScalar fact1p3x = fac * _1p3x;
	res[8] = fact1m3x * _1myt1mz;
	res[9] = fact1p3x * _1myt1mz;
	res[10] = fact1m3x * _1myt1pz;
	res[11] = fact1p3x * _1myt1pz;
	res[12] = fact1m3x * _1pyt1mz;
	res[13] = fact1p3x * _1pyt1mz;
	res[14] = fact1m3x * _1pyt1pz;
	res[15] = fact1p3x * _1pyt1pz;

	fac = 9.0 / 64.0 * _1my2;
	cbtScalar fact1m3y = fac * _1m3y;
	cbtScalar fact1p3y = fac * _1p3y;
	res[16] = fact1m3y * _1mxt1mz;
	res[17] = fact1p3y * _1mxt1mz;
	res[18] = fact1m3y * _1pxt1mz;
	res[19] = fact1p3y * _1pxt1mz;
	res[20] = fact1m3y * _1mxt1pz;
	res[21] = fact1p3y * _1mxt1pz;
	res[22] = fact1m3y * _1pxt1pz;
	res[23] = fact1p3y * _1pxt1pz;

	fac = 9.0 / 64.0 * _1mz2;
	cbtScalar fact1m3z = fac * _1m3z;
	cbtScalar fact1p3z = fac * _1p3z;
	res[24] = fact1m3z * _1mxt1my;
	res[25] = fact1p3z * _1mxt1my;
	res[26] = fact1m3z * _1mxt1py;
	res[27] = fact1p3z * _1mxt1py;
	res[28] = fact1m3z * _1pxt1my;
	res[29] = fact1p3z * _1pxt1my;
	res[30] = fact1m3z * _1pxt1py;
	res[31] = fact1p3z * _1pxt1py;

	if (gradient)
	{
		cbtShapeGradients& dN = *gradient;

		cbtScalar _9t3x2py2pz2m19 = 9.0 * (3.0 * x2 + y2 + z2) - 19.0;
		cbtScalar _9tx2p3y2pz2m19 = 9.0 * (x2 + 3.0 * y2 + z2) - 19.0;
		cbtScalar _9tx2py2p3z2m19 = 9.0 * (x2 + y2 + 3.0 * z2) - 19.0;
		cbtScalar _18x = 18.0 * x;
		cbtScalar _18y = 18.0 * y;
		cbtScalar _18z = 18.0 * z;

		cbtScalar _3m9x2 = 3.0 - 9.0 * x2;
		cbtScalar _3m9y2 = 3.0 - 9.0 * y2;
		cbtScalar _3m9z2 = 3.0 - 9.0 * z2;

		cbtScalar _2x = 2.0 * x;
		cbtScalar _2y = 2.0 * y;
		cbtScalar _2z = 2.0 * z;

		cbtScalar _18xm9t3x2py2pz2m19 = _18x - _9t3x2py2pz2m19;
		cbtScalar _18xp9t3x2py2pz2m19 = _18x + _9t3x2py2pz2m19;
		cbtScalar _18ym9tx2p3y2pz2m19 = _18y - _9tx2p3y2pz2m19;
		cbtScalar _18yp9tx2p3y2pz2m19 = _18y + _9tx2p3y2pz2m19;
		cbtScalar _18zm9tx2py2p3z2m19 = _18z - _9tx2py2p3z2m19;
		cbtScalar _18zp9tx2py2p3z2m19 = _18z + _9tx2py2p3z2m19;

		dN(0, 0) = _18xm9t3x2py2pz2m19 * _1myt1mz;
		dN(0, 1) = _1mxt1mz * _18ym9tx2p3y2pz2m19;
		dN(0, 2) = _1mxt1my * _18zm9tx2py2p3z2m19;
		dN(1, 0) = _18xp9t3x2py2pz2m19 * _1myt1mz;
		dN(1, 1) = _1pxt1mz * _18ym9tx2p3y2pz2m19;
		dN(1, 2) = _1pxt1my * _18zm9tx2py2p3z2m19;
		dN(2, 0) = _18xm9t3x2py2pz2m19 * _1pyt1mz;
		dN(2, 1) = _1mxt1mz * _18yp9tx2p3y2pz2m19;
		dN(2, 2) = _1mxt1py * _18zm9tx2py2p3z2m19;
		dN(3, 0) = _18xp9t3x2py2pz2m19 * _1pyt1mz;
		dN(3, 1) = _1pxt1mz * _18yp9tx2p3y2pz2m19;
		dN(3, 2) = _1pxt1py * _18zm9tx2py2p3z2m19;
		dN(4, 0) = _18xm9t3x2py2pz2m19 * _1myt1pz;
		dN(4, 1) = _1mxt1pz * _18ym9tx2p3y2pz2m19;
		dN(4, 2) = _1mxt1my * _18zp9tx2py2p3z2m19;
		dN(5, 0) = _18xp9t3x2py2pz2m19 * _1myt1pz;
		dN(5, 1) = _1pxt1pz * _18ym9tx2p3y2pz2m19;
		dN(5, 2) = _1pxt1my * _18zp9tx2py2p3z2m19;
		dN(6, 0) = _18xm9t3x2py2pz2m19 * _1pyt1pz;
		dN(6, 1) = _1mxt1pz * _18yp9tx2p3y2pz2m19;
		dN(6, 2) = _1mxt1py * _18zp9tx2py2p3z2m19;
		dN(7, 0) = _18xp9t3x2py2pz2m19 * _1pyt1pz;
		dN(7, 1) = _1pxt1pz * _18yp9tx2p3y2pz2m19;
		dN(7, 2) = _1pxt1py * _18zp9tx2py2p3z2m19;

		dN.topRowsDivide(8, 64.0);

		cbtScalar _m3m9x2m2x = -_3m9x2 - _2x;
		cbtScalar _p3m9x2m2x = _3m9x2 - _2x;
		cbtScalar _1mx2t1m3x = _1mx2 * _1m3x;
		cbtScalar _1mx2t1p3x = _1mx2 * _1p3x;
		dN(8, 0) = _m3m9x2m2x * _1myt1mz,
			  dN(8, 1) = -_1mx2t1m3x * _1mz,
			  dN(8, 2) = -_1mx2t1m3x * _1my;
		dN(9, 0) = _p3m9x2m2x * _1myt1mz,
			  dN(9, 1) = -_1mx2t1p3x * _1mz,
			  dN(9, 2) = -_1mx2t1p3x * _1my;
		dN(10, 0) = _m3m9x2m2x * _1myt1pz,
			   dN(10, 1) = -_1mx2t1m3x * _1pz,
			   dN(10, 2) = _1mx2t1m3x * _1my;
		dN(11, 0) = _p3m9x2m2x * _1myt1pz,
			   dN(11, 1) = -_1mx2t1p3x * _1pz,
			   dN(11, 2) = _1mx2t1p3x * _1my;
		dN(12, 0) = _m3m9x2m2x * _1pyt1mz,
			   dN(12, 1) = _1mx2t1m3x * _1mz,
			   dN(12, 2) = -_1mx2t1m3x * _1py;
		dN(13, 0) = _p3m9x2m2x * _1pyt1mz,
			   dN(13, 1) = _1mx2t1p3x * _1mz,
			   dN(13, 2) = -_1mx2t1p3x * _1py;
		dN(14, 0) = _m3m9x2m2x * _1pyt1pz,
			   dN(14, 1) = _1mx2t1m3x * _1pz,
			   dN(14, 2) = _1mx2t1m3x * _1py;
		dN(15, 0) = _p3m9x2m2x * _1pyt1pz,
			   dN(15, 1) = _1mx2t1p3x * _1pz,
			   dN(15, 2) = _1mx2t1p3x * _1py;

		cbtScalar _m3m9y2m2y = -_3m9y2 - _2y;
		cbtScalar _p3m9y2m2y = _3m9y2 - _2y;
		cbtScalar _1my2t1m3y = _1my2 * _1m3y;
		cbtScalar _1my2t1p3y = _1my2 * _1p3y;
		dN(16, 0) = -_1my2t1m3y * _1mz,
			   dN(16, 1) = _m3m9y2m2y * _1mxt1mz,
			   dN(16, 2) = -_1my2t1m3y * _1mx;
		dN(17, 0) = -_1my2t1p3y * _1mz,
			   dN(17, 1) = _p3m9y2m2y * _1mxt1mz,
			   dN(17, 2) = -_1my2t1p3y * _1mx;
		dN(18, 0) = _1my2t1m3y * _1mz,
			   dN(18, 1) = _m3m9y2m2y * _1pxt1mz,
			   dN(18, 2) = -_1my2t1m3y * _1px;
		dN(19, 0) = _1my2t1p3y * _1mz,
			   dN(19, 1) = _p3m9y2m2y * _1pxt1mz,
			   dN(19, 2) = -_1my2t1p3y * _1px;
		dN(20, 0) = -_1my2t1m3y * _1pz,
			   dN(20, 1) = _m3m9y2m2y * _1mxt1pz,
			   dN(20, 2) = _1my2t1m3y * _1mx;
		dN(21, 0) = -_1my2t1p3y * _1pz,
			   dN(21, 1) = _p3m9y2m2y * _1mxt1pz,
			   dN(21, 2) = _1my2t1p3y * _1mx;
		dN(22, 0) = _1my2t1m3y * _1pz,
			   dN(22, 1) = _m3m9y2m2y * _1pxt1pz,
			   dN(22, 2) = _1my2t1m3y * _1px;
		dN(23, 0) = _1my2t1p3y * _1pz,
			   dN(23, 1) = _p3m9y2m2y * _1pxt1pz,
			   dN(23, 2) = _1my2t1p3y * _1px;

		cbtScalar _m3m9z2m2z = -_3m9z2 - _2z;
		cbtScalar _p3m9z2m2z = _3m9z2 - _2z;
		cbtScalar _1mz2t1m3z = _1mz2 * _1m3z;
		cbtScalar _1mz2t1p3z = _1mz2 * _1p3z;
		dN(24, 0) = -_1mz2t1m3z * _1my,
			   dN(24, 1) = -_1mz2t1m3z * _1mx,
			   dN(24, 2) = _m3m9z2m2z * _1mxt1my;
		dN(25, 0) = -_1mz2t1p3z * _1my,
			   dN(25, 1) = -_1mz2t1p3z * _1mx,
			   dN(25, 2) = _p3m9z2m2z * _1mxt1my;
		dN(26, 0) = -_1mz2t1m3z * _1py,
			   dN(26, 1) = _1mz2t1m3z * _1mx,
			   dN(26, 2) = _m3m9z2m2z * _1mxt1py;
		dN(27, 0) = -_1mz2t1p3z * _1py,
			   dN(27, 1) = _1mz2t1p3z * _1mx,
			   dN(27, 2) = _p3m9z2m2z * _1mxt1py;
		dN(28, 0) = _1mz2t1m3z * _1my,
			   dN(28, 1) = -_1mz2t1m3z * _1px,
			   dN(28, 2) = _m3m9z2m2z * _1pxt1my;
		dN(29, 0) = _1mz2t1p3z * _1my,
			   dN(29, 1) = -_1mz2t1p3z * _1px,
			   dN(29, 2) = _p3m9z2m2z * _1pxt1my;
		dN(30, 0) = _1mz2t1m3z * _1py,
			   dN(30, 1) = _1mz2t1m3z * _1px,
			   dN(30, 2) = _m3m9z2m2z * _1pxt1py;
		dN(31, 0) = _1mz2t1p3z * _1py,
			   dN(31, 1) = _1mz2t1p3z * _1px,
			   dN(31, 2) = _p3m9z2m2z * _1pxt1py;

		dN.bottomRowsMul(32u - 8u, 9.0 / 64.0);
	}

	return res;
}

bool cbtMiniSDF::interpolate(unsigned int field_id, double& dist, cbtVector3 const& x,
							cbtVector3* gradient) const
{
	cbtAssert(m_isValid);
	if (!m_isValid)
		return false;

	if (!m_domain.contains(x))
		return false;

	cbtVector3 tmpmi = ((x - m_domain.min()) * (m_inv_cell_size));  //.cast<unsigned int>().eval();
	unsigned int mi[3] = {(unsigned int)tmpmi[0], (unsigned int)tmpmi[1], (unsigned int)tmpmi[2]};
	if (mi[0] >= m_resolution[0])
		mi[0] = m_resolution[0] - 1;
	if (mi[1] >= m_resolution[1])
		mi[1] = m_resolution[1] - 1;
	if (mi[2] >= m_resolution[2])
		mi[2] = m_resolution[2] - 1;
	cbtMultiIndex mui;
	mui.ijk[0] = mi[0];
	mui.ijk[1] = mi[1];
	mui.ijk[2] = mi[2];
	int i = multiToSingleIndex(mui);
	unsigned int i_ = m_cell_map[field_id][i];
	if (i_ == UINT_MAX)
		return false;

	cbtAlignedBox3d sd = subdomain(i);
	i = i_;
	cbtVector3 d = sd.m_max - sd.m_min;  //.diagonal().eval();

	cbtVector3 denom = (sd.max() - sd.min());
	cbtVector3 c0 = cbtVector3(2.0, 2.0, 2.0) / denom;
	cbtVector3 c1 = (sd.max() + sd.min()) / denom;
	cbtVector3 xi = (c0 * x - c1);

	cbtCell32 const& cell = m_cells[field_id][i];
	if (!gradient)
	{
		//auto phi = m_coefficients[field_id][i].dot(shape_function_(xi, 0));
		double phi = 0.0;
		cbtShapeMatrix N = shape_function_(xi, 0);
		for (unsigned int j = 0u; j < 32u; ++j)
		{
			unsigned int v = cell.m_cells[j];
			double c = m_nodes[field_id][v];
			if (c == DBL_MAX)
			{
				return false;
				;
			}
			phi += c * N[j];
		}

		dist = phi;
		return true;
	}

	cbtShapeGradients dN;
	cbtShapeMatrix N = shape_function_(xi, &dN);

	double phi = 0.0;
	gradient->setZero();
	for (unsigned int j = 0u; j < 32u; ++j)
	{
		unsigned int v = cell.m_cells[j];
		double c = m_nodes[field_id][v];
		if (c == DBL_MAX)
		{
			gradient->setZero();
			return false;
		}
		phi += c * N[j];
		(*gradient)[0] += c * dN(j, 0);
		(*gradient)[1] += c * dN(j, 1);
		(*gradient)[2] += c * dN(j, 2);
	}
	(*gradient) *= c0;
	dist = phi;
	return true;
}
