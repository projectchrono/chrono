/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <hacdRaycastMesh.h>
#include <math.h>
#include <assert.h>
#include <limits>
#include <algorithm>
#include <hacdManifoldMesh.h>
namespace HACD
{
	bool BBox::Raycast(const Vec3<Float> & origin, const Vec3<Float> & dir, Float & distMin) const
	{
		Vec3<Float> d = m_max - m_min;
		Float r2 = 0.25 * (d.X()*d.X()+d.Y()*d.Y()+d.Z()*d.Z());
		Vec3<Float> c = 0.5 * m_max + 0.5 * m_min;
		Vec3<Float> u = (c-origin);
		Vec3<Float> a = u - (u * dir) * dir;
		Float dist2 = (a.X()*a.X()+a.Y()*a.Y()+a.Z()*a.Z());
		distMin = (u.X()*u.X()+u.Y()*u.Y()+u.Z()*u.Z());
		if ( distMin > r2 ) // origin outside the sphere of center c and radius r 
		{
			distMin = sqrt(distMin) - sqrt(r2); // the distance to any point inside the sphere is higher than |origin - c| - r
		}
		else
		{
			distMin = 0.0;
		}
		if (dist2 > r2) return false;
		return true;
	}	
	void RMNode::ComputeBB()
	{
		if (m_triIDs.Size() == 0)
		{
			return;
		}
		Vec3<long> * const triangles = m_rm->m_triangles;			
		Vec3<Float> * const vertices = m_rm->m_vertices;
		const Float minFloat = std::numeric_limits<Float>::min();
		const Float maxFloat = std::numeric_limits<Float>::max();
		m_bBox.m_max = Vec3<Float>(minFloat, minFloat, minFloat);
		m_bBox.m_min = Vec3<Float>(maxFloat, maxFloat, maxFloat);
		Float x, y, z;
		long v, f;
		for(size_t id = 0; id < m_triIDs.Size(); ++id)
		{
			f = m_triIDs[id];
			for(size_t k = 0; k < 3; ++k)
			{
				v = triangles[f][k];
				x = vertices[v].X();
				y = vertices[v].Y();
				z = vertices[v].Z();
				if ( x < m_bBox.m_min.X()) m_bBox.m_min.X() = x;
				if ( x > m_bBox.m_max.X()) m_bBox.m_max.X() = x;
				if ( y < m_bBox.m_min.Y()) m_bBox.m_min.Y() = y;
				if ( y > m_bBox.m_max.Y()) m_bBox.m_max.Y() = y;
				if ( z < m_bBox.m_min.Z()) m_bBox.m_min.Z() = z;
				if ( z > m_bBox.m_max.Z()) m_bBox.m_max.Z() = z;
			}
        }
	}
	void RaycastMesh::ComputeBB()
	{
		if (m_nVertices == 0)
		{
			return;
		}
		m_bBox.m_min = m_vertices[0];
		m_bBox.m_max = m_vertices[0];
		Float x, y, z;
        for (size_t v = 1; v < m_nVertices ; v++) 
        {
            x = m_vertices[v].X();
            y = m_vertices[v].Y();
            z = m_vertices[v].Z();
            if ( x < m_bBox.m_min.X()) m_bBox.m_min.X() = x;
			else if ( x > m_bBox.m_max.X()) m_bBox.m_max.X() = x;
            if ( y < m_bBox.m_min.Y()) m_bBox.m_min.Y() = y;
			else if ( y > m_bBox.m_max.Y()) m_bBox.m_max.Y() = y;
            if ( z < m_bBox.m_min.Z()) m_bBox.m_min.Z() = z;
			else if ( z > m_bBox.m_max.Z()) m_bBox.m_max.Z() = z;
        }
	}
	void RaycastMesh::Initialize(size_t nVertices, size_t nTriangles, 
								Vec3<Float> *  vertices,  Vec3<long> * triangles, 
								size_t maxDepth, size_t minLeafSize, Float minAxisSize)
	{
		m_triangles  = triangles;			
		m_vertices   = vertices;		
		m_nVertices  = nVertices;
		m_nTriangles = nTriangles;
		delete [] m_nodes;
		m_nNodes = 0;
		m_nMaxNodes = 0;
		for(size_t k = 0; k < maxDepth; k++)
		{
			m_nMaxNodes += ((size_t)1 << maxDepth);
		}
		m_nodes = new RMNode[m_nMaxNodes];
		RMNode & root = m_nodes[AddNode()];
		root.m_triIDs.Resize(nTriangles);
		for(size_t t = 0; t < m_nTriangles; ++t) root.m_triIDs.PushBack((long)t);
		root.m_rm = this;
		root.m_id = 0;
		root.Create(0, maxDepth, minLeafSize, minAxisSize);
	}
	RaycastMesh::RaycastMesh(void)
	{
		m_triangles  = 0;			
		m_vertices   = 0;		
		m_nVertices  = 0;
		m_nTriangles = 0;
		m_nodes		 = 0;
		m_nNodes	 = 0;
		m_nMaxNodes  = 0;
	}
	RaycastMesh::~RaycastMesh(void)
	{
		delete [] m_nodes;
	}
	void RMNode::Create(size_t depth, size_t maxDepth, size_t minLeafSize, Float minAxisSize)
	{
		ComputeBB();
		Vec3<Float> d = m_bBox.m_max - m_bBox.m_min;
		Float maxDiff = std::max<Float>(d.X(), std::max<Float>(d.Y(), d.Z()));
		RMSplitAxis split;
		if		(d.X() == maxDiff) split = RMSplitAxis_X;
		else if (d.Y() == maxDiff) split = RMSplitAxis_Y;
		else					   split = RMSplitAxis_Z;
	
		if (depth == maxDepth || minLeafSize >= m_triIDs.Size() || maxDiff < minAxisSize)
		{
			m_leaf = true;
			return;
		}
		m_idLeft  = (long)m_rm->AddNode();
		m_idRight = (long)m_rm->AddNode();
		RMNode & leftNode  = m_rm->m_nodes[m_idLeft];	
		RMNode & rightNode = m_rm->m_nodes[m_idRight];
		leftNode.m_id      = m_idLeft;
		rightNode.m_id     = m_idRight;
		leftNode.m_bBox    = m_bBox;
		rightNode.m_bBox   = m_bBox;
		leftNode.m_rm	   = m_rm;
		rightNode.m_rm	   = m_rm;

		if ( split == RMSplitAxis_X)
		{
			leftNode.m_bBox.m_max.X() = leftNode.m_bBox.m_max.X() - d.X() * 0.5;
			rightNode.m_bBox.m_min.X() = leftNode.m_bBox.m_max.X();
		}
		else if ( split == RMSplitAxis_Y)
		{
			leftNode.m_bBox.m_max.Y() = leftNode.m_bBox.m_max.Y() - d.Y() * 0.5;
			rightNode.m_bBox.m_min.Y() = leftNode.m_bBox.m_max.Y();
		}
		else
		{
			leftNode.m_bBox.m_max.Z() = leftNode.m_bBox.m_max.Z() - d.Z() * 0.5;
			rightNode.m_bBox.m_min.Z() = leftNode.m_bBox.m_max.Z();
		}
		long f;
		long pts[3];
		long v;
		Vec3<long> * const triangles = m_rm->m_triangles;			
		Vec3<Float> * const vertices = m_rm->m_vertices;
		leftNode.m_triIDs.Resize(m_triIDs.Size());
		rightNode.m_triIDs.Resize(m_triIDs.Size());
		bool found;
		for(size_t id = 0; id < m_triIDs.Size(); ++id)
		{
			f = m_triIDs[id];
			pts[0] = triangles[f].X();
            pts[1] = triangles[f].Y();
            pts[2] = triangles[f].Z();

			found = false;
			for (int k = 0; k < 3; ++k)
			{
				v = pts[k];
				if ( vertices[v].X() <= leftNode.m_bBox.m_max.X() && 
					 vertices[v].X() >= leftNode.m_bBox.m_min.X() &&
					 vertices[v].Y() <= leftNode.m_bBox.m_max.Y() && 
					 vertices[v].Y() >= leftNode.m_bBox.m_min.Y() &&
					 vertices[v].Z() <= leftNode.m_bBox.m_max.Z() && 
					 vertices[v].Z() >= leftNode.m_bBox.m_min.Z() )
				{
					leftNode.m_triIDs.PushBack(f);
					found = true;
					break;
				}
			}
			if (!found)
			{
				for (int k = 0; k < 3; ++k)
				{
					v = pts[k];
					if ( vertices[v].X() <= rightNode.m_bBox.m_max.X() && 
						 vertices[v].X() >= rightNode.m_bBox.m_min.X() &&
						 vertices[v].Y() <= rightNode.m_bBox.m_max.Y() && 
						 vertices[v].Y() >= rightNode.m_bBox.m_min.Y() &&
						 vertices[v].Z() <= rightNode.m_bBox.m_max.Z() && 
						 vertices[v].Z() >= rightNode.m_bBox.m_min.Z() )
					{
						rightNode.m_triIDs.PushBack(f);
						break;
					}
				}		
			}
		}
		rightNode.Create(depth+1, maxDepth, minLeafSize, minAxisSize);
		leftNode.Create(depth+1, maxDepth, minLeafSize, minAxisSize);
		m_triIDs.Clear();
	}
	bool RaycastMesh::Raycast(const Vec3<Float> & from, const Vec3<Float> & dir, long & triID, Float & distance, Vec3<Real> & hitPoint, Vec3<Real> & hitNormal) const
	{
		distance = std::numeric_limits<Float>::max();
		if (m_nNodes == 0) return false;
		return m_nodes[0].Raycast(from, dir, triID, distance, hitPoint, hitNormal);
	}
	bool RMNode::Raycast(const Vec3<Float> & from, const Vec3<Float> & dir, long & triID, Float & distance, Vec3<Real> & hitPoint, Vec3<Real> & hitNormal) const 
	{
		Float distToSphere;
		if (m_bBox.Raycast(from, dir, distToSphere) && (distToSphere < distance))
		{
			if (m_leaf)
			{
				long f, i1, j1, k1;
				Vec3<long> * const triangles = m_rm->m_triangles;			
				Vec3<Float> * const vertices = m_rm->m_vertices;
				Vec3<Real> u1, v1, normal1;
				double dist = 0.0;
				long nhit = 0;
				bool ret = false;
				for(size_t id = 0; id < m_triIDs.Size(); ++id)
				{
					f = m_triIDs[id];
					i1 = triangles[f].X();
					j1 = triangles[f].Y();
					k1 = triangles[f].Z();
					u1 = vertices[j1] - vertices[i1];
					v1 = vertices[k1] - vertices[i1];
					normal1 = (u1 ^ v1);
					if (dir * normal1 > 0.0)
					{
						nhit = IntersectRayTriangle(from, dir, vertices[i1], vertices[j1], vertices[k1], dist);
						if (nhit==1 && distance>dist)
						{
							normal1.Normalize();
							hitNormal = normal1;
							hitPoint = from + dist * dir;
							distance = dist;
							triID = f;
							ret = true;
						}						
					}
				}
				return ret;
			}
			bool ret1 = false;
			bool ret2 = false;
			if (m_idRight >= 0) ret1 = m_rm->m_nodes[m_idRight].Raycast(from, dir, triID, distance, hitPoint, hitNormal);
			if (m_idLeft >= 0)  ret2 = m_rm->m_nodes[m_idLeft].Raycast(from, dir, triID, distance, hitPoint, hitNormal);
			return ret1 || ret2;
		}
		return false;
	}
}







