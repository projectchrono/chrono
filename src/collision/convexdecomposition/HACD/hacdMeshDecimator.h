/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
All rights reserved.


 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#ifndef HACD_MESH_DECEMATOR_H
#define HACD_MESH_DECEMATOR_H
#include <queue>
#include <set>
#include <vector>
#include <limits>
#include <hacdVersion.h>
#include <hacdVector.h>
#include <hacdSArray.h>

namespace HACD
{
	typedef double Float;
	struct MDVertex
	{
		SArray<long, SARRAY_DEFAULT_MIN_SIZE>   m_edges;	
		SArray<long, SARRAY_DEFAULT_MIN_SIZE>	m_triangles;	
		Float									m_Q[10];
												// 0 1 2 3
	  											//   4 5 6
	   											//     7 8
		  										//       9
		bool									m_tag;
		bool									m_onBoundary;
	};
	
	struct MDEdge
	{
		long									m_v1;		
		long									m_v2;		
		double									m_qem;	
		Vec3<Float>								m_pos;
		bool									m_onBoundary;
		bool									m_tag;
	};
	struct MDEdgePriorityQueue
	{
		long									m_name;		
		double									m_qem;	
		inline	friend bool						operator<(const MDEdgePriorityQueue & lhs, const MDEdgePriorityQueue & rhs) { return (lhs.m_qem > rhs.m_qem);}
		inline	friend bool						operator>(const MDEdgePriorityQueue & lhs, const MDEdgePriorityQueue & rhs) { return (lhs.m_qem < rhs.m_qem);}
	};
	typedef void (*CallBackFunction)(const char *, double, double, size_t);
	class MeshDecimator
	{
	public:
		//! Sets the call-back function
		//! @param callBack pointer to the call-back function
		void									SetCallBack(CallBackFunction  callBack) { m_callBack = callBack;}
		//! Gives the call-back function
		//! @return pointer to the call-back function
		const CallBackFunction                  GetCallBack() const { return m_callBack;}

		inline void								SetEColManifoldConstraint(bool ecolManifoldConstraint) { m_ecolManifoldConstraint = ecolManifoldConstraint; }
		inline size_t							GetNVertices()const {return m_nVertices;};
		inline size_t							GetNTriangles() const {return m_nTriangles;};
		inline size_t							GetNEdges() const {return m_nEdges;};
		void									GetMeshData(Vec3<Float> * points, Vec3<long> * triangles) const;
		void									ReleaseMemory();
		void									Initialize(size_t nVertices, size_t nTriangles, 
														   Vec3<Float> *  points, 
														   Vec3<long> * triangles);		
		bool 									Decimate(size_t targetNVertices = 100, 
														 size_t targetNTriangles = 0, 
														 double targetError = std::numeric_limits<double>::max());

												MeshDecimator(void);
												~MeshDecimator(void);
	private : 
		void									EdgeCollapse(long v1, long v2);
		long									GetTriangle(long v1, long v2, long v3) const;
		long									GetEdge(long v1, long v2) const;
		long									IsBoundaryEdge(long v1, long v2) const;
		bool									IsBoundaryVertex(long v) const;
		void									InitializePriorityQueue();
		void									InitializeQEM();
		bool									ManifoldConstraint(long v1, long v2) const;
		double									ComputeEdgeCost(long v1, long v2, Vec3<Float> & pos) const;
		bool									EdgeCollapse(double & error);
	private:
		Vec3<long> *							m_triangles;			
		Vec3<Float> *							m_points;		
		size_t									m_nPoints;
		size_t									m_nInitialTriangles;
		size_t									m_nVertices;
		size_t									m_nTriangles;
		size_t									m_nEdges;
		double									m_diagBB;
		std::vector<MDVertex>					m_vertices;
		std::vector<MDEdge>						m_edges;
		std::priority_queue<
			 MDEdgePriorityQueue, 
			 std::vector<MDEdgePriorityQueue>, 
			 std::less<MDEdgePriorityQueue> >	m_pqueue;
		CallBackFunction						m_callBack;					//>! call-back function
		bool *									m_trianglesTags;
		bool									m_ecolManifoldConstraint;
	};
}
#endif