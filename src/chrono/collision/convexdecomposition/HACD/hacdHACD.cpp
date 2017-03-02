/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define _CRT_SECURE_NO_WARNINGS
#include <sstream>
#include <hacdGraph.h>
#include <hacdHACD.h>
#include <hacdICHull.h>
#include <string.h>
#include <algorithm>
#include <iterator>
#include <limits>
#include <hacdMeshDecimator.h>
#include <hacdRaycastMesh.h>

#if defined _WIN32
#define SIZET_FMT "%zu"
#else
#define SIZET_FMT "%lu"
#endif

//#define THREAD_DIST_POINTS 1

//#define HACD_DEBUG
namespace HACD
{ 

	double  HACD::Concavity(ICHUll & ch, std::map<long, DPoint> & distPoints)
    {
		double concavity = 0.0;
		double distance = 0.0;       
		std::map<long, DPoint>::iterator itDP(distPoints.begin());
		std::map<long, DPoint>::iterator itDPEnd(distPoints.end());
		long pt;
		const long nPoints = static_cast<long>(m_nPoints);
		const double eps = -0.001;
		for(; itDP != itDPEnd; ++itDP) 
		{
            if (!(itDP->second).m_computed)
            {
                if (itDP->first >= nPoints)
                {
					pt = itDP->first - nPoints;
					if ( ch.IsInside(m_extraDistPoints[pt], eps))
					{
						distance = ch.ComputeDistance(itDP->first, m_extraDistPoints[pt], m_extraDistNormals[pt], (itDP->second).m_computed, true);
					}
					else
					{
						distance = 0.0;
					}
                }
				else if (itDP->first >= 0)
                {
					pt = itDP->first;
					distance = ch.ComputeDistance(itDP->first, m_points[pt], m_normals[pt], (itDP->second).m_computed, true);
                }
                else
                {
					pt = -itDP->first-1;
					distance = ch.ComputeDistance(itDP->first, m_facePoints[pt], m_faceNormals[pt], (itDP->second).m_computed, true);
				}
            }
            else
            {
                distance = (itDP->second).m_dist;
            }

			if (concavity < distance) 
			{
				concavity = distance;
			}
		}
		return concavity;
    }

	void HACD::CreateGraph()
    {
		// vertex to triangle adjacency information
		std::vector< std::set<long> >  vertexToTriangles;		
		vertexToTriangles.resize(m_nPoints);
		for(size_t t = 0; t < m_nTriangles; ++t)
		{
			vertexToTriangles[m_triangles[t].X()].insert(static_cast<long>(t));
			vertexToTriangles[m_triangles[t].Y()].insert(static_cast<long>(t));
			vertexToTriangles[m_triangles[t].Z()].insert(static_cast<long>(t));
		}

		m_graph.Clear();
		m_graph.Allocate(m_nTriangles, 5 * m_nTriangles);
		unsigned long long tr1[3];
		unsigned long long tr2[3];
        long i1, j1, k1, i2, j2, k2;
        long t1, t2;
        for (size_t v = 0; v < m_nPoints; v++) 
		{
			std::set<long>::const_iterator it1(vertexToTriangles[v].begin()), itEnd(vertexToTriangles[v].end()); 
			for(; it1 != itEnd; ++it1)
			{
                t1 = *it1;
                i1 = m_triangles[t1].X();
                j1 = m_triangles[t1].Y();
                k1 = m_triangles[t1].Z();
				tr1[0] = GetEdgeIndex(i1, j1);
				tr1[1] = GetEdgeIndex(j1, k1);
				tr1[2] = GetEdgeIndex(k1, i1);
				std::set<long>::const_iterator it2(it1);
				for(++it2; it2 != itEnd; ++it2)
				{
                    t2 = *it2;
                    i2 = m_triangles[t2].X();
                    j2 = m_triangles[t2].Y();
                    k2 = m_triangles[t2].Z();
					tr2[0] = GetEdgeIndex(i2, j2);
					tr2[1] = GetEdgeIndex(j2, k2);
					tr2[2] = GetEdgeIndex(k2, i2);
					int shared = 0;
					for(int i = 0; i < 3; ++i)
					{
						for(int j = 0; j < 3; ++j)
						{
							if (tr1[i] == tr2[j])	
							{
								shared++;
							}
						}
					}
					if (shared == 1) // two triangles are connected if they share exactly one edge
					{
						m_graph.AddEdge(t1, t2);
					}
				}
			}
        }
        if (m_ccConnectDist >= 0.0)
        {
            m_graph.ExtractCCs();
            if (m_callBack)
            {
                char msg[1024];
                sprintf(msg, "nCC " SIZET_FMT "\n", m_graph.m_nCCs);
                (*m_callBack)(msg, 0.0, 0.0,  m_graph.GetNVertices());
                
            }
            
            if (m_graph.m_nCCs > 1) 
            {
                std::vector< std::set<long> > cc2V;
                cc2V.resize(m_graph.m_nCCs);
                long cc;
                for(size_t t = 0; t < m_nTriangles; ++t)
                {
                    cc = m_graph.m_vertices[t].m_cc;
                    cc2V[cc].insert(m_triangles[t].X());
                    cc2V[cc].insert(m_triangles[t].Y());
                    cc2V[cc].insert(m_triangles[t].Z());
                }
                
                for(size_t cc1 = 0; cc1 < m_graph.m_nCCs; ++cc1)
                {
                    for(size_t cc2 = cc1+1; cc2 < m_graph.m_nCCs; ++cc2)
                    {
                        std::set<long>::const_iterator itV1(cc2V[cc1].begin()), itVEnd1(cc2V[cc1].end()); 
                        for(; itV1 != itVEnd1; ++itV1)
                        {
							double distC1C2 = std::numeric_limits<double>::max();
                            double dist;
                            t1 = -1;
                            t2 = -1;
                            std::set<long>::const_iterator itV2(cc2V[cc2].begin()), itVEnd2(cc2V[cc2].end()); 
                            for(; itV2 != itVEnd2; ++itV2)
                            {
                                dist = (m_points[*itV1] - m_points[*itV2]).GetNorm();
                                if (dist < distC1C2)
                                {
                                    distC1C2 = dist;
                                    t1 = *vertexToTriangles[*itV1].begin();
                                    
									std::set<long>::const_iterator it2(vertexToTriangles[*itV2].begin()), 
																   it2End(vertexToTriangles[*itV2].end()); 
									t2 = -1;
									for(; it2 != it2End; ++it2)
									{
										if (*it2 != t1)
										{
											t2 = *it2;
											break;
										}
									}
                                }
                            }
                            if (distC1C2 <= m_ccConnectDist && t1 >= 0 && t2 >= 0)
                            {
                                m_graph.AddEdge(t1, t2);                    
                            }
                        }
                    }
                }
            }
        }
    }
    void HACD::InitializeDualGraph()
    {
        long i, j, k;
        Vec3<Real> u, v, w, normal;
		delete [] m_normals;
		m_normals = new Vec3<Real>[m_nPoints];
        if (m_addFacesPoints)
        {
            delete [] m_facePoints;
            delete [] m_faceNormals;
            
			m_facePoints = new Vec3<Real>[m_nTriangles];
            m_faceNormals = new Vec3<Real>[m_nTriangles];
/*
            m_facePoints = new Vec3<Real>[4*m_nTriangles];
            m_faceNormals = new Vec3<Real>[4*m_nTriangles];
*/
        }
		memset(m_normals, 0, sizeof(Vec3<Real>) * m_nPoints);

        RaycastMesh rm;
        if (m_addExtraDistPoints)
        {
            rm.Initialize(m_nPoints, m_nTriangles, m_points, m_triangles, 15);
            m_extraDistPoints = new Vec3<Real>[m_nTriangles];
            m_extraDistNormals = new Vec3<Real>[m_nTriangles];
        }
        double progressOld = -1.0;
        double progress = 0.0;
		char msg[1024];
		double ptgStep = 1.0;
		m_area = 0.0;				
		for(unsigned long f = 0; f < m_nTriangles; f++)
        {
            progress = f * 100.0 / m_nTriangles;
            if (fabs(progress-progressOld) > ptgStep && m_callBack)
            {
				sprintf(msg, "%3.2f %% \t \t \r", progress);
				(*m_callBack)(msg, progress, 0.0,  m_nTriangles);
                progressOld = progress;
            }

			i = m_triangles[f].X();
            j = m_triangles[f].Y();
            k = m_triangles[f].Z();
			m_graph.m_vertices[f].m_distPoints.PushBack(DPoint(i, 0, false, false));
			m_graph.m_vertices[f].m_distPoints.PushBack(DPoint(j, 0, false, false));
			m_graph.m_vertices[f].m_distPoints.PushBack(DPoint(k, 0, false, false));
            
            ICHUll  * ch = new ICHUll(m_heapManager);
            m_graph.m_vertices[f].m_convexHull = ch;
            ch->AddPoint(m_points[i], i);
            ch->AddPoint(m_points[j], j);
            ch->AddPoint(m_points[k], k);
			ch->SetDistPoints(0);

			u = m_points[j] - m_points[i];
			v = m_points[k] - m_points[i];
			w = m_points[k] - m_points[j];
			normal = u ^ v;

			m_normals[i] += normal;
			m_normals[j] += normal;
			m_normals[k] += normal;

			m_graph.m_vertices[f].m_surf = normal.GetNorm();
			m_area += m_graph.m_vertices[f].m_surf;
            normal.Normalize();
			m_graph.m_vertices[f].m_boudaryEdges.Insert(GetEdgeIndex(i,j));
			m_graph.m_vertices[f].m_boudaryEdges.Insert(GetEdgeIndex(j,k));
			m_graph.m_vertices[f].m_boudaryEdges.Insert(GetEdgeIndex(k,i));
            if(m_addFacesPoints)
            {
                m_faceNormals[f] = normal;
                m_facePoints[f] = (m_points[i] + m_points[j] + m_points[k]) / 3.0;
				m_graph.m_vertices[f].m_distPoints.PushBack(DPoint(-static_cast<long>(f)-1, 0, false, true));
            }
        }

		// Now we have all the points in the KD tree, optimize the distance points insertion by running them in parallel
		// if possible. 
		if (m_addExtraDistPoints)	
        {
			if (m_callBack)
				(*m_callBack)("++ Also adding distance points\n", 0.0, 0.0, 0);

	        progressOld = -1.0;
			progress = 0.0;
			long completed = 0;

#ifdef THREAD_DIST_POINTS
#pragma omp parallel for
#endif
			for(long f = 0; f < (long)m_nTriangles; f++)
			{
				Vec3<Real> seedPoint((m_points[i] + m_points[j] + m_points[k]) / 3.0);
				Vec3<Real> hitPoint;
				Vec3<Real> hitNormal;
				normal = -normal;
                size_t faceIndex = m_nTriangles;

				Float dist;
				long hitTriangle;
				if (rm.Raycast(seedPoint,normal,hitTriangle,dist, hitPoint, hitNormal))
				{
					faceIndex = hitTriangle;
				}

                if (faceIndex < m_nTriangles )
                {
					m_extraDistPoints[f] = hitPoint;
					m_extraDistNormals[f] = hitNormal;
					m_graph.m_vertices[f].m_distPoints.PushBack(DPoint((long)(m_nPoints+f), 0, false, true));
				}

				// Atomic update of the progress
				#ifdef THREAD_DIST_POINTS
				#pragma omp critical
				#endif
				{
					completed++;
				
					progress = completed * 100.0 / m_nTriangles;
					if (fabs(progress-progressOld) > ptgStep && m_callBack)
					{
						sprintf(msg, "%3.2f %% \t \t \r", progress);
						(*m_callBack)(msg, progress, 0.0,  m_nTriangles);
						progressOld = progress;
					}
				}        
			}
		}

        for (size_t v = 0; v < m_nPoints; v++) 
		{
			m_normals[v].Normalize();
		}
    }

	void HACD::NormalizeData()
	{
		if (m_nPoints == 0)
		{
			return;
		}
        m_barycenter = m_points[0];
		Vec3<Real> min = m_points[0];
		Vec3<Real> max = m_points[0];
		Real x, y, z;
        for (size_t v = 1; v < m_nPoints ; v++) 
        {
			m_barycenter += m_points[v];
            x = m_points[v].X();
            y = m_points[v].Y();
            z = m_points[v].Z();
            if ( x < min.X()) min.X() = x;
			else if ( x > max.X()) max.X() = x;
            if ( y < min.Y()) min.Y() = y;
			else if ( y > max.Y()) max.Y() = y;
            if ( z < min.Z()) min.Z() = z;
			else if ( z > max.Z()) max.Z() = z;
        }
		m_barycenter /= static_cast<Real>(m_nPoints);
        m_diag = (max-min).GetNorm();
        const Real invDiag = static_cast<Real>(2.0 * m_scale / m_diag);
		if (m_diag != 0.0)
		{
			for (size_t v = 0; v < m_nPoints ; v++) 
			{
				m_points[v] = (m_points[v] - m_barycenter) * invDiag;
			}
		}
    }
	void HACD::DenormalizeData()
	{
		if (m_nPoints == 0)
		{
			return;
		}
		if (m_diag != 0.0)
		{
			const Real diag = static_cast<Real>(m_diag / (2.0 * m_scale));
			for (size_t v = 0; v < m_nPoints ; v++) 
			{
				m_points[v] = m_points[v] * diag + m_barycenter;
			}
		}
    }
	HACD::HACD(HeapManager * heapManager):m_heapManager(heapManager)
	{
		m_extraDistPoints = 0;
		m_extraDistNormals = 0;;
        m_convexHulls = 0;
		m_trianglesDecimated = 0;
        m_pointsDecimated = 0;
        m_nTrianglesDecimated = 0;
        m_nPointsDecimated = 0;
		m_triangles = 0;
        m_points = 0;
        m_normals = 0;
        m_nTriangles = 0;
        m_nPoints = 0;
        m_nClusters = 0;
        m_concavity = 0.0;
        m_diag = 1.0;
		m_barycenter = Vec3<Real>(0.0, 0.0,0.0);
        m_alpha = 0.0;
        m_beta = 0.0;
		m_gamma = 0.01;
        m_nVerticesPerCH = 30;
		m_callBack = 0;
        m_addExtraDistPoints = false;
		m_scale = 1000.0;
		m_partition = 0;
		m_nMinClusters = 3;
        m_facePoints = 0;
        m_faceNormals = 0;
        m_ccConnectDist = 30;
		m_targetNTrianglesDecimatedMesh = 1000;
		m_flatRegionThreshold = 1.0;
		m_smallClusterThreshold = 0.25;
		m_area = 0.0;					
	}																
	HACD::~HACD(void)
	{
		delete [] m_normals;
        delete [] m_convexHulls;
		delete [] m_partition;
        delete [] m_facePoints;
        delete [] m_faceNormals;
        delete [] m_trianglesDecimated;
        delete [] m_pointsDecimated;
        delete [] m_extraDistPoints;
        delete [] m_extraDistNormals;
	}

    void HACD::ComputeEdgeCost(size_t e)
    {
		GraphEdge & gE = m_graph.m_edges[e];
        long v1 = gE.m_v1;
        long v2 = gE.m_v2;

        if (m_graph.m_vertices[v2].m_ancestors.size()>m_graph.m_vertices[v1].m_ancestors.size())
        {
            gE.m_v1 = v2;
            gE.m_v2 = v1;
			std::swap(v1, v2);
        }
		GraphVertex & gV1 = m_graph.m_vertices[v1];
		GraphVertex & gV2 = m_graph.m_vertices[v2];
#ifdef HACD_DEBUG
		if (v1 == 308 && v2==276)
		{
			gV1.m_convexHull->m_mesh.Save("debug1.wrl");
			gV2.m_convexHull->m_mesh.Save("debug2.wrl");
		}

#endif
	
        // create the edge's convex-hull
        ICHUll  * ch = new ICHUll(m_heapManager);
        (*ch) = (*gV1.m_convexHull);       
		// update distPoints
#ifdef HACD_PRECOMPUTE_CHULLS
        delete gE.m_convexHull;
        gE.m_convexHull = 0;
#endif
        std::map<long, DPoint> distPoints;
		for(size_t p = 0; p < gV1.m_distPoints.Size(); ++p) 
		{
			distPoints[gV1.m_distPoints[p].m_name] = gV1.m_distPoints[p];
		}

		std::map<long, DPoint>::iterator itDP1;	
		for(size_t p = 0; p < gV2.m_distPoints.Size(); ++p) 
		{
			const DPoint & point =  gV2.m_distPoints[p];
			itDP1 = distPoints.find(point.m_name);
			if (itDP1 == distPoints.end())
			{
				DPoint newPoint(point.m_name, 0, false, point.m_distOnly);
				distPoints.insert(std::pair<long, DPoint>(point.m_name, newPoint));
                if ( !point.m_distOnly )
                {
                    ch->AddPoint(m_points[point.m_name], point.m_name);
                }
			}
            else
            {
                if ( (itDP1->second).m_distOnly && !point.m_distOnly)
                {
                    (itDP1->second).m_distOnly = false;
                    ch->AddPoint(m_points[point.m_name], point.m_name);
                }
            }
		}
		
		ch->SetDistPoints(&distPoints);
        // create the convex-hull
        while (ch->Process() == ICHUllErrorInconsistent)		// if we face problems when constructing the visual-hull. really ugly!!!!
		{
//			if (m_callBack) (*m_callBack)("\t Problem with convex-hull construction [HACD::ComputeEdgeCost]\n", 0.0, 0.0, 0);
            ICHUll  * chOld = ch;
			ch = new ICHUll(m_heapManager);
			CircularList<TMMVertex> & verticesCH = chOld->GetMesh().m_vertices;
			size_t nV = verticesCH.GetSize();
			long ptIndex = 0;
			verticesCH.Next();
			// add noise to avoid the problem
			ptIndex = verticesCH.GetHead()->GetData().m_name;			
			ch->AddPoint(m_points[ptIndex]+ m_scale * 0.0001 * Vec3<Real>(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5), ptIndex);
			for(size_t v = 1; v < nV; ++v)
			{
				ptIndex = verticesCH.GetHead()->GetData().m_name;			
				ch->AddPoint(m_points[ptIndex], ptIndex);
				verticesCH.Next();
			}
			delete chOld;
		}
#ifdef HACD_DEBUG
		if (v1 == 438 && v2==468)
		{
			const long nPoints = static_cast<long>(m_nPoints);
            std::map<long, DPoint>::iterator itDP(distPoints.begin());
            std::map<long, DPoint>::iterator itDPEnd(distPoints.end());
            for(; itDP != itDPEnd; ++itDP) 
            {	

				if (itDP->first >= nPoints)
                {
					long pt = itDP->first - nPoints;
					ch->AddPoint(m_extraDistPoints[pt], itDP->first);
				}
				else if (itDP->first >= 0)
                {
					long pt = itDP->first;
					ch->AddPoint(m_points[pt], itDP->first);
                }
                else
                {
					long pt = -itDP->first-1;
					ch->AddPoint(m_facePoints[pt], itDP->first);
					ch->AddPoint(m_facePoints[pt] + 10.0 * m_faceNormals[pt] , itDP->first);
				}
			}
			printf("-***->\n");

			ch->m_mesh.Save("debug.wrl");
		}
#endif
		double surf = gV1.m_surf + gV2.m_surf;
        double concavity = 0.0;	
		double surfCH = ch->ComputeArea() / 2.0;
		double volumeCH = ch->ComputeVolume();
		double vol2Surf = volumeCH / surfCH;
		double concavity_flat = sqrt(fabs(surfCH-surf));
		double weightFlat = std::max(0.0, 1.0 - pow(- vol2Surf * 100.0 / (m_scale * m_flatRegionThreshold), 2.0));

//		concavity_flat *= std::max(exp(- vol2Surf * 100.0 / (m_scale * m_flatRegionThreshold)) - exp(-1.0), 0.0);
		concavity_flat *= weightFlat;
		if(!ch->IsFlat())
        {
            concavity = Concavity(*ch, distPoints);
        }
		concavity += concavity_flat;
#ifdef HACD_PRECOMPUTE_CHULLS
        gE.m_convexHull = ch;
#else
        delete ch;
#endif

		// compute boudary edges
		double perimeter = 0.0;
		if (m_alpha > 0.0)
		{
            std::set<unsigned long long> boudaryEdges1;
            for(size_t edV1 = 0; edV1 < gV1.m_boudaryEdges.Size(); ++edV1) 
            {
                boudaryEdges1.insert(gV1.m_boudaryEdges[edV1]);
            }
            std::set<unsigned long long> boudaryEdges2;
            for(size_t edV2 = 0; edV2 < gV2.m_boudaryEdges.Size(); ++edV2) 
            {
                boudaryEdges2.insert(gV2.m_boudaryEdges[edV2]);
            }                      
			std::set<unsigned long long> boudaryEdges;
			std::set_symmetric_difference (boudaryEdges1.begin(), 
								           boudaryEdges1.end(), 
								           boudaryEdges2.begin(), 
								           boudaryEdges2.end(),
								           std::inserter( boudaryEdges, boudaryEdges.begin() ) );

			std::set<unsigned long long>::const_iterator itBE(boudaryEdges.begin());
			std::set<unsigned long long>::const_iterator itBEEnd(boudaryEdges.end());
			for(; itBE != itBEEnd; ++itBE)
			{
					perimeter += (m_points[static_cast<long>((*itBE) >> 32)] - 
								   m_points[static_cast<long>((*itBE) & 0xFFFFFFFFULL)]).GetNorm();
			}
		}
        double ratio   = perimeter * perimeter / (4.0 * sc_pi * surf);
        gE.m_concavity = concavity;                     // cluster's concavity
		double volume  = volumeCH/pow(m_scale, 3.0);	// cluster's volume
        gE.m_error     = static_cast<Real>(concavity +  m_alpha * (1.0 - weightFlat) * ratio + m_beta * volume + m_gamma * static_cast<double>(distPoints.size()) / m_nPoints);	// cluster's priority
	}
    bool HACD::InitializePriorityQueue()
    {
//		m_pqueue.reserve(m_graph.m_nE + 100);
        for (size_t e=0; e < m_graph.m_nE; ++e) 
        {
            ComputeEdgeCost(static_cast<long>(e));
//			m_pqueue.push(GraphEdgePriorityQueue(static_cast<long>(e), m_graph.m_edges[e].m_error));
        }
		return true;
    }
	void HACD::Simplify()
	{
		double areaThreshold = m_area * m_smallClusterThreshold / 100.0;
		long v1 = -1;
        long v2 = -1;        
        double progressOld = -1.0;
        double progress = 0.0;
        double globalConcavity  = 0.0;     
		char msg[1024];
		double ptgStep = 1.0;
/*        while ( !m_pqueue.empty() ) 
		{

            progress = 100.0-m_graph.GetNVertices() * 100.0 / m_nTriangles;
            if (fabs(progress-progressOld) > ptgStep && m_callBack)
            {
				sprintf(msg, "%3.2f %% V = %lu \t C = %f \t \t \r", progress, static_cast<unsigned long>(m_graph.GetNVertices()), globalConcavity);
				(*m_callBack)(msg, progress, globalConcavity,  m_graph.GetNVertices());
                progressOld = progress;
				if (progress > 99.0)
				{
					ptgStep = 0.01;
				}
				else if (progress > 90.0)
				{
					ptgStep = 0.1;
				}
            }

			GraphEdgePriorityQueue currentEdge(0,0.0);
			bool done = false;
			do
			{
				done = false;
				if (m_pqueue.size() == 0)
				{
                    done = true;
                    break;
				}
                currentEdge = m_pqueue.top();
                m_pqueue.pop();
			}
			while (  m_graph.m_edges[currentEdge.m_name].m_deleted || 
					 m_graph.m_edges[currentEdge.m_name].m_error != currentEdge.m_priority);

			if (!done)
			{
				v1 = m_graph.m_edges[currentEdge.m_name].m_v1;
				v2 = m_graph.m_edges[currentEdge.m_name].m_v2;	
				bool condition1 = (m_graph.m_edges[currentEdge.m_name].m_concavity < m_concavity) && (globalConcavity < m_concavity) && (m_graph.GetNVertices() > m_nMinClusters) && (m_graph.GetNEdges() > 0);
				bool condition2 = (m_graph.m_vertices[v1].m_surf < areaThreshold || m_graph.m_vertices[v2].m_surf < areaThreshold);				
				if (condition1 || condition2)
                {
					if ((!condition1) && m_callBack)
					{
						sprintf(msg, "\n-> %lu\t%f\t%f\t%f\n", m_pqueue.size(), m_graph.m_vertices[v1].m_surf*100.0/m_area, m_graph.m_vertices[v2].m_surf*100.0/m_area, m_graph.m_edges[currentEdge.m_name].m_concavity);
						(*m_callBack)(msg, progress, globalConcavity,  m_graph.GetNVertices());
					}
					globalConcavity = std::max<double>(globalConcavity ,m_graph.m_edges[currentEdge.m_name].m_concavity);
					GraphEdge & gE = m_graph.m_edges[currentEdge.m_name];
					GraphVertex & gV1 = m_graph.m_vertices[v1];
					GraphVertex & gV2 = m_graph.m_vertices[v2];
					// update vertex info
					gV1.m_concavity     = gE.m_concavity;
	#ifdef HACD_PRECOMPUTE_CHULLS
					(*gV1.m_convexHull) = (*gE.m_convexHull);
					(gV1.m_convexHull)->SetDistPoints(0);
					// update distPoints
					std::map<long, DPoint> distPoints;
					for(size_t p = 0; p < gV1.m_distPoints.Size(); ++p) 
					{
						distPoints[gV1.m_distPoints[p].m_name] = gV1.m_distPoints[p];
					}

					std::map<long, DPoint>::iterator itDP1;	
					for(size_t p = 0; p < gV2.m_distPoints.Size(); ++p) 
					{
						const DPoint & point =  gV2.m_distPoints[p];
						itDP1 = distPoints.find(point.m_name);
						if (itDP1 == distPoints.end())
						{
							DPoint newPoint(point.m_name, 0, false, point.m_distOnly);
							distPoints.insert(std::pair<long, DPoint>(point.m_name, newPoint));
						}
						else
						{
							if ( (itDP1->second).m_distOnly && !point.m_distOnly)
							{
								(itDP1->second).m_distOnly = false;
							}
						}
					}
					gV1.m_distPoints.Clear();
					gV1.m_distPoints.Resize(distPoints.size());
					std::map<long, DPoint>::iterator itDP(distPoints.begin());
					std::map<long, DPoint>::iterator itDPEnd(distPoints.end());
					for(; itDP != itDPEnd; ++itDP) 
					{
						const DPoint & point = itDP->second;
						gV1.m_distPoints.PushBack(itDP->second);
					}
	#else
					ICHUll  * ch = gV1.m_convexHull;								  

					ch->SetDistPoints(0);											  
					// update distPoints
					std::map<long, DPoint> distPoints;
					for(size_t p = 0; p < gV1.m_distPoints.Size(); ++p) 
					{
						distPoints[gV1.m_distPoints[p].m_name] = gV1.m_distPoints[p];
					}

					std::map<long, DPoint>::iterator itDP1;	
					for(size_t p = 0; p < gV2.m_distPoints.Size(); ++p) 
					{
						const DPoint & point =  gV2.m_distPoints[p];
						itDP1 = distPoints.find(point.m_name);
						if (itDP1 == distPoints.end())
						{
							DPoint newPoint(point.m_name, 0, false, point.m_distOnly);
							distPoints.insert(std::pair<long, DPoint>(point.m_name, newPoint));
							if ( !point.m_distOnly )
							{
								ch->AddPoint(m_points[point.m_name], point.m_name);
							}
						}
						else
						{
							if ( (itDP1->second).m_distOnly && !point.m_distOnly)
							{
								(itDP1->second).m_distOnly = false;
								ch->AddPoint(m_points[point.m_name], point.m_name);
							}
						}
					}
					gV1.m_distPoints.Clear();
					gV1.m_distPoints.Resize(distPoints.size());
					std::map<long, DPoint>::iterator itDP(distPoints.begin());
					std::map<long, DPoint>::iterator itDPEnd(distPoints.end());
					for(; itDP != itDPEnd; ++itDP) 
					{
						gV1.m_distPoints.PushBack(itDP->second);
					}
					ch->SetDistPoints(0);
					while (ch->Process() == ICHUllErrorInconsistent)		// if we face problems when constructing the visual-hull. really ugly!!!!
					{
			//			if (m_callBack) (*m_callBack)("\t Problem with convex-hull construction [HACD::ComputeEdgeCost]\n", 0.0, 0.0, 0);
						ICHUll  * chOld = ch;
						ch = new ICHUll(m_heapManager);
						CircularList<TMMVertex> & verticesCH = chOld->GetMesh().m_vertices;
						size_t nV = verticesCH.GetSize();
						long ptIndex = 0;
						verticesCH.Next();
						// add noise to avoid the problem
						ptIndex = verticesCH.GetHead()->GetData().m_name;			
						ch->AddPoint(m_points[ptIndex]+ m_scale * 0.0001 * Vec3<Real>(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5), ptIndex);
						for(size_t v = 1; v < nV; ++v)
						{
							ptIndex = verticesCH.GetHead()->GetData().m_name;			
							ch->AddPoint(m_points[ptIndex], ptIndex);
							verticesCH.Next();
						}
						gV1.m_convexHull = ch;
						delete chOld;
					}

	#ifdef HACD_DEBUG
			if (v1 == 90 && v2==98)
			{
				const long nPoints = static_cast<long>(m_nPoints);
				std::map<long, DPoint>::iterator itDP(distPoints.begin());
				std::map<long, DPoint>::iterator itDPEnd(distPoints.end());
				for(; itDP != itDPEnd; ++itDP) 
				{	

					if (itDP->first >= nPoints)
					{
						long pt = itDP->first - nPoints;
						ch->AddPoint(m_extraDistPoints[pt], itDP->first);
					}
					else if (itDP->first >= 0)
					{
						long pt = itDP->first;
						ch->AddPoint(m_points[pt], itDP->first);
					}
					else
					{
						long pt = -itDP->first-1;
						ch->AddPoint(m_facePoints[pt], itDP->first);
						ch->AddPoint(m_facePoints[pt] + 10.0 * m_faceNormals[pt] , itDP->first);
					}
				}
				printf("-***->\n");

				ch->m_mesh.Save("debug.wrl");
			}
	#endif

	#endif
					if (m_alpha > 0.0)
					{
						std::set<unsigned long long> boudaryEdges1;
						for(size_t edV1 = 0; edV1 < gV1.m_boudaryEdges.Size(); ++edV1) 
						{
							boudaryEdges1.insert(gV1.m_boudaryEdges[edV1]);
						}
						std::set<unsigned long long> boudaryEdges2;
						for(size_t edV2 = 0; edV2 < gV2.m_boudaryEdges.Size(); ++edV2) 
						{
							boudaryEdges2.insert(gV2.m_boudaryEdges[edV2]);
						}                      
						std::set<unsigned long long> boudaryEdges;
						std::set_symmetric_difference (boudaryEdges1.begin(), 
													   boudaryEdges1.end(), 
													   boudaryEdges2.begin(), 
													   boudaryEdges2.end(),
													   std::inserter( boudaryEdges, boudaryEdges.begin() ) );
						gV1.m_boudaryEdges.Clear();
						std::set<unsigned long long>::const_iterator itBE(boudaryEdges.begin());
						std::set<unsigned long long>::const_iterator itBEEnd(boudaryEdges.end());
						for(; itBE != itBEEnd; ++itBE)
						{
							gV1.m_boudaryEdges.Insert(*itBE);
						}
					}
					gV1.m_surf += gV2.m_surf;

	#ifdef HACD_DEBUG				
					printf("v1 %i v2 %i \n", v1, v2);
	#endif
					m_graph.EdgeCollapse(v1, v2);
					long idEdge;
					for(size_t itE = 0; itE < m_graph.m_vertices[v1].m_edges.Size(); ++itE)
					{
						idEdge = m_graph.m_vertices[v1].m_edges[itE];
						ComputeEdgeCost(idEdge);
						m_pqueue.push(GraphEdgePriorityQueue(idEdge, m_graph.m_edges[idEdge].m_error));
					}
				}
			}
			else
			{
				break;
			}
		}*/
        m_cVertices.clear();
		m_nClusters = m_graph.GetNVertices();
        m_cVertices.reserve(m_nClusters);
		for (size_t p=0, v = 0; v != m_graph.m_vertices.size(); ++v) 
		{
			if (!m_graph.m_vertices[v].m_deleted)
			{
                if (m_callBack) 
                {
                    char msg[1024];
                    sprintf(msg, "\t CH(" SIZET_FMT ") \t %lu \t %lf \t " SIZET_FMT " \t %f \t " SIZET_FMT "\n", v, static_cast<unsigned long>(p), m_graph.m_vertices[v].m_concavity, m_graph.m_vertices[v].m_distPoints.Size(),  m_graph.m_vertices[v].m_surf*100.0/m_area, m_graph.m_vertices[v].m_ancestors.size());
					(*m_callBack)(msg, 0.0, 0.0, m_nClusters);
					p++;
                }
                m_cVertices.push_back(static_cast<long>(v));			
			}
		}
        if (m_callBack)
        {
			sprintf(msg, "# clusters =  %lu \t C = %f\n", static_cast<unsigned long>(m_nClusters), globalConcavity);
			(*m_callBack)(msg, progress, globalConcavity,  m_graph.GetNVertices());
        }

	}
        
    bool HACD::Compute(bool fullCH, bool exportDistPoints)
    {
		if ( !m_points || !m_triangles || !m_nPoints || !m_nTriangles)
		{
			return false;
		}

		Vec3<Real> *	pointsOld		= m_points;
		Vec3<long> *	triangles		= m_triangles;
		size_t			nTrianglesOld	= m_nTriangles;
		size_t			PointsOld		= m_nPoints;
        bool decimatedMeshComputed = false;
		if (m_targetNTrianglesDecimatedMesh > 0 && m_targetNTrianglesDecimatedMesh < m_nTriangles)
		{
            decimatedMeshComputed = true;
			MeshDecimator myMDecimator;
			myMDecimator.SetCallBack(m_callBack);
			myMDecimator.Initialize(m_nPoints, m_nTriangles,m_points, m_triangles);
			myMDecimator.Decimate(0, m_targetNTrianglesDecimatedMesh);
			m_nTriangles = myMDecimator.GetNTriangles();
			m_nPoints = myMDecimator.GetNVertices();
			m_points    = new Vec3<Real>[m_nPoints];
			m_triangles = new Vec3<long>[m_nTriangles];
			myMDecimator.GetMeshData(m_points, m_triangles);
		}
		size_t nV = m_nTriangles;

		if (m_callBack)
		{
			std::ostringstream msg;
			msg << "+ Mesh" << std::endl;
			msg << "\t # vertices                     \t" << m_nPoints << std::endl;
			msg << "\t # triangles                    \t" << m_nTriangles << std::endl;
			msg << "+ Parameters" << std::endl;
			msg << "\t min # of clusters              \t" << m_nMinClusters << std::endl;
			msg << "\t max concavity                  \t" << m_concavity << std::endl;
			msg << "\t compacity weigth               \t" << m_alpha << std::endl;
            msg << "\t volume weigth                  \t" << m_beta << std::endl;
			msg << "\t # vertices per convex-hull     \t" << m_nVerticesPerCH << std::endl;
			msg << "\t scale                          \t" << m_scale << std::endl;
			msg << "\t add extra distance points      \t" << m_addExtraDistPoints << std::endl;
            msg << "\t add face distance points       \t" << m_addFacesPoints << std::endl;
			msg << "\t produce full convex-hulls      \t" << fullCH << std::endl;	
			msg << "\t max. distance to connect CCs   \t" << m_ccConnectDist << std::endl;
			msg << "\t threshold for small clusters   \t" << m_smallClusterThreshold << std::endl;
			(*m_callBack)(msg.str().c_str(), 0.0, 0.0, nV);
		}
		if (m_callBack) (*m_callBack)("+ Normalizing Data\n", 0.0, 0.0, nV);
		NormalizeData();
		if (m_callBack) (*m_callBack)("+ Creating Graph\n", 0.0, 0.0, nV);
		CreateGraph();
        // Compute the surfaces and perimeters of all the faces
		if (m_callBack) (*m_callBack)("+ Initializing Dual Graph\n", 0.0, 0.0, nV);
		InitializeDualGraph();
		if (m_callBack) (*m_callBack)("+ Initializing Priority Queue\n", 0.0, 0.0, nV);
        InitializePriorityQueue();
        // we simplify the graph		
		if (m_callBack) (*m_callBack)("+ Simplification ...\n", 0.0, 0.0, m_nTriangles);
		Simplify();
		if (m_callBack) (*m_callBack)("+ Denormalizing Data\n", 0.0, 0.0, m_nClusters);
		DenormalizeData();
		if (m_callBack) (*m_callBack)("+ Computing final convex-hulls\n", 0.0, 0.0, m_nClusters);
        delete [] m_convexHulls;
        m_convexHulls = new ICHUll[m_nClusters];
		delete [] m_partition;
	    m_partition = new long [m_nTriangles];
		for (size_t p = 0; p != m_cVertices.size(); ++p) 
		{
			size_t v = m_cVertices[p];
			m_partition[v] = static_cast<long>(p);
			for(size_t a = 0; a < m_graph.m_vertices[v].m_ancestors.size(); a++)
			{
				m_partition[m_graph.m_vertices[v].m_ancestors[a]] = static_cast<long>(p);
			}
            // compute the convex-hull
            for(size_t itCH = 0; itCH < m_graph.m_vertices[v].m_distPoints.Size(); ++itCH) 
            {
				const DPoint & point = m_graph.m_vertices[v].m_distPoints[itCH];
                if (!point.m_distOnly)
                {
                    m_convexHulls[p].AddPoint(m_points[point.m_name], point.m_name);
                }
            }
			m_convexHulls[p].SetDistPoints(0); //&m_graph.m_vertices[v].m_distPoints
            if (fullCH)
            {
				while (m_convexHulls[p].Process() == ICHUllErrorInconsistent)		// if we face problems when constructing the visual-hull. really ugly!!!!
				{
					ICHUll * ch = new ICHUll(m_heapManager);
					CircularList<TMMVertex> & verticesCH = m_convexHulls[p].GetMesh().m_vertices;
					size_t nV = verticesCH.GetSize();
					long ptIndex = 0;
					verticesCH.Next();
					// add noise to avoid the problem
					ptIndex = verticesCH.GetHead()->GetData().m_name;			
					ch->AddPoint(m_points[ptIndex]+ m_diag * 0.0001 * Vec3<Real>(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5), ptIndex);
					for(size_t v = 1; v < nV; ++v)
					{
						ptIndex = verticesCH.GetHead()->GetData().m_name;			
						ch->AddPoint(m_points[ptIndex], ptIndex);
						verticesCH.Next();
					}
					m_convexHulls[p] = (*ch);
					delete ch;
				}
            }
            else
            {
				while ( m_convexHulls[p].Process(static_cast<unsigned long>(m_nVerticesPerCH)) == ICHUllErrorInconsistent)		// if we face problems when constructing the visual-hull. really ugly!!!!
				{
					ICHUll * ch = new ICHUll(m_heapManager);
					CircularList<TMMVertex> & verticesCH = m_convexHulls[p].GetMesh().m_vertices;
					size_t nV = verticesCH.GetSize();
					long ptIndex = 0;
					verticesCH.Next();
					// add noise to avoid the problem
					ptIndex = verticesCH.GetHead()->GetData().m_name;			
					ch->AddPoint(m_points[ptIndex]+ m_diag * 0.0001 * Vec3<Real>(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5), ptIndex);
					for(size_t v = 1; v < nV; ++v)
					{
						ptIndex = verticesCH.GetHead()->GetData().m_name;			
						ch->AddPoint(m_points[ptIndex], ptIndex);
						verticesCH.Next();
					}
					m_convexHulls[p] = (*ch);
					delete ch;
				}
            }
#ifdef HACD_DEBUG
			if (v==90)
			{
				m_convexHulls[p].m_mesh.Save("debug.wrl");
			}
#endif 
            if (exportDistPoints)
            {
                for(size_t itCH = 0; itCH < m_graph.m_vertices[v].m_distPoints.Size(); ++itCH) 
				{
					const DPoint & point = m_graph.m_vertices[v].m_distPoints[itCH];
                    if (point.m_distOnly)
                    {
                        if (point.m_name >= 0)
                        {
                            m_convexHulls[p].AddPoint(m_points[point.m_name], point.m_name);
                        }
                        else
                        {
                            m_convexHulls[p].AddPoint(m_facePoints[-point.m_name-1], point.m_name);
                        }
                    }
                }
            }
		}
		if (decimatedMeshComputed)
		{
            m_trianglesDecimated  = m_triangles;
            m_pointsDecimated     = m_points;
            m_nTrianglesDecimated = m_nTriangles;
            m_nPointsDecimated    = m_nPoints;
			m_points	 = pointsOld;
			m_triangles	 = triangles;
			m_nTriangles = nTrianglesOld;
			m_nPoints	 = PointsOld;
		}
        return true;
    }
    
    size_t HACD::GetNTrianglesCH(size_t numCH) const
    {
        if (numCH >= m_nClusters)
        {
            return 0;
        }
        return m_convexHulls[numCH].GetMesh().GetNTriangles();
    }
    size_t HACD::GetNPointsCH(size_t numCH) const
    {
        if (numCH >= m_nClusters)
        {
            return 0;
        }
        return m_convexHulls[numCH].GetMesh().GetNVertices();
    }

    bool HACD::GetCH(size_t numCH, Vec3<Real> * const points, Vec3<long> * const triangles)
    {
        if (numCH >= m_nClusters)
        {
            return false;
        }
        m_convexHulls[numCH].GetMesh().GetIFS(points, triangles);
        return true;
    }

    bool HACD::Save(const char * fileName, bool uniColor, long numCluster) const
    {
        std::ofstream fout(fileName);
        if (fout.is_open())
        {
            if (m_callBack)
            {
                char msg[1024];
                sprintf(msg, "Saving %s\n", fileName);
                (*m_callBack)(msg, 0.0, 0.0, m_graph.GetNVertices());
            }
            Material mat;
            if (numCluster < 0)
            {
                for (size_t p = 0; p != m_nClusters; ++p) 
                {
                    if (!uniColor)
                    {
                        mat.m_diffuseColor.X() = mat.m_diffuseColor.Y() = mat.m_diffuseColor.Z() = 0.0;
                        while (mat.m_diffuseColor.X() == mat.m_diffuseColor.Y() ||
                               mat.m_diffuseColor.Z() == mat.m_diffuseColor.Y() ||
                               mat.m_diffuseColor.Z() == mat.m_diffuseColor.X()  )
                        {
                            mat.m_diffuseColor.X() = (rand()%100) / 100.0;
                            mat.m_diffuseColor.Y() = (rand()%100) / 100.0;
                            mat.m_diffuseColor.Z() = (rand()%100) / 100.0;
                        }
                    }
                    m_convexHulls[p].GetMesh().SaveVRML2(fout, mat);
                }
            }
            else if (numCluster < static_cast<long>(m_cVertices.size()))
            {
                m_convexHulls[numCluster].GetMesh().SaveVRML2(fout, mat);
            }
            fout.close();
            return true;
        }
        else
        {
            if (m_callBack)
            {
                char msg[1024];
                sprintf(msg, "Error saving %s\n", fileName);
                (*m_callBack)(msg, 0.0, 0.0, m_graph.GetNVertices());
            }
            return false;
        }
    }
}