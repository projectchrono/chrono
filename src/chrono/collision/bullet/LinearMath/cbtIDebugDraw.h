/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_IDEBUG_DRAW__H
#define BT_IDEBUG_DRAW__H

#include "cbtVector3.h"
#include "cbtTransform.h"

///The cbtIDebugDraw interface class allows hooking up a debug renderer to visually debug simulations.
///Typical use case: create a debug drawer object, and assign it to a cbtCollisionWorld or cbtDynamicsWorld using setDebugDrawer and call debugDrawWorld.
///A class that implements the cbtIDebugDraw interface has to implement the drawLine method at a minimum.
///For color arguments the X,Y,Z components refer to Red, Green and Blue each in the range [0..1]
class cbtIDebugDraw
{
public:
	ATTRIBUTE_ALIGNED16(struct)
	DefaultColors
	{
		cbtVector3 m_activeObject;
		cbtVector3 m_deactivatedObject;
		cbtVector3 m_wantsDeactivationObject;
		cbtVector3 m_disabledDeactivationObject;
		cbtVector3 m_disabledSimulationObject;
		cbtVector3 m_aabb;
		cbtVector3 m_contactPoint;

		DefaultColors()
			: m_activeObject(1, 1, 1),
			  m_deactivatedObject(0, 1, 0),
			  m_wantsDeactivationObject(0, 1, 1),
			  m_disabledDeactivationObject(1, 0, 0),
			  m_disabledSimulationObject(1, 1, 0),
			  m_aabb(1, 0, 0),
			  m_contactPoint(1, 1, 0)
		{
		}
	};

	enum DebugDrawModes
	{
		DBG_NoDebug = 0,
		DBG_DrawWireframe = 1,
		DBG_DrawAabb = 2,
		DBG_DrawFeaturesText = 4,
		DBG_DrawContactPoints = 8,
		DBG_NoDeactivation = 16,
		DBG_NoHelpText = 32,
		DBG_DrawText = 64,
		DBG_ProfileTimings = 128,
		DBG_EnableSatComparison = 256,
		DBG_DisableBulletLCP = 512,
		DBG_EnableCCD = 1024,
		DBG_DrawConstraints = (1 << 11),
		DBG_DrawConstraintLimits = (1 << 12),
		DBG_FastWireframe = (1 << 13),
		DBG_DrawNormals = (1 << 14),
		DBG_DrawFrames = (1 << 15),
		DBG_MAX_DEBUG_DRAW_MODE
	};

	virtual ~cbtIDebugDraw(){};

	virtual DefaultColors getDefaultColors() const
	{
		DefaultColors colors;
		return colors;
	}
	///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
	virtual void setDefaultColors(const DefaultColors& /*colors*/) {}

	virtual void drawLine(const cbtVector3& from, const cbtVector3& to, const cbtVector3& color) = 0;

	virtual void drawLine(const cbtVector3& from, const cbtVector3& to, const cbtVector3& fromColor, const cbtVector3& toColor)
	{
		(void)toColor;
		drawLine(from, to, fromColor);
	}

	virtual void drawSphere(cbtScalar radius, const cbtTransform& transform, const cbtVector3& color)
	{
		cbtVector3 center = transform.getOrigin();
		cbtVector3 up = transform.getBasis().getColumn(1);
		cbtVector3 axis = transform.getBasis().getColumn(0);
		cbtScalar minTh = -SIMD_HALF_PI;
		cbtScalar maxTh = SIMD_HALF_PI;
		cbtScalar minPs = -SIMD_HALF_PI;
		cbtScalar maxPs = SIMD_HALF_PI;
		cbtScalar stepDegrees = 30.f;
		drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, stepDegrees, false);
		drawSpherePatch(center, up, -axis, radius, minTh, maxTh, minPs, maxPs, color, stepDegrees, false);
	}

	virtual void drawSphere(const cbtVector3& p, cbtScalar radius, const cbtVector3& color)
	{
		cbtTransform tr;
		tr.setIdentity();
		tr.setOrigin(p);
		drawSphere(radius, tr, color);
	}

	virtual void drawTriangle(const cbtVector3& v0, const cbtVector3& v1, const cbtVector3& v2, const cbtVector3& /*n0*/, const cbtVector3& /*n1*/, const cbtVector3& /*n2*/, const cbtVector3& color, cbtScalar alpha)
	{
		drawTriangle(v0, v1, v2, color, alpha);
	}
	virtual void drawTriangle(const cbtVector3& v0, const cbtVector3& v1, const cbtVector3& v2, const cbtVector3& color, cbtScalar /*alpha*/)
	{
		drawLine(v0, v1, color);
		drawLine(v1, v2, color);
		drawLine(v2, v0, color);
	}

	virtual void drawContactPoint(const cbtVector3& PointOnB, const cbtVector3& normalOnB, cbtScalar distance, int lifeTime, const cbtVector3& color) = 0;

	virtual void reportErrorWarning(const char* warningString) = 0;

	virtual void draw3dText(const cbtVector3& location, const char* textString) = 0;

	virtual void setDebugMode(int debugMode) = 0;

	virtual int getDebugMode() const = 0;

	virtual void drawAabb(const cbtVector3& from, const cbtVector3& to, const cbtVector3& color)
	{
		cbtVector3 halfExtents = (to - from) * 0.5f;
		cbtVector3 center = (to + from) * 0.5f;
		int i, j;

		cbtVector3 edgecoord(1.f, 1.f, 1.f), pa, pb;
		for (i = 0; i < 4; i++)
		{
			for (j = 0; j < 3; j++)
			{
				pa = cbtVector3(edgecoord[0] * halfExtents[0], edgecoord[1] * halfExtents[1],
							   edgecoord[2] * halfExtents[2]);
				pa += center;

				int othercoord = j % 3;
				edgecoord[othercoord] *= -1.f;
				pb = cbtVector3(edgecoord[0] * halfExtents[0], edgecoord[1] * halfExtents[1],
							   edgecoord[2] * halfExtents[2]);
				pb += center;

				drawLine(pa, pb, color);
			}
			edgecoord = cbtVector3(-1.f, -1.f, -1.f);
			if (i < 3)
				edgecoord[i] *= -1.f;
		}
	}
	virtual void drawTransform(const cbtTransform& transform, cbtScalar orthoLen)
	{
		cbtVector3 start = transform.getOrigin();
		drawLine(start, start + transform.getBasis() * cbtVector3(orthoLen, 0, 0), cbtVector3(cbtScalar(1.), cbtScalar(0.3), cbtScalar(0.3)));
		drawLine(start, start + transform.getBasis() * cbtVector3(0, orthoLen, 0), cbtVector3(cbtScalar(0.3), cbtScalar(1.), cbtScalar(0.3)));
		drawLine(start, start + transform.getBasis() * cbtVector3(0, 0, orthoLen), cbtVector3(cbtScalar(0.3), cbtScalar(0.3), cbtScalar(1.)));
	}

	virtual void drawArc(const cbtVector3& center, const cbtVector3& normal, const cbtVector3& axis, cbtScalar radiusA, cbtScalar radiusB, cbtScalar minAngle, cbtScalar maxAngle,
						 const cbtVector3& color, bool drawSect, cbtScalar stepDegrees = cbtScalar(10.f))
	{
		const cbtVector3& vx = axis;
		cbtVector3 vy = normal.cross(axis);
		cbtScalar step = stepDegrees * SIMD_RADS_PER_DEG;
		int nSteps = (int)cbtFabs((maxAngle - minAngle) / step);
		if (!nSteps) nSteps = 1;
		cbtVector3 prev = center + radiusA * vx * cbtCos(minAngle) + radiusB * vy * cbtSin(minAngle);
		if (drawSect)
		{
			drawLine(center, prev, color);
		}
		for (int i = 1; i <= nSteps; i++)
		{
			cbtScalar angle = minAngle + (maxAngle - minAngle) * cbtScalar(i) / cbtScalar(nSteps);
			cbtVector3 next = center + radiusA * vx * cbtCos(angle) + radiusB * vy * cbtSin(angle);
			drawLine(prev, next, color);
			prev = next;
		}
		if (drawSect)
		{
			drawLine(center, prev, color);
		}
	}
	virtual void drawSpherePatch(const cbtVector3& center, const cbtVector3& up, const cbtVector3& axis, cbtScalar radius,
								 cbtScalar minTh, cbtScalar maxTh, cbtScalar minPs, cbtScalar maxPs, const cbtVector3& color, cbtScalar stepDegrees = cbtScalar(10.f), bool drawCenter = true)
	{
		cbtVector3 vA[74];
		cbtVector3 vB[74];
		cbtVector3 *pvA = vA, *pvB = vB, *pT;
		cbtVector3 npole = center + up * radius;
		cbtVector3 spole = center - up * radius;
		cbtVector3 arcStart;
		cbtScalar step = stepDegrees * SIMD_RADS_PER_DEG;
		const cbtVector3& kv = up;
		const cbtVector3& iv = axis;
		cbtVector3 jv = kv.cross(iv);
		bool drawN = false;
		bool drawS = false;
		if (minTh <= -SIMD_HALF_PI)
		{
			minTh = -SIMD_HALF_PI + step;
			drawN = true;
		}
		if (maxTh >= SIMD_HALF_PI)
		{
			maxTh = SIMD_HALF_PI - step;
			drawS = true;
		}
		if (minTh > maxTh)
		{
			minTh = -SIMD_HALF_PI + step;
			maxTh = SIMD_HALF_PI - step;
			drawN = drawS = true;
		}
		int n_hor = (int)((maxTh - minTh) / step) + 1;
		if (n_hor < 2) n_hor = 2;
		cbtScalar step_h = (maxTh - minTh) / cbtScalar(n_hor - 1);
		bool isClosed = false;
		if (minPs > maxPs)
		{
			minPs = -SIMD_PI + step;
			maxPs = SIMD_PI;
			isClosed = true;
		}
		else if ((maxPs - minPs) >= SIMD_PI * cbtScalar(2.f))
		{
			isClosed = true;
		}
		else
		{
			isClosed = false;
		}
		int n_vert = (int)((maxPs - minPs) / step) + 1;
		if (n_vert < 2) n_vert = 2;
		cbtScalar step_v = (maxPs - minPs) / cbtScalar(n_vert - 1);
		for (int i = 0; i < n_hor; i++)
		{
			cbtScalar th = minTh + cbtScalar(i) * step_h;
			cbtScalar sth = radius * cbtSin(th);
			cbtScalar cth = radius * cbtCos(th);
			for (int j = 0; j < n_vert; j++)
			{
				cbtScalar psi = minPs + cbtScalar(j) * step_v;
				cbtScalar sps = cbtSin(psi);
				cbtScalar cps = cbtCos(psi);
				pvB[j] = center + cth * cps * iv + cth * sps * jv + sth * kv;
				if (i)
				{
					drawLine(pvA[j], pvB[j], color);
				}
				else if (drawS)
				{
					drawLine(spole, pvB[j], color);
				}
				if (j)
				{
					drawLine(pvB[j - 1], pvB[j], color);
				}
				else
				{
					arcStart = pvB[j];
				}
				if ((i == (n_hor - 1)) && drawN)
				{
					drawLine(npole, pvB[j], color);
				}

				if (drawCenter)
				{
					if (isClosed)
					{
						if (j == (n_vert - 1))
						{
							drawLine(arcStart, pvB[j], color);
						}
					}
					else
					{
						if (((!i) || (i == (n_hor - 1))) && ((!j) || (j == (n_vert - 1))))
						{
							drawLine(center, pvB[j], color);
						}
					}
				}
			}
			pT = pvA;
			pvA = pvB;
			pvB = pT;
		}
	}

	virtual void drawBox(const cbtVector3& bbMin, const cbtVector3& bbMax, const cbtVector3& color)
	{
		drawLine(cbtVector3(bbMin[0], bbMin[1], bbMin[2]), cbtVector3(bbMax[0], bbMin[1], bbMin[2]), color);
		drawLine(cbtVector3(bbMax[0], bbMin[1], bbMin[2]), cbtVector3(bbMax[0], bbMax[1], bbMin[2]), color);
		drawLine(cbtVector3(bbMax[0], bbMax[1], bbMin[2]), cbtVector3(bbMin[0], bbMax[1], bbMin[2]), color);
		drawLine(cbtVector3(bbMin[0], bbMax[1], bbMin[2]), cbtVector3(bbMin[0], bbMin[1], bbMin[2]), color);
		drawLine(cbtVector3(bbMin[0], bbMin[1], bbMin[2]), cbtVector3(bbMin[0], bbMin[1], bbMax[2]), color);
		drawLine(cbtVector3(bbMax[0], bbMin[1], bbMin[2]), cbtVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(cbtVector3(bbMax[0], bbMax[1], bbMin[2]), cbtVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(cbtVector3(bbMin[0], bbMax[1], bbMin[2]), cbtVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(cbtVector3(bbMin[0], bbMin[1], bbMax[2]), cbtVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(cbtVector3(bbMax[0], bbMin[1], bbMax[2]), cbtVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(cbtVector3(bbMax[0], bbMax[1], bbMax[2]), cbtVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(cbtVector3(bbMin[0], bbMax[1], bbMax[2]), cbtVector3(bbMin[0], bbMin[1], bbMax[2]), color);
	}
	virtual void drawBox(const cbtVector3& bbMin, const cbtVector3& bbMax, const cbtTransform& trans, const cbtVector3& color)
	{
		drawLine(trans * cbtVector3(bbMin[0], bbMin[1], bbMin[2]), trans * cbtVector3(bbMax[0], bbMin[1], bbMin[2]), color);
		drawLine(trans * cbtVector3(bbMax[0], bbMin[1], bbMin[2]), trans * cbtVector3(bbMax[0], bbMax[1], bbMin[2]), color);
		drawLine(trans * cbtVector3(bbMax[0], bbMax[1], bbMin[2]), trans * cbtVector3(bbMin[0], bbMax[1], bbMin[2]), color);
		drawLine(trans * cbtVector3(bbMin[0], bbMax[1], bbMin[2]), trans * cbtVector3(bbMin[0], bbMin[1], bbMin[2]), color);
		drawLine(trans * cbtVector3(bbMin[0], bbMin[1], bbMin[2]), trans * cbtVector3(bbMin[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * cbtVector3(bbMax[0], bbMin[1], bbMin[2]), trans * cbtVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * cbtVector3(bbMax[0], bbMax[1], bbMin[2]), trans * cbtVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * cbtVector3(bbMin[0], bbMax[1], bbMin[2]), trans * cbtVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * cbtVector3(bbMin[0], bbMin[1], bbMax[2]), trans * cbtVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * cbtVector3(bbMax[0], bbMin[1], bbMax[2]), trans * cbtVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * cbtVector3(bbMax[0], bbMax[1], bbMax[2]), trans * cbtVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * cbtVector3(bbMin[0], bbMax[1], bbMax[2]), trans * cbtVector3(bbMin[0], bbMin[1], bbMax[2]), color);
	}

	virtual void drawCapsule(cbtScalar radius, cbtScalar halfHeight, int upAxis, const cbtTransform& transform, const cbtVector3& color)
	{
		int stepDegrees = 30;

		cbtVector3 capStart(0.f, 0.f, 0.f);
		capStart[upAxis] = -halfHeight;

		cbtVector3 capEnd(0.f, 0.f, 0.f);
		capEnd[upAxis] = halfHeight;

		// Draw the ends
		{
			cbtTransform childTransform = transform;
			childTransform.getOrigin() = transform * capStart;
			{
				cbtVector3 center = childTransform.getOrigin();
				cbtVector3 up = childTransform.getBasis().getColumn((upAxis + 1) % 3);
				cbtVector3 axis = -childTransform.getBasis().getColumn(upAxis);
				cbtScalar minTh = -SIMD_HALF_PI;
				cbtScalar maxTh = SIMD_HALF_PI;
				cbtScalar minPs = -SIMD_HALF_PI;
				cbtScalar maxPs = SIMD_HALF_PI;

				drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, cbtScalar(stepDegrees), false);
			}
		}

		{
			cbtTransform childTransform = transform;
			childTransform.getOrigin() = transform * capEnd;
			{
				cbtVector3 center = childTransform.getOrigin();
				cbtVector3 up = childTransform.getBasis().getColumn((upAxis + 1) % 3);
				cbtVector3 axis = childTransform.getBasis().getColumn(upAxis);
				cbtScalar minTh = -SIMD_HALF_PI;
				cbtScalar maxTh = SIMD_HALF_PI;
				cbtScalar minPs = -SIMD_HALF_PI;
				cbtScalar maxPs = SIMD_HALF_PI;
				drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, cbtScalar(stepDegrees), false);
			}
		}

		// Draw some additional lines
		cbtVector3 start = transform.getOrigin();

		for (int i = 0; i < 360; i += stepDegrees)
		{
			capEnd[(upAxis + 1) % 3] = capStart[(upAxis + 1) % 3] = cbtSin(cbtScalar(i) * SIMD_RADS_PER_DEG) * radius;
			capEnd[(upAxis + 2) % 3] = capStart[(upAxis + 2) % 3] = cbtCos(cbtScalar(i) * SIMD_RADS_PER_DEG) * radius;
			drawLine(start + transform.getBasis() * capStart, start + transform.getBasis() * capEnd, color);
		}
	}

	virtual void drawCylinder(cbtScalar radius, cbtScalar halfHeight, int upAxis, const cbtTransform& transform, const cbtVector3& color)
	{
		cbtVector3 start = transform.getOrigin();
		cbtVector3 offsetHeight(0, 0, 0);
		offsetHeight[upAxis] = halfHeight;
		int stepDegrees = 30;
		cbtVector3 capStart(0.f, 0.f, 0.f);
		capStart[upAxis] = -halfHeight;
		cbtVector3 capEnd(0.f, 0.f, 0.f);
		capEnd[upAxis] = halfHeight;

		for (int i = 0; i < 360; i += stepDegrees)
		{
			capEnd[(upAxis + 1) % 3] = capStart[(upAxis + 1) % 3] = cbtSin(cbtScalar(i) * SIMD_RADS_PER_DEG) * radius;
			capEnd[(upAxis + 2) % 3] = capStart[(upAxis + 2) % 3] = cbtCos(cbtScalar(i) * SIMD_RADS_PER_DEG) * radius;
			drawLine(start + transform.getBasis() * capStart, start + transform.getBasis() * capEnd, color);
		}
		// Drawing top and bottom caps of the cylinder
		cbtVector3 yaxis(0, 0, 0);
		yaxis[upAxis] = cbtScalar(1.0);
		cbtVector3 xaxis(0, 0, 0);
		xaxis[(upAxis + 1) % 3] = cbtScalar(1.0);
		drawArc(start - transform.getBasis() * (offsetHeight), transform.getBasis() * yaxis, transform.getBasis() * xaxis, radius, radius, 0, SIMD_2_PI, color, false, cbtScalar(10.0));
		drawArc(start + transform.getBasis() * (offsetHeight), transform.getBasis() * yaxis, transform.getBasis() * xaxis, radius, radius, 0, SIMD_2_PI, color, false, cbtScalar(10.0));
	}

	virtual void drawCone(cbtScalar radius, cbtScalar height, int upAxis, const cbtTransform& transform, const cbtVector3& color)
	{
		int stepDegrees = 30;
		cbtVector3 start = transform.getOrigin();

		cbtVector3 offsetHeight(0, 0, 0);
		cbtScalar halfHeight = height * cbtScalar(0.5);
		offsetHeight[upAxis] = halfHeight;
		cbtVector3 offsetRadius(0, 0, 0);
		offsetRadius[(upAxis + 1) % 3] = radius;
		cbtVector3 offset2Radius(0, 0, 0);
		offset2Radius[(upAxis + 2) % 3] = radius;

		cbtVector3 capEnd(0.f, 0.f, 0.f);
		capEnd[upAxis] = -halfHeight;

		for (int i = 0; i < 360; i += stepDegrees)
		{
			capEnd[(upAxis + 1) % 3] = cbtSin(cbtScalar(i) * SIMD_RADS_PER_DEG) * radius;
			capEnd[(upAxis + 2) % 3] = cbtCos(cbtScalar(i) * SIMD_RADS_PER_DEG) * radius;
			drawLine(start + transform.getBasis() * (offsetHeight), start + transform.getBasis() * capEnd, color);
		}

		drawLine(start + transform.getBasis() * (offsetHeight), start + transform.getBasis() * (-offsetHeight + offsetRadius), color);
		drawLine(start + transform.getBasis() * (offsetHeight), start + transform.getBasis() * (-offsetHeight - offsetRadius), color);
		drawLine(start + transform.getBasis() * (offsetHeight), start + transform.getBasis() * (-offsetHeight + offset2Radius), color);
		drawLine(start + transform.getBasis() * (offsetHeight), start + transform.getBasis() * (-offsetHeight - offset2Radius), color);

		// Drawing the base of the cone
		cbtVector3 yaxis(0, 0, 0);
		yaxis[upAxis] = cbtScalar(1.0);
		cbtVector3 xaxis(0, 0, 0);
		xaxis[(upAxis + 1) % 3] = cbtScalar(1.0);
		drawArc(start - transform.getBasis() * (offsetHeight), transform.getBasis() * yaxis, transform.getBasis() * xaxis, radius, radius, 0, SIMD_2_PI, color, false, 10.0);
	}

	virtual void drawPlane(const cbtVector3& planeNormal, cbtScalar planeConst, const cbtTransform& transform, const cbtVector3& color)
	{
		cbtVector3 planeOrigin = planeNormal * planeConst;
		cbtVector3 vec0, vec1;
		cbtPlaneSpace1(planeNormal, vec0, vec1);
		cbtScalar vecLen = 100.f;
		cbtVector3 pt0 = planeOrigin + vec0 * vecLen;
		cbtVector3 pt1 = planeOrigin - vec0 * vecLen;
		cbtVector3 pt2 = planeOrigin + vec1 * vecLen;
		cbtVector3 pt3 = planeOrigin - vec1 * vecLen;
		drawLine(transform * pt0, transform * pt1, color);
		drawLine(transform * pt2, transform * pt3, color);
	}

	virtual void clearLines()
	{
	}

	virtual void flushLines()
	{
	}
};

#endif  //BT_IDEBUG_DRAW__H
