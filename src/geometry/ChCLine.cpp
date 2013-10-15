//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChCline.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "ChCLine.h"

namespace chrono
{
namespace geometry
{

/////////////////////////////////////////////////////////
///
///   GENERIC LINE CLASS
///
///


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLine> a_registration_ChLine;




//
// basic geometric  functions
//

int ChLine::FindNearestLinePoint (Vector& point, double& resU, double approxU, double tol)
{
	double mu;
	int points = 20;
	int closed = FALSE;
	double bestU = 0;
	double bestdist = 9999999;
	double dist, d1, d2;
	int iters = 0;
	int maxiters = 11;

	Vector vres, vp1, vp2;

	points = this->Get_complexity();
	closed = this->Get_closed();

	points = points * 4; // double sampling along line.

		// first approximation
	for (int i = 0; i<=points; i++)
	{
		mu = (double)i/(double)points;
		this->Evaluate(vres, mu);
		dist = Vlenght(Vsub(vres,point));
		if (dist < bestdist)
		{
			bestdist = dist;
			bestU = mu;
		}
	}
		// refine position with pseudo-NR
	double step = 1/(double)points;
	double nrU = bestU;
	double u1, u2;

	u1 = nrU - step;
	if (u1 < 0)
		{ if (!closed) u1 = 0;	else u1 = u1 + 1;}
	this->Evaluate(vres, u1);
	d1 = Vlenght(Vsub(vres,point));
	vp1 = vres;

	u2 = nrU + step;
	if (u2 > 1)
		{ if (!closed) u2 = 1;	else u2 = u2 - 1;}
	this->Evaluate(vres, u2);
	d2 = Vlenght(Vsub(vres,point));
	vp2 = vres;

	while (TRUE)
	{
		iters++;

		if (nrU < 0)
			{ if (!closed) nrU = 0;	else nrU = nrU + 1;}
		if (nrU > 1)
			{ if (!closed) nrU = 1;	else nrU = nrU - 1;}
		this->Evaluate(vres, nrU);
		dist = Vlenght(Vsub(vres,point));

		bestU = nrU;

		if (d1 < d2)
		{
			u2 = nrU; d2 = dist; vp2 = vres; // move point 2 to nrU
			step = step / 2;
			nrU = nrU - step;	 // move nrU in middle
			if (d1 < dist) bestU = u1;
		}
		else
		{
			u1 = nrU; d1 = dist; vp1 = vres; // move point 1 to nrU
			step = step / 2;
			nrU = nrU + step;	 // move nrU in middle
			if (d2 < dist) bestU = u2;
		}

		if ((Vlenght(Vsub(vp1, vp2)) <= tol)||(dist <= tol)) {
			resU = bestU;
			return TRUE; }
		if (iters > maxiters)  {resU = bestU;
			return FALSE;}
	}

	resU = bestU;
	return TRUE;
}


double ChLine::CurveCurveDist (ChLine* compline, int samples)
{
	double mres = 0;
	double par;
	// distances this<->compare_line
	for (par = 0; par < 1; par = par + 1/((double)samples))
	{
		double mpos;
		Vector ptB; compline->Evaluate(ptB, par);
		this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
		Vector ptA; this->Evaluate(ptA, mpos);
		mres += Vlenght(Vsub(ptA,ptB));
	}
	// ..and viceversa
	for (par = 0; par < 1; par = par + 1/((double)samples))
	{
		double mpos;
		Vector ptB; this->Evaluate(ptB, par);
		compline->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
		Vector ptA; compline->Evaluate(ptA, mpos);
		mres += Vlenght(Vsub(ptA,ptB));
	}

	return (mres/(samples*2));
}

double ChLine::CurveCurveDistMax (ChLine* compline, int samples)
{
	double mres = 0;
	double par;
	double mdis;
	// distances this<->compare_line
	for (par = 0; par < 1; par = par + 1/((double)samples))
	{
		double mpos;
		Vector ptB; compline->Evaluate(ptB, par);
		this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
		Vector ptA; this->Evaluate(ptA, mpos);
		mdis = Vlenght(Vsub(ptA,ptB));
		if (mres < mdis) mres = mdis;
	}
	// ..and viceversa
	for (par = 0; par < 1; par = par + 1/((double)samples))
	{
		double mpos;
		Vector ptB;  this->Evaluate(ptB, par);
		compline->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
		Vector ptA;  compline->Evaluate(ptA, mpos);
		mdis = Vlenght(Vsub(ptA,ptB));
		if (mres < mdis) mres = mdis;
	}

	return mres;
}


double ChLine::CurveSegmentDist (ChLine* complinesegm, int samples)
{
	double mres = 0;
	double par;
	// distances this<->compare_line
	for (par = 0; par < 1; par = par + 1/((double)samples))
	{
		double mpos;
		Vector ptB;  complinesegm->Evaluate(ptB, par);
		this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
		Vector ptA;  this->Evaluate(ptA, mpos);
		mres += Vlenght(Vsub(ptA,ptB));
	}
	return (mres/samples);
}

double ChLine::CurveSegmentDistMax (ChLine* complinesegm, int samples)
{
	double mres = 0;
	double par;
	double mdis;
	// distances this<->compare_line
	for (par = 0; par < 1; par = par + 1/((double)samples))
	{
		double mpos;
		Vector ptB;  complinesegm->Evaluate(ptB, par);
		this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
		Vector ptA;  this->Evaluate(ptA, mpos);
		mdis = Vlenght(Vsub(ptA,ptB));
		if (mres < mdis) mres = mdis;
	}
	return (mres);
}

double ChLine::Lenght (int sampling)
{
	double mres = 0;
	double par, step;

	if (!closed)
		step = 1/((double)(Get_complexity()-1));
	else
		step = 1/((double)(Get_complexity()));

	if (sampling >1)
		step = step/(double)sampling;

	Vector pA; this->Evaluate(pA, 0.);
	Vector pB;
	for (par = 0; par <= 1.000000001; par = par + step)
	{
		this->Evaluate(pB, par);
		mres += Vlenght(Vsub(pA,pB));
		pA = pB;
	}
	return mres;
}





// Draw into the current graph viewport of a ChFile_ps file

int ChLine::DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate)
{
	ChPageVect mp1;
	Vector mv1;

	mfle->GrSave();
	mfle->ClipRectangle(mfle->Get_G_p(), mfle->Get_Gs_p(), PS_SPACE_PAGE);
				// start a line, move cursor to beginning
	mfle->StartLine();
	this->Evaluate(mv1, 0.0);
	mp1.x = mv1.x;	mp1.y = mv1.y;
	mp1 = mfle->To_page_from_graph(mp1);
	mfle->MoveTo(mp1);
	double maxpoints = this->Get_complexity()*10;
				// add points into line
	for (int i = 1; i <= maxpoints; i++)
	{
		this->Evaluate(mv1, ((double)i)/((double)maxpoints));
		mp1.x = mv1.x;		mp1.y = mv1.y;
		mp1 = mfle->To_page_from_graph(mp1);
		mfle->AddLinePoint(mp1);
	}
	if (this->Get_closed())
		mfle->CloseLine();		// if periodic curve, close it

	mfle->PaintStroke();	// draw it!
	mfle->GrRestore();		// restore old modes, with old clipping

	return TRUE;
}




void ChLine::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChGeometry::StreamOUT(mstream);

		// stream out all member data
	mstream << closed;
	mstream << complexityU;
}

void ChLine::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChGeometry::StreamIN(mstream);

		// stream in all member data
	mstream >> closed;
	mstream >> complexityU;
}




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

////// end
