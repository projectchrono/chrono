//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction_Recorder.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Recorder.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Recorder> a_registration_recorder;


void ChFunction_Recorder::Copy (ChFunction_Recorder* source)
{
	points.KillAll();
	lastnode = NULL;
	ChRecPoint* mpt;
	for (ChNode<ChRecPoint>* mnode = source->points.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mpt = new ChRecPoint;
		mpt->x = mnode->data->x;
		mpt->y = mnode->data->y;
		mpt->w = mnode->data->w;
		points.AddTail(mpt);
	}
}

ChFunction* ChFunction_Recorder::new_Duplicate ()
{
	ChFunction_Recorder* m_func;
	m_func = new ChFunction_Recorder;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Recorder::Estimate_x_range (double& xmin, double& xmax)
{
	if (!points.GetTail())
	{
		xmin = 0.0; xmax = 1.2;
		return;
	}
	xmin = points.GetHead()->data->x;
	xmax = points.GetTail()->data->x;
	if (xmin == xmax) xmax = xmin+ 0.5;
}


int ChFunction_Recorder::AddPoint (double mx, double my, double mw)
{
	ChRecPoint* mpt = new ChRecPoint;
	mpt->x = mx;  	mpt->y = my;  	mpt->w = mw;
	double dist;

	if (!points.GetTail())
	{
		points.AddTail(mpt);
		return TRUE;
	}

	for (ChNode<ChRecPoint>* mnode = points.GetTail(); mnode != NULL; mnode= mnode->prev)
	{
		dist = mx - mnode->data->x;
		if (fabs(dist) < CH_RECORDER_EPSILON)
		{
			mnode->data = mpt;	// copy to preexisting node
			return TRUE;
		}
		if (dist > 0.0)
		{
			points.InsertAfter(mnode, mpt);  // insert
			return TRUE;
		}
	}


	points.AddHead(mpt); // was 1st in list
	return TRUE;
}

int ChFunction_Recorder::AddPointClean (double mx, double my, double dx_clean)
{
	ChRecPoint* mpt = new ChRecPoint;
	mpt->x = mx;  	mpt->y = my;  	mpt->w = 0;
	double dist;
	ChNode<ChRecPoint>* msetnode = NULL;

	if (!points.GetHead())
	{
		points.AddHead(mpt);
		lastnode = msetnode = points.GetHead();
		return TRUE;
	}

	if (!lastnode)
		lastnode = points.GetHead();

	if (lastnode->data->x < mx) // 1 forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= mx)
			{
				lastnode = mnode;
				dist = - mx + mnode->data->x;
				if (fabs(dist) < CH_RECORDER_EPSILON) {
					mnode->data = mpt;
					msetnode = mnode;}	// copy to preexisting node
				else {
					points.InsertBefore(mnode, mpt);
					msetnode = mnode->prev;} // insert
				break;

			}
		}
		if (!msetnode)
		{			// ..oh, it should be tail..
			points.AddTail(mpt);
			msetnode = points.GetTail();
		}
	}
	else						// 2 backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= mx)
			{
				lastnode = mnode;
				dist =  mx - mnode->data->x;
				if (fabs(dist) < CH_RECORDER_EPSILON) {
					mnode->data = mpt;	// copy to preexisting node
					msetnode = mnode;}
				else {
					points.InsertAfter(mnode, mpt);  // insert
					msetnode = mnode->next; }
				break;
			}
		}
		if (!msetnode)
		{			// ..oh, it should be head
			points.AddHead(mpt);
			msetnode = points.GetHead();
		}
	}

	lastnode = msetnode;

	// clean on dx
	if (dx_clean >0)
	{
		ChNode<ChRecPoint>* mnode = msetnode->next;
		ChNode<ChRecPoint>* mnextnode = NULL;
		while (mnode != NULL)
		{
			mnextnode = mnode->next;
			if ((mnode->data->x - mx) < dx_clean)
			{
				points.Kill(mnode);
			}
			mnode = mnextnode;
		}
	}

	return TRUE;
}

double ChFunction_Recorder::Get_y      (double x)
{
	double y = 0;

	ChRecPoint p1;
	p1.w = 1;
	p1.x = 0;
	p1.y = 0;
	ChRecPoint p2;
	p2 = p1;
	p2.x = p1.x + 1.0;

	if (points.GetHead() == NULL) return 0.0;

	if (x < points.GetHead()->data->x) return 0.0;
	if (x > points.GetTail()->data->x) return 0.0;

	if (!lastnode) lastnode = points.GetHead();

	if (lastnode->data->x < x) // forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= x)
			{
				p2 = *mnode->data; lastnode = mnode;
				if (mnode->prev)	{ p1 = *mnode->prev->data; }
				break;
			}
		}
	}
	else	// backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= x)
			{
				p1 = *mnode->data;	lastnode = mnode;
				if (mnode->next)	{ p2 = *mnode->next->data; }
				break;
			}
		}
	}


	y = ((x - p1.x)*p2.y + (p2.x -x)*p1.y)/(p2.x - p1.x);

	return y;
}

double ChFunction_Recorder::Get_y_dx   (double x)
{
	double dy = 0;

	ChRecPoint p1;					//    p0...p1..x...p2.....p3
	p1.x = x;	p1.y = 0;	p1.w = 1;
	ChRecPoint p2;
	p2 = p1;	p2.x += 100.0;
	ChRecPoint p0;
	p0 = p1;	p0.x -= 100.0;
	ChRecPoint p3;
	p3 = p1;	p3.x += 200.0;

	if (points.GetHead() == NULL) return 0.0;

	if (x < points.GetHead()->data->x) return 0.0;
	if (x > points.GetTail()->data->x) return 0.0;

	if (!lastnode) lastnode = points.GetHead();

	if (lastnode->data->x < x) // forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= x)
			{
				p2 = *mnode->data; lastnode = mnode;
				if (mnode->prev) {
					p1 = *mnode->prev->data;
					if (mnode->prev->prev) {
						p0 = *mnode->prev->prev->data;
						if (mnode->next) {
							p3 = *mnode->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p1 = p2; p1.x -= 1.0;}

				break;
			}
		}
	}
	else	// backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= x)
			{
				p1 = *mnode->data;	lastnode = mnode;
				if (mnode->next)  {
					p2 = *mnode->next->data;
					if (mnode->prev)  {
						p0 = *mnode->prev->data;
						if (mnode->next->next)  {
							p3 = *mnode->next->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p2 = p1; p2.x += 1.0;}

				break;
			}
		}
	}

	double vA = (p1.y -p0.y)/(p1.x -p0.x);
	double vB = (p2.y -p1.y)/(p2.x -p1.x);
	double vC = (p3.y -p2.y)/(p3.x -p2.x);

	double v1 = 0.5* (vA + vB);
	double v2 = 0.5* (vB + vC);

	dy = ((x - p1.x)* v2 + (p2.x -x)* v1)  /  (p2.x - p1.x);

	return dy;
}

double ChFunction_Recorder::Get_y_dxdx (double x)
{
	double ddy = 0;

	ChRecPoint p1;					//    p0...p1..x...p2.....p3
	p1.x = x;	p1.y = 0;	p1.w = 1;
	ChRecPoint p2;
	p2 = p1;	p2.x = p1.x + 100.0;
	ChRecPoint p0;
	p0 = p1;	p0.x = p1.x - 100.0;
	ChRecPoint p3;
	p3 = p1;	p3.x = p1.x + 200.0;

	if (points.GetHead() == NULL) return 0.0;

	if (x < points.GetHead()->data->x) return 0.0;
	if (x > points.GetTail()->data->x) return 0.0;

	if (!lastnode) lastnode = points.GetHead();

	if (lastnode->data->x < x) // forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= x)
			{
				p2 = *mnode->data; lastnode = mnode;
				if (mnode->prev) {
					p1 = *mnode->prev->data;
					if (mnode->prev->prev) {
						p0 = *mnode->prev->prev->data;
						if (mnode->next) {
							p3 = *mnode->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p1 = p2; p1.x -= 1.0;}

				break;
			}
		}
	}
	else	// backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= x)
			{
				p1 = *mnode->data;	lastnode = mnode;
				if (mnode->next)  {
					p2 = *mnode->next->data;
					if (mnode->prev)  {
						p0 = *mnode->prev->data;
						if (mnode->next->next)  {
							p3 = *mnode->next->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p2 = p1; p2.x += 1.0;}

				break;
			}
		}
	}

	double vA = (p1.y -p0.y)/(p1.x -p0.x);
	double vB = (p2.y -p1.y)/(p2.x -p1.x);
	double vC = (p3.y -p2.y)/(p3.x -p2.x);

	double a1 = 2.0 * (vB - vA)/(p2.x - p0.x);
	double a2 = 2.0 * (vC - vB)/(p3.x - p1.x);

	ddy = ((x - p1.x)* a2 + (p2.x -x)* a1)  /  (p2.x - p1.x);

	return ddy;
}


void ChFunction_Recorder::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << (int)(points.Count());

	for (ChNode<ChRecPoint>* mnode = points.GetTail(); mnode != NULL; mnode= mnode->prev)
	{
		mstream << mnode->data->x;
		mstream << mnode->data->y;
		mstream << mnode->data->w;
	}
}

void ChFunction_Recorder::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	int mcount;
	mstream >> mcount;

	for (int i = 0; i < mcount; i++)
	{
		ChRecPoint* mpt = new ChRecPoint;
		mstream >> mpt->x;
		mstream >> mpt->y;
		mstream >> mpt->w;
		points.AddHead(mpt);
	}

}

void ChFunction_Recorder::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_RECORDER  \n";

	//***TO DO***
}






} // END_OF_NAMESPACE____


// eof
