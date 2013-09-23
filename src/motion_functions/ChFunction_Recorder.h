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

#ifndef CHFUNCT_RECORDER_H
#define CHFUNCT_RECORDER_H

//////////////////////////////////////////////////
//  
//   ChFunction_Recorder.h
//
//   Function objects, 
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Base.h"


namespace chrono 
{

#define FUNCT_RECORDER	11



class ChApi ChRecPoint 
{
public:
	double x;	
	double y;
	double w;  // weight
};

#define CH_RECORDER_EPSILON 1.e-10


/////////////////////////////////////////////
/// RECORDER FUNCTION
/// y = interpolation of array of (x,y) data, 
///     where (x,y) points can be inserted randomly.

class ChApi ChFunction_Recorder : public ChFunction
{
	CH_RTTI(ChFunction_Recorder, ChFunction);
private:
	ChList<ChRecPoint> points;		// the list of points
	ChNode<ChRecPoint>* lastnode;	// speed optimization: remember the last used pointer

public:
	ChFunction_Recorder () {lastnode = NULL;};
	~ChFunction_Recorder () {points.KillAll();};
	void Copy (ChFunction_Recorder* source);
	ChFunction* new_Duplicate ();

	int AddPoint (double mx, double my, double mw);
	int AddPoint (double mx, double my) {return AddPoint(mx,my,1.0);};
	int AddPointClean (double mx, double my, double dx_clean); // also clean nodes to the right, upt to dx interval
	void Reset() {points.KillAll(); lastnode = NULL;};

	ChList<ChRecPoint>*  GetPointList() {return &points;};

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);

	int Get_Type () {return (FUNCT_RECORDER);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



} // END_OF_NAMESPACE____


#endif
