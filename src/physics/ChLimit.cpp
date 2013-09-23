//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
// 
//   ChLimit.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLimit.h"

namespace chrono 
{

ChLinkLimit::ChLinkLimit() 
{
	active = 0;		// default: inactive limit
	penalty_only = 0; // use also link-locking
	polar = 0; // default: no polar limit
	rotation = 0;
	max = 1;
	min = -1;
	maxCushion = 0.0;
	minCushion = 0.0;
	Kmax= 1000; Kmin = 1000;
	Rmax= 100;  Rmin =  100;
	minElastic = 0.0;
	maxElastic = 0.0;
	modul_Kmax =   new ChFunction_Const (1);  // default: const.modulation of K
	modul_Kmin =   new ChFunction_Const (1);  // default: const.modulation of K
	modul_Rmax =   new ChFunction_Const (1);  // default: const.modulation of K
	modul_Rmin =   new ChFunction_Const (1);  // default: const.modulation of K
	polar_Max  =   new ChFunction_Const (1);  // default function for polar limits
	constr_upper.SetMode(CONSTRAINT_UNILATERAL);
	constr_lower.SetMode(CONSTRAINT_UNILATERAL);
}


ChLinkLimit::~ChLinkLimit () 
{
	if (modul_Kmax) delete modul_Kmax;
	if (modul_Kmin) delete modul_Kmin;
	if (modul_Rmax) delete modul_Rmax;
	if (modul_Rmin) delete modul_Rmin;
	if (polar_Max)  delete polar_Max;
}

void ChLinkLimit::Copy (ChLinkLimit* source)
{
	active = source->active;
	penalty_only = source->penalty_only;
	polar = source->polar;
	rotation = source->rotation;
	max = source->max;
	min = source->min;
	maxCushion = source->maxCushion;
	minCushion = source->minCushion;
	Kmax= source->Kmax;   Kmin = source->Kmin;
	Rmax= source->Rmax;   Rmin = source->Rmin;
	minElastic = source->minElastic;
	maxElastic = source->maxElastic;
		// replace functions:
	if (modul_Kmax) delete modul_Kmax;
	if (modul_Kmin) delete modul_Kmin;
	if (modul_Rmax) delete modul_Rmax;
	if (modul_Rmin) delete modul_Rmin;
	if (polar_Max)  delete polar_Max;
	modul_Kmax =   source->modul_Kmax->new_Duplicate();
	modul_Kmin =   source->modul_Kmin->new_Duplicate();
	modul_Rmax =   source->modul_Rmax->new_Duplicate();
	modul_Rmin =   source->modul_Rmin->new_Duplicate();
	if (source->polar_Max) 
		polar_Max =   source->polar_Max->new_Duplicate();
	else polar_Max = NULL;
}

ChLinkLimit* ChLinkLimit::new_Duplicate ()
{
	ChLinkLimit* m_lim;
	m_lim = new ChLinkLimit;
	m_lim->Copy(this);
	return (m_lim);
}



void ChLinkLimit::Set_max(double m_max)
{
	max = m_max;
	if (max < min) min = max;
	if ((max-maxCushion) < min) maxCushion = max - min;
	if ((max-maxCushion) < (min+minCushion)) minCushion = max - min - maxCushion;
}

void ChLinkLimit::Set_min(double m_min)
{
	min = m_min;
	if (min > max) max = min;
	if ((min+minCushion) > max) minCushion = max - min;
	if ((min+minCushion) > (max-maxCushion)) maxCushion = max - min - minCushion;
}

void ChLinkLimit::Set_maxCushion(double m_maxCushion)
{
	maxCushion = m_maxCushion;
	if ((max-maxCushion) < min)	 maxCushion = max - min;
	if ((max-maxCushion) < (min+minCushion)) minCushion = max - min - maxCushion;
}

void ChLinkLimit::Set_minCushion(double m_minCushion)
{
	minCushion = m_minCushion;
	if ((min+minCushion) > max) minCushion = max - min;
	if ((min+minCushion) > (max-maxCushion)) maxCushion = max - min - minCushion;
}

void ChLinkLimit::SetModul_Kmax	(ChFunction* m_funct)
{
	if (modul_Kmax) delete modul_Kmax;
	modul_Kmax = m_funct;
}
void ChLinkLimit::SetModul_Kmin	(ChFunction* m_funct)
{
	if (modul_Kmin) delete modul_Kmin;
	modul_Kmin = m_funct;
}
void ChLinkLimit::SetModul_Rmax	(ChFunction* m_funct)
{
	if (modul_Rmax) delete modul_Rmax;
	modul_Rmax = m_funct;
}
void ChLinkLimit::SetModul_Rmin	(ChFunction* m_funct)
{
	if (modul_Rmin) delete modul_Rmin;
	modul_Rmin = m_funct;
}
void ChLinkLimit::SetPolar_Max	(ChFunction* m_funct)
{
	if (polar_Max) delete polar_Max;
	polar_Max = m_funct;
}





// file parsing / dumping



void ChLinkLimit::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number 
	mstream.VersionWrite(1); 

		// stream out all member data
	mstream << Get_active();
	mstream << Get_polar();
	mstream << Get_penalty();
	mstream << Get_min();
	mstream << Get_max();
	mstream << Get_minCushion();
	mstream << Get_maxCushion();
	mstream << Get_minElastic();
	mstream << Get_maxElastic();
	mstream << Get_Kmin();
	mstream << Get_Kmax();
	mstream << Get_Rmin();
	mstream << Get_Rmax();
	mstream.AbstractWrite(GetPolar_Max());
	mstream.AbstractWrite(GetModul_Kmin());
	mstream.AbstractWrite(GetModul_Kmax());
	mstream.AbstractWrite(GetModul_Rmin());
	mstream.AbstractWrite(GetModul_Rmax());
}
				
void ChLinkLimit::StreamIN(ChStreamInBinary& mstream)
{
		// class version number 
	int version = mstream.VersionRead();

		// stream in all member data
	double dfoo;
	int ifoo;
	ChFunction* ffoo;
	mstream >> ifoo;		Set_active(ifoo);
	mstream >> ifoo;		Set_polar(ifoo);
	mstream >> ifoo;		Set_penalty(ifoo);
	mstream >> dfoo;		Set_min(dfoo);
	mstream >> dfoo;		Set_max(dfoo);
	mstream >> dfoo;		Set_minCushion(dfoo);
	mstream >> dfoo;		Set_maxCushion(dfoo);	
	mstream >> dfoo;		Set_minElastic(dfoo);
	mstream >> dfoo;		Set_maxElastic(dfoo);
	mstream >> dfoo;		Set_Kmin(dfoo);
	mstream >> dfoo;		Set_Kmax(dfoo);
	mstream >> dfoo;		Set_Rmin(dfoo);
	mstream >> dfoo;		Set_Rmax(dfoo);
	mstream.AbstractReadCreate(&ffoo);		SetPolar_Max(ffoo);
	mstream.AbstractReadCreate(&ffoo);		SetModul_Kmin(ffoo);
	mstream.AbstractReadCreate(&ffoo);		SetModul_Kmax(ffoo);
	mstream.AbstractReadCreate(&ffoo);		SetModul_Rmin(ffoo);
	mstream.AbstractReadCreate(&ffoo);		SetModul_Rmax(ffoo);
}

void ChLinkLimit::StreamOUT(ChStreamOutAscii& mstream)
{
	//***TO DO***
}
		




///////////////////

double ChLinkLimit::GetViolation (double x)
{
	if ((active == 0) || (penalty_only == 1))
	{
		return 0;
	}
	else
	{
		if ((x > min) && (x < max)) return 0; 
		if (x <= min) return (x-min);
		if (x >= max) return (x-max);
	}
	return 0;
}


double ChLinkLimit::GetForce (double x, double x_dt)
{
	double cush_coord;
	double cush_coord_norm;
	double force;
	double m_min, m_max;

	if (penalty_only == 0)
	{
		m_min = min;
		m_max = max;
	}
	else
	{
		m_min = -999999999;
		m_max =  999999999;
	}

	if ((x > m_min) && (x < (min + minCushion)) )
	{
		cush_coord = (min + minCushion) - x;

		if (minCushion >= 0.0000001)
			cush_coord_norm = cush_coord / minCushion;
		else cush_coord_norm = 1;
			
		if (cush_coord_norm > 1) cush_coord_norm = 1;  // clip cushion forces at stopper limit

		force = cush_coord * Kmin * modul_Kmin->Get_y(cush_coord_norm);
		force +=   (-x_dt) * Rmin * modul_Rmin->Get_y(cush_coord_norm);
		if (force < 0) {force = 0;} // damping could cause neg force while going away,
									// so -as the limit is not "sticky"- clip force sign.	
		
		return (force); 
	}

	if ((x < m_max) && (x > (max - maxCushion)) )
	{
		cush_coord = x - (max - maxCushion);

		if (maxCushion >= 0.0000001)
			cush_coord_norm = cush_coord / maxCushion;
		else cush_coord_norm = 1;

		if (cush_coord_norm > 1) cush_coord_norm = 1;  // clip cushion forces at stopper limit

		force = (-cush_coord) * Kmax * modul_Kmax->Get_y(cush_coord_norm);
		force +=   (-x_dt) * Rmax * modul_Rmax->Get_y(cush_coord_norm);
		if (force > 0) {force = 0;} // damping could cause pos force while going away,
									// so -as the limit is not "sticky"- clip force sign.	
		return (force); 
	}
	return 0;
}


////

double ChLinkLimit::Get_polar_max(double pol_ang)
{
	if (!polar_Max) return 0.001;
	return (polar_Max->Get_y(pol_ang));
}

//// The same, but for conical limits, in polar coordinates



double ChLinkLimit::GetPolarForce(double x, double x_dt, double pol_ang)
{
	double cush_coord;
	double cush_coord_norm;
	double cushion_thick;
	double force;
	double m_max;
	double ang_max;

	if (!polar_Max) return 0;

	if (penalty_only == 0)
	{
		m_max = max;
	}
	else
	{
		m_max =  999999999;
	}

	ang_max = polar_Max->Get_y(pol_ang);

	if ( (x < m_max) && (x > (ang_max - maxCushion)) )
	{
		cushion_thick = maxCushion;
		if (cushion_thick > ang_max) cushion_thick = ang_max;

		cush_coord = x - (ang_max - maxCushion);

		if (cushion_thick >= 0.0000001)
			cush_coord_norm = cush_coord / cushion_thick;
		else cush_coord_norm = 1;

		if (cush_coord_norm > 1) cush_coord_norm = 1;  // clip cushion forces at stopper limit

		force = (-cush_coord) * Kmax * modul_Kmax->Get_y(cush_coord_norm);
		force +=   (-x_dt) * Rmax * modul_Rmax->Get_y(cush_coord_norm);
		if (force > 0) {force = 0;} // damping could cause pos force while going away,
									// so -as the limit is not "sticky"- clip force sign.	
		return (force); 
	}
	return 0;
}


} // END_OF_NAMESPACE____

// eof


