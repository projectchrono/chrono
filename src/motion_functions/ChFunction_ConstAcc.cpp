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
//   ChFunction_ConstAcc.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_ConstAcc.h"


namespace chrono
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_ConstAcc> a_registration_constacc;


void ChFunction_ConstAcc::Copy (ChFunction_ConstAcc* source)
{
	h = source->h;
	av = source->av;
	aw = source->aw;
	end = source->end;
}

ChFunction* ChFunction_ConstAcc::new_Duplicate ()
{
	ChFunction_ConstAcc* m_func;
	m_func = new ChFunction_ConstAcc;
	m_func->Copy(this);
	return (m_func);
}


double ChFunction_ConstAcc::Get_y      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return h;
	double ev = av*end;
	double ew = aw*end;
	double A = 2*h/((ev)*(end-ev+ew));
	double B = 2*h/((end-ew)*(end-ev+ew));
	if ((x>0)&&(x<ev))
	{
		ret = 0.5*A*x*x;
	}
	if ((x>=ev)&&(x<=ew))
	{
		ret = A*ev*(x-ev*0.5);
	}
	if ((x>ew)&&(x<end))
	{
		ret = A*ev*(x-ev*0.5) - B*0.5*pow((x-ew),2);
	}
	return ret;
}

double ChFunction_ConstAcc::Get_y_dx   (double x)
{
	double ret = 0;
	double ev = av*end;
	double ew = aw*end;
	double A = 2*h/((ev)*(end-ev+ew));
	double B = 2*h/((end-ew)*(end-ev+ew));
	if ((x>0)&&(x<ev))
	{
		ret = A*x;
	}
	if ((x>=ev)&&(x<=ew))
	{
		ret = A*ev;
	}
	if ((x>ew)&&(x<end))
	{
		ret = A*ev - B*(x-ew);
	}
	return ret;
}

double ChFunction_ConstAcc::Get_y_dxdx (double x)
{
	double ret = 0;
	double ev = av*end;
	double ew = aw*end;
	double A = 2*h/((ev)*(end-ev+ew));
	double B = 2*h/((end-ew)*(end-ev+ew));
	if ((x>0)&&(x<ev))
	{
		ret = A;
	}
	if ((x>=ev)&&(x<=ew))
	{
		ret = 0;
	}
	if ((x>ew)&&(x<end))
	{
		ret = -B;
	}
	return ret;
}

double ChFunction_ConstAcc::Get_Ca_pos ()
{
	return 2*(end*end)/(av*end*(end - av*end + aw*end));
}
double ChFunction_ConstAcc::Get_Ca_neg ()
{
	return 2*(end*end)/((end - aw*end)*(end - av*end + aw*end));
}
double ChFunction_ConstAcc::Get_Cv ()
{
	return 2*(end)/(end - av*end + aw*end);
}

void ChFunction_ConstAcc::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << h;
	mstream << aw;
	mstream << av;
	mstream << end;
}

void ChFunction_ConstAcc::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> h;
	mstream >> aw;
	mstream >> av;
	mstream >> end;
}

void ChFunction_ConstAcc::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}





} // END_OF_NAMESPACE____


// eof
