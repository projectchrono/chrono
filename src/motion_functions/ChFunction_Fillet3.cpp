///////////////////////////////////////////////////
//
//   ChFunction_Fillet3.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Fillet3.h"
#include "core/ChLinearAlgebra.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Fillet3> a_registration_fillet3;


void ChFunction_Fillet3::Copy (ChFunction_Fillet3* source)
{
	end = source->end;
	y1  = source->y1;
	y2 = source->y2;
	dy1 = source->dy1;
	dy2 = source->dy2;

	SetupCoefficients();
}

ChFunction* ChFunction_Fillet3::new_Duplicate ()
{
	ChFunction_Fillet3* m_func;
	m_func = new ChFunction_Fillet3;
	m_func->Copy(this);
	return (m_func);
}


double ChFunction_Fillet3::Get_y      (double x)
{
	double ret = 0;
	if (x<=0) return y1;
	if (x>=end) return y2;
	double a = x/end;
	ret = c1*pow(x,3) + c2*pow(x,2) + c3*x + c4;
 	return ret;
}

double ChFunction_Fillet3::Get_y_dx      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = 3*c1*pow(x,2) + 2*c2*x + c3;
 	return ret;
}

double ChFunction_Fillet3::Get_y_dxdx      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = 6*c1*x + 2*c2;
 	return ret;
}

int ChFunction_Fillet3::SetupCoefficients()
{
	ChMatrixDynamic<> ma(4,4);
	ChMatrixDynamic<> mb(4,1);
	ChMatrixDynamic<> mx(4,1);

	mb(0,0) = y1;
	mb(1,0) = y2;
	mb(2,0) = dy1;
	mb(3,0) = dy2;

	ma(0,3) = 1.0;

	ma(1,0) = pow(end,3);
	ma(1,1) = pow(end,2);
	ma(1,2) = end;
	ma(1,3) = 1.0;

	ma(2,2) = 1.0;

	ma(3,0) = 3*pow(end,2);
	ma(3,1) = 2*end;
	ma(3,2) = 1.0;

	ChLinearAlgebra::Solve_LinSys(ma, &mb, &mx);

	c1= mx(0,0);	c2= mx(1,0);	c3= mx(2,0);	c4= mx(3,0);

 	return TRUE;
}

void ChFunction_Fillet3::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << end;
	mstream << y1;
	mstream << y2;
	mstream << dy1;
	mstream << dy2;
}

void ChFunction_Fillet3::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> end;
	mstream >> y1;
	mstream >> y2;
	mstream >> dy1;
	mstream >> dy2;
	SetupCoefficients();
}

void ChFunction_Fillet3::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_FILLET3  \n";

	//***TO DO***
}







} // END_OF_NAMESPACE____


// eof
