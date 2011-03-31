///////////////////////////////////////////////////
//
//   ChCLineCam.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  

#include "ChCLineCam.h"


namespace chrono
{
namespace geometry
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLineCam> a_registration_ChLineCam;




ChLineCam::ChLineCam ()
{
	Set_complexity(10);
	this->closed = TRUE;
	type = CAM_TYPE_SLIDEFOLLOWER;
	Rb = 1.0;
	Rr = 0.0;
	p = 1.8;
	d = 2;
	b0 = CH_C_PI / 6.0;
	center = VNULL;
	e = 0;
	s = Rb;
	law = new ChFunction_Const(0);
	negative = FALSE;
	internal = FALSE;
}

ChLineCam::~ChLineCam ()
{
	if (law) {delete law; law = 0;}
};

void ChLineCam::Copy (const ChLineCam* source) 
{
	// copy parent class data
	ChLine::Copy(source);
	// copy class data

	type = source->type;

	Rb = source->Rb;
	Rr = source->Rr;
	p = source->p;
	d = source->d;
	b0 = source->b0;
	center = source->center;
	e = source->e;
	s = source->s;
	if (law) 
		delete law; 
	law = source->law->new_Duplicate();
	negative = source->negative;
	internal = source->internal;
};

void ChLineCam::EvaluateCamPoint(double par, Vector& res, double& g, double& q)
{
	double a = par * 2 * CH_C_PI;  // range : par 0..1 -> angle 0...2PI
	double r, f, b, B, fshift,y,ydx, ydxdx, sa, fxalpha, u, uh = 0;
	double sign, signdx, signdxdx;

	// defaults
	g = 0;
	q = 0;
	res = VNULL;

	if (internal) Rr = -Rr;
	if (negative) e = -e;

	fxalpha = a;
	sign = signdx = signdxdx = +1;

	if (negative)
	{
		sign = signdx = signdxdx = -1;	// reverse sign

		if ((type == CAM_TYPE_ROTATEFOLLOWER) ||
			(type == CAM_TYPE_FLATOSCILLATE) )
		{
				fxalpha = 2*CH_C_PI - a;			// also reverse direction
				signdx = signdxdx = +1;
		}
	}

	y = sign * law->Get_y(fxalpha);
	ydx = signdx * law->Get_y_dx(fxalpha);
	ydxdx = signdxdx * law->Get_y_dxdx(fxalpha);


	switch (this->type)
	{
	case CAM_TYPE_SLIDEFOLLOWER:
		g = atan(ydx /(Rb+y));
		r = sqrt(Rr*Rr + pow(Rb+y,2) -2*Rr*(Rb+y)*cos(g));
		fshift = asin(Rr*sin(g)/r);
		if (Rr > Rb) fshift = CH_C_PI - fshift;
		f = a + fshift;
		q = pow(ydx*ydx + pow(Rb+y,2) , 1.5) / (pow(Rb+y,2)-ydxdx*(Rb+y)+2*(ydx*ydx)) - Rr;
		break;
	case CAM_TYPE_ROTATEFOLLOWER:
		b = b0 + y;
		u = atan2 ( (p*sin(b)*(1-ydx)) , (d - p*cos(b)*(1-ydx)) );
		g = CH_C_PI/2.0 - b - u;
		r = sqrt( pow( p*sin(b)-Rr*sin(u), 2) + pow ( d-p*cos(b)-Rr*cos(u) ,2) );
		fshift = atan2( (p*sin(b)-Rr*sin(u)) , (d-p*cos(b)-Rr*cos(u)) );
		f = (a + fshift);
		uh = (p*(1-ydx)*ydx*cos(b+u) - p*ydxdx*sin(b+u)) / (d*cos(u)-p*(1-ydx)*cos(b+u));
		q = ((p*cos(b0+y)*(1-ydx))+d) / ((1+uh)*cos(u)) - Rr;
		break;
	case CAM_TYPE_ECCENTRICFOLLOWER:
		s = Get_s();
		sa = s+y;
		g = atan ( (ydx - e) / (s+y) );
		r = sqrt ( pow(( sa -Rr*cos(g)),2) + pow((e+Rr*sin(g)),2) );
		fshift = atan( (e+Rr*sin(g))/( sa -Rr*cos(g)) );
		if (Rr > Rb) fshift = CH_C_PI + fshift;
		f = a + fshift;
		q = pow((pow(s+y,2)+pow(e-ydx,2) ),1.5) / (pow(s+y,2)+(e-ydx)*(e-2*ydx)-(s+y)*ydxdx) - Rr;
		break;
	case CAM_TYPE_FLAT:
		g = 0;
		B = ydx;
		r = sqrt (pow(Rb+y,2) + ydx*ydx );
		f = a + atan2( ydx , (Rb+y));
		q = Rb+y+ydxdx;
		break;
	case CAM_TYPE_FLATOSCILLATE:
		b = b0 + y;
		B = (d*cos(b)) / (1 - ydx);
		g = atan2 (e,B);
		r = sqrt ( pow(d - e *sin(b) - B*cos(b), 2) + pow (B*sin(b)-e*cos(b), 2) );
		f = a + atan2( (B*sin(b)-e*cos(b)) , (d - e *sin(b) - B*cos(b)) );
		q = (d*sin(b)*(1-2*ydx)+B*ydxdx ) / (pow(1-ydx,2)) - e;
		break;
	default:
		g = r = f = 0;
		break;
	}

	if (negative)
	{
		if ((type == CAM_TYPE_FLAT) ||
			(type == CAM_TYPE_SLIDEFOLLOWER) ||
			(type == CAM_TYPE_ECCENTRICFOLLOWER) )
				f += CH_C_PI;		// polar 180

		if ((type == CAM_TYPE_ROTATEFOLLOWER) ||
			(type == CAM_TYPE_FLATOSCILLATE) )
		{
				f = -f;			// y mirror
		}
	}

	res.z= 0;
	res.x = this->center.x + r*cos(f+phase);
	res.y = this->center.y + r*sin(f+phase);


	if (internal) Rr = -Rr; // restore Rr if sign inverted.
	if (negative) e = -e;	// restore e
}


void ChLineCam::Evaluate(Vector& pos, 
						const double parU, 
						const double parV, 
						const double parW) 
{
	double qtmp, gtmp;
	EvaluateCamPoint(parU, pos , gtmp, qtmp);
}




void ChLineCam::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLine::StreamOUT(mstream);

		// stream out all member data
	mstream << (int) type;
	mstream.AbstractWrite(law);
	mstream << phase;
	mstream << Rb;
	mstream << Rr;
	mstream << p;
	mstream << d;
	mstream << b0;
	mstream << e;
	mstream << s;
	mstream << negative;
	mstream << internal;		
	mstream << center;	
}

void ChLineCam::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLine::StreamIN(mstream);

		// stream in all member data
	int mint;
	mstream >> mint; type = (eChCamType)mint;
	if (law) delete law;
	mstream.AbstractReadCreate(&law);
	mstream >> phase;
	mstream >> Rb;
	mstream >> Rr;
	mstream >> p;
	mstream >> d;
	mstream >> b0;
	mstream >> e;
	mstream >> s;
	mstream >> negative;
	mstream >> internal;		
	mstream >> center;	
}




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

////// end
