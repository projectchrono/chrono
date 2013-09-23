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

//////////////////////////////////////////////////
//
//   ChCTriangle.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdio.h>


#include "ChCTriangle.h"

 

namespace chrono
{
namespace geometry
{
	
ChTriangle::ChTriangle() 
{
	p1=p2=p3= VNULL;
};
	
ChTriangle::ChTriangle(const ChVector<>& mp1, const ChVector<>& mp2, const ChVector<>& mp3) 
{
	p1= mp1; p2= mp2; p3= mp3;
}
	
ChTriangle::ChTriangle(const ChTriangle& source)
{
	Copy(&source);
}

ChTriangle::~ChTriangle() {};


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTriangle> a_registration_ChTriangle;




void ChTriangle::GetBoundingBox(double& xmin, double& xmax,
					    double& ymin, double& ymax,
						double& zmin, double& zmax,
						ChMatrix33<>* Rot)
{
	if (Rot==NULL)
	{
		xmin=ChMin(ChMin(p1.x, p2.x),p3.x);
		ymin=ChMin(ChMin(p1.y, p2.y),p3.y);
		zmin=ChMin(ChMin(p1.z, p2.z),p3.z);
		xmax=ChMax(ChMax(p1.x, p2.x),p3.x);
		ymax=ChMax(ChMax(p1.y, p2.y),p3.y);
		zmax=ChMax(ChMax(p1.z, p2.z),p3.z);
	}
	else
	{
		Vector trp1 = Rot->MatrT_x_Vect(p1);
		Vector trp2 = Rot->MatrT_x_Vect(p2);
		Vector trp3 = Rot->MatrT_x_Vect(p3);
		xmin=ChMin(ChMin(trp1.x, trp2.x),trp3.x);
		ymin=ChMin(ChMin(trp1.y, trp2.y),trp3.y);
		zmin=ChMin(ChMin(trp1.z, trp2.z),trp3.z);
		xmax=ChMax(ChMax(trp1.x, trp2.x),trp3.x);
		ymax=ChMax(ChMax(trp1.y, trp2.y),trp3.y);
		zmax=ChMax(ChMax(trp1.z, trp2.z),trp3.z);
	}
}

Vector ChTriangle::Baricenter()
{
	Vector mb;
	mb.x = (p1.x + p2.x + p3.x)/3.;
	mb.y = (p1.y + p2.y + p3.y)/3.;
	mb.z = (p1.z + p2.z + p3.z)/3.;
	return mb;
}

void ChTriangle::CovarianceMatrix(ChMatrix33<>& C)
{
	C(0,0)= p1.x*p1.x +
		    p2.x*p2.x +
			p3.x*p3.x;
	C(1,1)= p1.y*p1.y +
		    p2.y*p2.y +
			p3.y*p3.y;
	C(2,2)= p1.z*p1.z +
		    p2.z*p2.z +
			p3.z*p3.z;
	C(0,1)= p1.x*p1.y +
		    p2.x*p2.y +
			p3.x*p3.y;
	C(0,2)= p1.x*p1.z +
		    p2.x*p2.z +
			p3.x*p3.z;
	C(1,2)= p1.y*p1.z +
		    p2.y*p2.z +
			p3.y*p3.z;
};


bool ChTriangle::Normal(Vector& N)
{
    Vector u;
	u = Vsub(p2,p1);
    Vector v;
	v = Vsub(p3,p1);

	Vector n;
	n = Vcross(u,v);

	double len = Vlenght(n);

	if (fabs (len) > EPS_TRIDEGENERATE)
		N = Vmul(n, (1.0/len));
	else
		return false;

	return true;
};


bool ChTriangle::IsDegenerated()
{
	Vector u = Vsub(p2,p1);
    Vector v = Vsub(p3,p1);

	Vector vcr;
	vcr = Vcross(u,v);
	if (fabs(vcr.x) < EPS_TRIDEGENERATE && fabs(vcr.y) < EPS_TRIDEGENERATE && fabs(vcr.z) < EPS_TRIDEGENERATE )
		return true;
	return false;
}




double ChTriangle::PointTriangleDistance(Vector B,
							   Vector& A1,			///< point of triangle
							   Vector& A2,			///< point of triangle
							   Vector& A3,			///< point of triangle
							   double& mu,
							   double& mv,
							   bool& is_into,
							   Vector& Bprojected)
{
	// defaults
	is_into = false;
	mu=mv=-1;
	double mdistance = 10e22;

	Vector Dx, Dy, Dz, T1, T1p;

	Dx= Vsub (A2, A1);
	Dz= Vsub (A3, A1);
	Dy= Vcross(Dz,Dx);

	double dylen = Vlenght(Dy);

	if(fabs(dylen)<EPS_TRIDEGENERATE)	// degenere triangle
		return mdistance;

	Dy= Vmul(Dy,1.0/dylen);

	ChMatrix33<> mA;
	ChMatrix33<> mAi;
	mA.Set_A_axis(Dx,Dy,Dz);

	// invert triangle coordinate matrix -if singular matrix, was degenerate triangle-.
	if (fabs(mA.FastInvert(&mAi)) <0.000001)
		return mdistance;

	T1 = mAi.Matr_x_Vect( Vsub (B, A1) );
	T1p = T1;
	T1p.y=0;
	mu = T1.x;
	mv = T1.z;
	if (mu >=0  &&  mv >=0  &&  mv<=1.0-mu)
	{
		is_into = true;
		mdistance = fabs(T1.y);
		Bprojected = Vadd(A1, mA.Matr_x_Vect(T1p));
	}

	return mdistance;
}





double ChTriangle::PointLineDistance(Vector& p, Vector& dA, Vector& dB, double& mu, bool& is_insegment)
{
	mu=-1.0;
	is_insegment = 0;
	double mdist=10e34;

	Vector vseg = Vsub(dB,dA);
	Vector vdir = Vnorm(vseg);
	Vector vray = Vsub(p,dA);

	mdist = Vlenght(Vcross(vray,vdir));
	mu = Vdot(vray,vdir)/Vlenght(vseg);

	if ((mu>=0) && (mu<=1.0))
		is_insegment = 1;

	return mdist;
}



void ChTriangle::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChGeometry::StreamOUT(mstream);

		// stream out all member data
	mstream << p1;
	mstream << p2;
	mstream << p3;
}

void ChTriangle::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChGeometry::StreamIN(mstream);

		// stream in all member data
	mstream >> p1;
	mstream >> p2;
	mstream >> p3;
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
