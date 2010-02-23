///////////////////////////////////////////////////
//
//   ChCLinePoly.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


 
#include "ChCLinePoly.h"


namespace chrono
{
namespace geometry
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinePoly> a_registration_ChLinePoly;



//
// CLASS FOR POLY LINE
//
// The object which represent 3d lines/splines  
//



ChLinePoly::ChLinePoly (int mnumpoints)
{
	closed= 0;
	numpoints = mnumpoints;
	points = new Vector [mnumpoints];
	int degree = 1;
}

ChLinePoly::~ChLinePoly ()
{
	if (points)
		delete[] points;
}

void ChLinePoly::Copy (const ChLinePoly* source) 
{
	// Copy parent data;
	ChLine::Copy(source);

	// Copy custom data;
	numpoints = source->numpoints;
	degree = source->degree;
	if (points) 
		delete[] points;
	points = new Vector[numpoints];
	memcpy (points, source->points, (sizeof(Vector) * numpoints));
};

int ChLinePoly::Get_closed()
{
	return closed;
}

void ChLinePoly::Set_closed(int mc)
{
	closed = mc;
}

int ChLinePoly::Get_numpoints()
{
	return numpoints;
}

int	ChLinePoly::Get_degree()
{
	return degree;
}

Vector ChLinePoly::Get_point(int mnum)
{
	if (mnum>=Get_numpoints()) return VNULL;

	return this->points[mnum];
}

int ChLinePoly::Set_point (int mnum, Vector mpoint)
{
	if (mnum>=Get_numpoints()) return FALSE;

	this->points[mnum] = mpoint;

	return TRUE;
}



//
// Curve evaluation.
//


void ChLinePoly::Evaluate(Vector& pos, 
						const double parU, 
						const double parV, 
						const double parW) 
{
	double par = parU;
	pos = VNULL;

	if (par<0) par=0;
	if (par>1) par=1;
	int pA= 0;
	int pB= 0;
	double epar;
	if (!closed)
		epar = par*(Get_numpoints()-1);
	else
		epar = par*Get_numpoints();
	pA = (int)floor(epar);
	pB = (int)ceil(epar);
	if (pA<0)
		pA=0;
	if (pA>=(Get_numpoints()-1))
		pA=(Get_numpoints()-1);
	if (pB>=Get_numpoints())
	{
		if (!closed)
			pB=(Get_numpoints()-1);
		else
			pB=0;
	}
		// linear interpolation
	pos = Vadd (Vmul (Get_point(pA), 1-(epar-(double)pA)),
		  Vmul (Get_point(pB),    epar-(double)pA));
}


double ChLinePoly::Lenght (int sampling)
{
	return ChLine::Lenght(1);
}



// Draw into the current graph viewport of a ChFile_ps file

int ChLinePoly::DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate)
{
	ChPageVect mp1;
	Vector mv1;

	mfle->GrSave();
	mfle->ClipRectangle(mfle->Get_G_p(), mfle->Get_Gs_p(), PS_SPACE_PAGE);
				// start a line, move cursor to beginning
	mfle->StartLine();
	mp1.x = Get_point(0).x;	mp1.y = Get_point(0).y;
	mp1 = mfle->To_page_from_graph(mp1);
	mfle->MoveTo(mp1);
				// add points into line
	for (int i = 1; i < this->Get_numpoints(); i++)
	{
		mv1 = Get_point(i);
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



void ChLinePoly::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLine::StreamOUT(mstream);

		// stream out all member data
	mstream << numpoints;
	mstream << degree;

	for (int i=0; i<numpoints; i++)
		mstream << points[i];
}

void ChLinePoly::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLine::StreamIN(mstream);

		// stream in all member data
	mstream >> numpoints;
	mstream >> degree;
	
	if (points) delete[] points;
	points = new Vector[numpoints];
	for (int i=0; i<numpoints; i++)
		mstream >> points[i];
}




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

////// end

