///////////////////////////////////////////////////
//
//   ChContinuumMaterial.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChContinuumMaterial.h"

namespace chrono 
{


ChContinuumMaterial::ChContinuumMaterial(double myoung, double mpoisson, double mdensity)
{
	E = myoung;
	Set_v(mpoisson); // sets also G and l
	density = mdensity; 
}

ChContinuumMaterial::~ChContinuumMaterial()
{
}

void ChContinuumMaterial::Set_E (double m_E)
{
	E = m_E;
	G = E/(2*(1+v));	// fixed v, E, get G
	l = (v*E)/((1+v)*(1-2*v));	// Lame's constant l
}

void ChContinuumMaterial::Set_v (double m_v)
{
	v = m_v;
	G = E/(2*(1+v));	// fixed v, E, get G
	l = (v*E)/((1+v)*(1-2*v));	// Lame's constant l
}

void ChContinuumMaterial::Set_G (double m_G)
{
	G = m_G;
	v = (E/(2*G))-1;	// fixed G, E, get v
	l = (v*E)/((1+v)*(1-2*v)); // Lame's constant l
}



//////// FILE I/O

void ChContinuumMaterial::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// stream out all member data
	mstream << this->E;
	mstream << this->v;
	mstream << this->G;
	mstream << this->l;
	mstream << this->density;
}

void ChContinuumMaterial::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in all member data
	mstream >> this->E;
	mstream >> this->v;
	mstream >> this->G;
	mstream >> this->l;
	mstream >> this->density;
}



///////////////////////////////


ChContinuumElastoplastic::ChContinuumElastoplastic(double myoung, double mpoisson, double mdensity,
												   double melastic_yeld, double  mplastic_yeld) : 
							ChContinuumMaterial(myoung, mpoisson, mdensity)
{
	elastic_yeld = melastic_yeld;
	plastic_yeld = mplastic_yeld;
	flow_rate = 1;
}


void ChContinuumElastoplastic::ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow, const ChStrainTensor<>& mtotstrain) const
{
	double vonm = mtotstrain.GetEquivalentVonMises();
	if (vonm > this->elastic_yeld)
	{
		ChVoightTensor<> mdev;
		mtotstrain.GetDeviatoricPart(mdev);
		mplasticstrainflow.CopyFromMatrix(mdev * ((vonm - this->elastic_yeld)/(vonm)));
	}
	else
	{
		mplasticstrainflow.FillElem(0);
	}
}

void ChContinuumElastoplastic::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// stream out parent class
	ChContinuumMaterial::StreamOUT(mstream);

		// stream out all member data
	mstream << this->elastic_yeld;
	mstream << this->plastic_yeld;
	mstream << this->flow_rate;
}

void ChContinuumElastoplastic::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream out parent class
	ChContinuumMaterial::StreamIN(mstream);

		// stream in all member data
	mstream >> this->elastic_yeld;
	mstream >> this->plastic_yeld;
	mstream >> this->flow_rate;
}







} // END_OF_NAMESPACE____

