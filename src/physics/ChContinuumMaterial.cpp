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



void ChContinuumMaterial::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// stream out all member data
	mstream << this->density;
}

void ChContinuumMaterial::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in all member data
	mstream >> this->density;
}




////////////////////////////////////////




ChContinuumElastic::ChContinuumElastic(double myoung, double mpoisson, double mdensity) 
		: ChContinuumMaterial(mdensity)
{
	E = myoung;
	Set_v(mpoisson); // sets also G and l
}

ChContinuumElastic::~ChContinuumElastic()
{
}

void ChContinuumElastic::Set_E (double m_E)
{
	E = m_E;
	G = E/(2*(1+v));	// fixed v, E, get G
	l = (v*E)/((1+v)*(1-2*v));	// Lame's constant l
}

void ChContinuumElastic::Set_v (double m_v)
{
	v = m_v;
	G = E/(2*(1+v));	// fixed v, E, get G
	l = (v*E)/((1+v)*(1-2*v));	// Lame's constant l
}

void ChContinuumElastic::Set_G (double m_G)
{
	G = m_G;
	v = (E/(2*G))-1;	// fixed G, E, get v
	l = (v*E)/((1+v)*(1-2*v)); // Lame's constant l
}

void ChContinuumElastic::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

			// stream in parent class
	ChContinuumMaterial::StreamOUT(mstream);

		// stream out all member data
	mstream << this->E;
	mstream << this->v;
	mstream << this->G;
	mstream << this->l;
}

void ChContinuumElastic::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in parent class
	ChContinuumMaterial::StreamIN(mstream);

		// stream in all member data
	mstream >> this->E;
	mstream >> this->v;
	mstream >> this->G;
	mstream >> this->l;
}



///////////////////////////////
//////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


ChContinuumPlasticVonMises::ChContinuumPlasticVonMises(double myoung, double mpoisson, double mdensity,
												   double melastic_yeld, double  mplastic_yeld) : 
							ChContinuumElastoplastic(myoung, mpoisson, mdensity)
{
	elastic_yeld = melastic_yeld;
	plastic_yeld = mplastic_yeld;
	flow_rate = 1;
}

double ChContinuumPlasticVonMises::ComputeYeldFunction(const ChStressTensor<>& mstress) const
{
	return (mstress.GetEquivalentVonMises() - this->elastic_yeld);
}

void ChContinuumPlasticVonMises::ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow, 
									const ChStrainTensor<>&	mincrementstrain, 
									const ChStrainTensor<>& mlastelasticstrain,
									const ChStrainTensor<>& mlastplasticstrain) const
{
	ChStrainTensor<> guesselstrain(mlastelasticstrain);
	guesselstrain.MatrInc(mincrementstrain); // assume increment is all elastic

	double vonm = guesselstrain.GetEquivalentVonMises();
	if (vonm > this->elastic_yeld)
	{
		ChVoightTensor<> mdev;
		guesselstrain.GetDeviatoricPart(mdev);
		mplasticstrainflow.CopyFromMatrix(mdev * ((vonm - this->elastic_yeld)/(vonm)));
	}
	else
	{
		mplasticstrainflow.FillElem(0);
	}
}


void ChContinuumPlasticVonMises::ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow, const ChStrainTensor<>& mtotstrain) const
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

void ChContinuumPlasticVonMises::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// stream out parent class
	ChContinuumElastoplastic::StreamOUT(mstream);

		// stream out all member data
	mstream << this->elastic_yeld;
	mstream << this->plastic_yeld;
	mstream << this->flow_rate;
}

void ChContinuumPlasticVonMises::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in parent class
	ChContinuumElastoplastic::StreamIN(mstream);

		// stream in all member data
	mstream >> this->elastic_yeld;
	mstream >> this->plastic_yeld;
	mstream >> this->flow_rate;
}




///////////////////////////////
//////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


ChContinuumDruckerPrager::ChContinuumDruckerPrager(double myoung, double mpoisson, double mdensity,
							double melastic_yeld, double  malpha, double mdilatancy) : 
						ChContinuumElastoplastic(myoung, mpoisson, mdensity)
{
	elastic_yeld = melastic_yeld;
	alpha = malpha;
	dilatancy = mdilatancy;
	hardening_limit = elastic_yeld;
	hardening_speed =0;
	flow_rate = 1;
}

void ChContinuumDruckerPrager::Set_from_MohrCoulomb(double phi, double cohesion, bool inner_approx)
{
	if (inner_approx)
	{
		alpha =			(2*sin(phi))/(sqrt(3.0)*(3.0-sin(phi)));
		elastic_yeld =  (6*cohesion*cos(phi))/(sqrt(3.0)*(3.0-sin(phi)));
	} else
	{
		alpha =			(2*sin(phi))/(sqrt(3.0)*(3.0+sin(phi)));
		elastic_yeld =  (6*cohesion*cos(phi))/(sqrt(3.0)*(3.0+sin(phi)));
	}
}

double ChContinuumDruckerPrager::ComputeYeldFunction(const ChStressTensor<>& mstress) const
{
	return (mstress.GetInvariant_I1() * this->alpha + sqrt (mstress.GetInvariant_J2()) - this->elastic_yeld);
}

void ChContinuumDruckerPrager::ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow, 
									const ChStrainTensor<>&	mincrementstrain, 
									const ChStrainTensor<>& mlastelasticstrain,
									const ChStrainTensor<>& mlastplasticstrain) const
{
	ChStrainTensor<> guesselstrain(mlastelasticstrain);
	guesselstrain.MatrInc(mincrementstrain); // assume increment is all elastic
	
	ChStressTensor<> mstress;
	this->ComputeElasticStress(mstress,guesselstrain);
	double fprager = mstress.GetInvariant_I1() * this->alpha + sqrt (mstress.GetInvariant_J2())  - this->elastic_yeld;
	if (fprager >0 )
	{
		ChStrainTensor<> dFdS;
		ChStrainTensor<> dGdS;
		double devsq = sqrt(mstress.GetInvariant_J2());
		if (devsq > 10e-20)
		{
			double sixdevsq = 6 * devsq;

			dFdS.XX() = this->alpha + (2* mstress.XX() - mstress.YY()   - mstress.ZZ() )/sixdevsq;
			dFdS.YY() = this->alpha + (- mstress.XX() + 2*mstress.YY()  - mstress.ZZ() )/sixdevsq;
			dFdS.ZZ() = this->alpha + (- mstress.XX() -  mstress.YY() + 2*mstress.ZZ() )/sixdevsq;
			dFdS.XY() = mstress.XY()/devsq;
			dFdS.YZ() = mstress.YZ()/devsq;
			dFdS.XZ() = mstress.XZ()/devsq;

			dGdS.XX() = this->dilatancy + (2* mstress.XX() - mstress.YY()   - mstress.ZZ() )/sixdevsq;
			dGdS.YY() = this->dilatancy + (- mstress.XX() + 2*mstress.YY()  - mstress.ZZ() )/sixdevsq;
			dGdS.ZZ() = this->dilatancy + (- mstress.XX() -  mstress.YY() + 2*mstress.ZZ() )/sixdevsq;
			dGdS.XY() = mstress.XY()/devsq;
			dGdS.YZ() = mstress.YZ()/devsq;
			dGdS.XZ() = mstress.XZ()/devsq;
		}
		else
		{
			//GetLog() << "Singular! devsq=" << devsq << "  ";
			// singularity for pure hydrostatic stress
			dFdS.FillElem(0); 
			dFdS.XX() = 1; dFdS.YY() = 1; dFdS.ZZ() = 1;
			dGdS.FillElem(0); 
			dGdS.XX() = 1; dGdS.YY() = 1; dGdS.ZZ() = 1;
		}
		ChStressTensor<> aux_dFdS_C;
		this->ComputeElasticStress(aux_dFdS_C, dFdS);

		ChMatrixNM<double,1,1> inner_up;
		inner_up.MatrTMultiply(aux_dFdS_C, mincrementstrain);
		ChMatrixNM<double,1,1> inner_dw;
		inner_dw.MatrTMultiply(aux_dFdS_C, dGdS);

		mplasticstrainflow.CopyFromMatrix(dGdS);
		mplasticstrainflow.MatrScale(inner_up(0)/inner_dw(0));
		//GetLog() << "scale " << inner_up(0)/inner_dw(0) << "  Fyeld " << fprager <<  "\n";
	} 
	else
	{
		mplasticstrainflow.FillElem(0);
	}
}

//***OBSOLETE***
void ChContinuumDruckerPrager::ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow, const ChStrainTensor<>& mestrain) const
{
	ChStressTensor<> mstress;
	this->ComputeElasticStress(mstress,mestrain);
	double prager = mstress.GetInvariant_I1() * this->alpha + sqrt (mstress.GetInvariant_J2());
	if (prager > this->elastic_yeld)
	{
		ChVoightTensor<> mdev;
		mstress.GetDeviatoricPart(mdev);
		double divisor = 2.*sqrt(mstress.GetInvariant_J2());
		if (divisor>10e-20)
			mdev.MatrScale(1./ divisor );
		mdev.XX()+=this->dilatancy;
		mdev.YY()+=this->dilatancy;
		mdev.ZZ()+=this->dilatancy;
		mplasticstrainflow.CopyFromMatrix(mdev);
	}
	else
	{
		mplasticstrainflow.FillElem(0);
	}
}

void ChContinuumDruckerPrager::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// stream out parent class
	ChContinuumElastic::StreamOUT(mstream);

		// stream out all member data
	mstream << this->elastic_yeld;
	mstream << this->alpha;
	mstream << this->dilatancy;
	mstream << this->hardening_speed;
	mstream << this->hardening_limit;
	mstream << this->flow_rate;
}

void ChContinuumDruckerPrager::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in parent class
	ChContinuumElastic::StreamIN(mstream);

		// stream in all member data
	mstream >> this->elastic_yeld;
	mstream >> this->alpha;
	mstream >> this->dilatancy;
	mstream >> this->hardening_speed;
	mstream >> this->hardening_limit;
	mstream >> this->flow_rate;
}







} // END_OF_NAMESPACE____

