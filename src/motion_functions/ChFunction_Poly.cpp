///////////////////////////////////////////////////
//
//   ChFunction_Poly.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Poly.h"


namespace chrono
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Poly> a_registration_poly;


ChFunction_Poly::ChFunction_Poly ()
{
	order = 0;
	for (int i=0; i< POLY_COEFF_ARRAY; i++)
		{coeff[i]=0 ;};
}

void ChFunction_Poly::Copy (ChFunction_Poly* source)
{
	order = source->order;
	for (int i = 0; i< POLY_COEFF_ARRAY; i++)
	{
		Set_coeff (source->Get_coeff(i), i);
	}
}

ChFunction* ChFunction_Poly::new_Duplicate ()
{
	ChFunction_Poly* m_func;
	m_func = new ChFunction_Poly;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Poly::Get_y      (double x)
{
	double total = 0;
	int i;
	for (i=0; i<=order; i++)
	{
		total += (coeff[i] * pow(x, (double)i));
	}
	return total;
}

double ChFunction_Poly::Get_y_dx   (double x)
{
	double total = 0;
	int i;

	if (order < 1) return 0; // constant function

	for (i=1; i<=order; i++)
	{
		total += ( (double)i * coeff[i] * pow(x, ((double)(i-1))) );
	}
	return total;
}

double ChFunction_Poly::Get_y_dxdx (double x)
{
	double total = 0;
	int i;

	if (order < 2) return 0; // constant function

	for (i=2; i<=order; i++)
	{
		total += ( (double)(i*(i-1)) * coeff[i] * pow(x, ((double)(i-2))) );
	}
	return total;
}

int ChFunction_Poly::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// add variables to own tree
	char msubvar[50];
	for (int mord=0; mord<this->order; mord++)
	{
		sprintf(msubvar,"C%d", mord);

		chjs_propdata* mdataA = new chjs_propdata;
		strcpy(mdataA->propname, msubvar);
		strcpy(mdataA->label,    mdataA->propname);
		mdataA->haschildren = FALSE;
		mtree->AddTail(mdataA);
		i++;
	}

	return i;
}

void ChFunction_Poly::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << this->order;
	for (int i= 0; i<= order; i++)
	{
		mstream << Get_coeff(i);
	}

}

void ChFunction_Poly::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	int ifoo;
	double dfoo;
	mstream >> ifoo;		this->Set_order(ifoo);
	for (int i= 0; i<= order; i++)
	{
		mstream >> dfoo;
		Set_coeff(dfoo,i);
	}
}

void ChFunction_Poly::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_POLY  \n";

	//***TO DO***
}





} // END_OF_NAMESPACE____


// eof
