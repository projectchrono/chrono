//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: Alessandro Tasora


#include "ChElementGeneric.h"


namespace chrono
{
namespace fem
{

void ChElementGeneric::EleIntLoadResidual_F(ChVectorDynamic<>& R, const double c )
{
	ChMatrixDynamic<> mFi(this->GetNdofs(), 1);
	this->ComputeInternalForces(mFi);
	//GetLog() << "EleIntLoadResidual_F , mFi=" << mFi << "  c=" << c << "\n";
	mFi.MatrScale(c);
	int stride = 0;
	for (int in=0; in < this->GetNnodes(); in++)
	{
		int nodedofs = GetNodeN(in)->Get_ndof_w();
		//GetLog() << "  in=" << in << "  stride=" << stride << "  nodedofs=" << nodedofs << " offset=" <<  GetNodeN(in)->NodeGetOffset_w() << "\n";
		if (!GetNodeN(in)->GetFixed())
			R.PasteSumClippedMatrix(&mFi, stride, 0, nodedofs,1, GetNodeN(in)->NodeGetOffset_w(), 0);
		stride += nodedofs;
	}
	//GetLog() << "EleIntLoadResidual_F , R=" << R << "\n";
}

void ChElementGeneric::EleIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c)
{
	// This is a default (VERY UNOPTIMAL) book keeping so that in children classes you can avoid 
	// implementing this VariablesFbIncrementMq function, unless you need faster code)

	ChMatrixDynamic<> mMi(this->GetNdofs(), this->GetNdofs());
	this->ComputeKRMmatricesGlobal(mMi, 0, 0, 1.0); // fill M mass matrix 
	
	ChMatrixDynamic<> mqi(this->GetNdofs(), 1);
	int stride = 0;
	for (int in=0; in < this->GetNnodes(); in++)
	{
		int nodedofs = GetNodeN(in)->Get_ndof_w();
		if (!GetNodeN(in)->GetFixed())
			mqi.PasteClippedMatrix(&w, GetNodeN(in)->NodeGetOffset_w(), 0, nodedofs, 1, stride,0);
		stride += nodedofs;
	}

	ChMatrixDynamic<> mFi(this->GetNdofs(), 1);
	mFi.MatrMultiply(mMi, mqi);
	mFi.MatrScale(c);

	stride = 0;
	for (int in=0; in < this->GetNnodes(); in++)
	{
		int nodedofs = GetNodeN(in)->Get_ndof_w();
		if (!GetNodeN(in)->GetFixed())
			R.PasteSumClippedMatrix(&mFi, stride,0, nodedofs,1, GetNodeN(in)->NodeGetOffset_w(),0);
		stride += nodedofs;
	}
}


void ChElementGeneric::VariablesFbLoadInternalForces(double factor) 
{
	
	ChMatrixDynamic<> mFi(this->GetNdofs(), 1);
	this->ComputeInternalForces(mFi);
	mFi.MatrScale(factor);
	int stride = 0;
	for (int in=0; in < this->GetNnodes(); in++)
	{
		int nodedofs = GetNodeN(in)->Get_ndof_w();
		GetNodeN(in)->Variables().Get_fb().PasteSumClippedMatrix(&mFi, stride,0, nodedofs,1, 0,0);
		stride += nodedofs;
	}
};



void ChElementGeneric::VariablesFbIncrementMq() 
{
	// This is a default (VERY UNOPTIMAL) book keeping so that in children classes you can avoid 
	// implementing this VariablesFbIncrementMq function, unless you need faster code)

	ChMatrixDynamic<> mMi(this->GetNdofs(), this->GetNdofs());
	this->ComputeKRMmatricesGlobal(mMi, 0, 0, 1.0); // fill M mass matrix 
	
	ChMatrixDynamic<> mqi(this->GetNdofs(), 1);
	int stride = 0;
	for (int in=0; in < this->GetNnodes(); in++)
	{
		int nodedofs = GetNodeN(in)->Get_ndof_w();
		mqi.PasteMatrix(&GetNodeN(in)->Variables().Get_qb(), stride, 0);
		stride += nodedofs;
	}

	ChMatrixDynamic<> mFi(this->GetNdofs(), 1);
	mFi.MatrMultiply(mMi, mqi);

	stride = 0;
	for (int in=0; in < this->GetNnodes(); in++)
	{
		int nodedofs = GetNodeN(in)->Get_ndof_w();
		GetNodeN(in)->Variables().Get_fb().PasteSumClippedMatrix(&mFi, stride,0, nodedofs,1, 0,0);
		stride += nodedofs;
	}

}





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____








