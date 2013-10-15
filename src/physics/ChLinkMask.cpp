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

///////////////////////////////////////////////////
//
//   ChLinkMask.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include "physics/ChLinkMask.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

 

namespace chrono
{



//////////////////////////////////////
// CLASS "LinkMask"


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMask> a_registration_ChLinkMask;


ChLinkMask::ChLinkMask()
{
    nconstr = 1;

	constraints.resize(1);
	constraints[0] = new ChLcpConstraintTwoBodies;
}

ChLinkMask::ChLinkMask(int mnconstr)
{

    nconstr = mnconstr;

	constraints.resize(nconstr);
	for (int i=0; i<nconstr; i++)
		constraints[i] = new ChLcpConstraintTwoBodies;

}

ChLinkMask::~ChLinkMask()
{
    for (int i=0; i<nconstr; i++)
	{
		if (constraints[i])
			delete constraints[i];
	}
}

ChLinkMask::ChLinkMask (ChLinkMask& source)
{
    nconstr = 0;

	nconstr = source.nconstr;
    constraints.resize(source.nconstr);
	for (int i=0; i<nconstr; i++)
		constraints[i] = (ChLcpConstraintTwoBodies*) source.Constr_N(i).new_Duplicate();
}


void ChLinkMask::ResetNconstr(int newnconstr)
{

    if (nconstr != newnconstr)
    {
        int i;
		for (i=0; i<nconstr; i++)
			if (constraints[i]) delete constraints[i];

		nconstr = newnconstr;

		constraints.resize(nconstr);

		for (i=0; i<nconstr; i++)
			constraints[i] = new ChLcpConstraintTwoBodies;

    }

}

void ChLinkMask::AddConstraint(ChLcpConstraintTwoBodies* aconstr)
{
	nconstr++;
	constraints.push_back(aconstr);
}

void ChLinkMask::Copy(ChLinkMask* source)
{
    int i;
	for (i=0; i<nconstr; i++)
			if (constraints[i]) delete constraints[i];

	nconstr = source->nconstr;

	constraints.resize(nconstr);

	for (i=0; i<nconstr; i++)
		constraints[i] = (ChLcpConstraintTwoBodies*) source->constraints[i]->new_Duplicate();
}

ChLinkMask* ChLinkMask::NewDuplicate()
{
    ChLinkMask* mm = new ChLinkMask();
    mm->Copy(this);
    return mm;
}

void ChLinkMask::SetTwoBodiesVariables(ChLcpVariables* var1, ChLcpVariables* var2)
{
	for (int i=0; i<nconstr; i++)
		constraints[i]->SetVariables(var1, var2);
}

int ChLinkMask::IsEqual(ChLinkMask& mask2)
{
	if (nconstr != mask2.nconstr) return FALSE;
    for (int j= 0; j < nconstr; j++)
    {
        if (!( Constr_N(j) == mask2.Constr_N(j)) ) return FALSE;
    }
    return TRUE;
}

bool ChLinkMask::IsUnilateral(int i)
{
	if( Constr_N(i).IsUnilateral() )
	   return true;
	return false;
}

int ChLinkMask::GetMaskDoc()
{
	int tot = 0;
	for (int j= 0; j < nconstr; j++)
    {
        if ( Constr_N(j).IsActive()) tot++;
    }
    return tot;
}

int ChLinkMask::GetMaskDoc_d()
{
    int cnt = 0;
    for (int i= 0; i < nconstr; i++)
    {
		if ( Constr_N(i).IsActive())
			if(this->IsUnilateral(i))    // if (Constr_N(i).IsUnilateral())  BETTER?
				cnt++;
    }
    return cnt;
}

int ChLinkMask::GetMaskDoc_c()
{
    return (GetMaskDoc()-GetMaskDoc_d());
}

ChLcpConstraintTwoBodies* ChLinkMask::GetActiveConstrByNum(int mnum)
{
	int cnt = 0;
	for (int i= 0; i < nconstr; i++)
    {
        if ( Constr_N(i).IsActive())
        {
            if (cnt == mnum)
				return &Constr_N(i);
            cnt++;
        }
    }
    return NULL;
}

int ChLinkMask::SetActiveRedundantByArray(int* mvector, int mcount)
{
    int cnt;

    ChLinkMask* newmask;
	newmask = this->NewDuplicate();
    for (int elem = 0; elem <mcount; elem++)
    {
        cnt = 0;
        for (int i= 0; i < nconstr; i++)
        {
            if ( Constr_N(i).IsActive() )
            {
                if (cnt == mvector[elem]) newmask->Constr_N(i).SetRedundant(true);
                cnt++;
            }
        }
    }
    this->Copy(newmask); // replace the mask with updated one.
    return mcount;

}



        // set lock =ON for costraints which were disabled because redundant
int ChLinkMask::RestoreRedundant()
{
	int tot = 0;
    for (int j= 0; j < nconstr; j++)
    {
		if (Constr_N(j).IsRedundant())
		{
			Constr_N(j).SetRedundant(false);
			tot++;
		}
    }
    return tot;
}

int ChLinkMask::SetAllDisabled(bool mdis)
{
    int cnt = 0;

	for (int i= 0; i < nconstr; i++)
    {
		if (Constr_N(i).IsDisabled()!=mdis)
		{
			Constr_N(i).SetDisabled(mdis);
			cnt++;
		}
	}

    return cnt;
}


int ChLinkMask::SetAllBroken(bool mdis)
{
    int cnt = 0;

	for (int i= 0; i < nconstr; i++)
    {
		if (Constr_N(i).IsBroken()!=mdis)
		{
			Constr_N(i).SetBroken(mdis);
			cnt++;
		}
	}

    return cnt;
}


void ChLinkMask::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(3);

		// stream out all member data
	mstream << nconstr;

	for (int i = 0; i < this->nconstr; i++)
    {
        mstream.AbstractWrite(this->constraints[i]);    // save the constraint states
    }
}



void ChLinkMask::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in all member data
	int my_nconstr;
	mstream >> my_nconstr;

	if (version==1)
	{
		this->ResetNconstr(my_nconstr);
		for (int i = 0; i < nconstr; i++)
		{
			int loadflag;
			mstream >> loadflag;
			if (loadflag==4)
				Constr_N(i).SetRedundant(true);
			if (loadflag==5)
				Constr_N(i).SetBroken(true);
			if (loadflag==6)
				Constr_N(i).SetDisabled(true);
			if (loadflag==7)
				Constr_N(i).SetRedundant(true);
			if (loadflag==8)
				Constr_N(i).SetDisabled(true);
			if (loadflag==20)
				Constr_N(i).SetMode(CONSTRAINT_FREE);
			if (loadflag==30)
				Constr_N(i).SetMode(CONSTRAINT_LOCK);
			if (loadflag==33 || loadflag==34)
				Constr_N(i).SetMode(CONSTRAINT_UNILATERAL);
		}
	}
  
	if (version==2)
	{
        int i;
		for (i=0; i<nconstr; i++)
			if (constraints[i]) delete constraints[i];
		nconstr = my_nconstr;
		constraints.resize(nconstr);
		for (i = 0; i < this->nconstr; i++)
		{
			std::string cls_name;
			mstream >> cls_name;
			if (cls_name=="ChLcpConstraintTwo")
				constraints[i] = new ChLcpConstraintTwoBodies();	// because in v3 ChLcpConstraintTwo is pure virtual class
			else
				GetLog() << "  ERROR unknown object in v2 mask:" << cls_name.c_str() <<"\n";
			constraints[i]->ChLcpConstraintTwo::StreamIN(mstream);
		}
	}

	if (version>=3)
	{
        int i;
		for (i=0; i<nconstr; i++)
			if (constraints[i]) delete constraints[i];
		nconstr = my_nconstr;
		constraints.resize(nconstr);
		for (i = 0; i < this->nconstr; i++)
		{
			mstream.AbstractReadCreate(&this->constraints[i]);    // load the constraint states
		}
	}

}




///////////////////////////////////////////////////
// Lock formulation LF link mask:
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMaskLF> a_registration_ChLinkMaskLF;



ChLinkMaskLF::ChLinkMaskLF()
{
    ResetNconstr(9);    // the LF formulation uses 9 constraint flags
}

void ChLinkMaskLF::Copy(ChLinkMaskLF* source)
{
    ChLinkMask::Copy(source);  // first, inherit
}

ChLinkMask* ChLinkMaskLF::NewDuplicate()
{
    ChLinkMaskLF* mm = new ChLinkMaskLF();
    mm->Copy(this);
    return mm;
}


        // temporarely set OFF the locked costraints, because of 2D mode, and viceversa
int ChLinkMaskLF::Set2Dmode(int mode)
{
    int cnt;

	if (mode == 1) // switching to 2D
    {
        cnt = 0;
        if (Constr_Z().IsDisabled()==false) {Constr_Z().SetDisabled(true); cnt++;}
		if (Constr_E1().IsDisabled()==false) {Constr_E1().SetDisabled(true); cnt++;}
		if (Constr_E2().IsDisabled()==false) {Constr_E2().SetDisabled(true); cnt++;}
        return cnt;
    }
    if (mode == 0) // switching back to 3D
    {
        cnt = 0;
        if (Constr_Z().IsDisabled()==true) {Constr_Z().SetDisabled(false); cnt++;}
		if (Constr_E1().IsDisabled()==true) {Constr_E1().SetDisabled(false); cnt++;}
		if (Constr_E2().IsDisabled()==true) {Constr_E2().SetDisabled(false); cnt++;}
        return cnt;
    }
    return 0;

}

void ChLinkMaskLF::SetLockMask(int x, int y, int z,
                       int e0, int e1, int e2, int e3,
                       int p, int d)
{
	if (x)
	{
		this->Constr_X().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_X().SetMode(CONSTRAINT_FREE);
	}

	if (y)
	{
		this->Constr_Y().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_Y().SetMode(CONSTRAINT_FREE);
	}

	if (z)
	{
		this->Constr_Z().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_Z().SetMode(CONSTRAINT_FREE);
	}

	if (e0)
	{
		this->Constr_E0().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_E0().SetMode(CONSTRAINT_FREE);
	}

	if (e1)
	{
		this->Constr_E1().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_E1().SetMode(CONSTRAINT_FREE);
	}

	if (e2)
	{
		this->Constr_E2().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_E2().SetMode(CONSTRAINT_FREE);
	}

	if (e3)
	{
		this->Constr_E3().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_E3().SetMode(CONSTRAINT_FREE);
	}

	if (p)
	{
		this->Constr_P().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_P().SetMode(CONSTRAINT_FREE);
	}

	if (d)
	{
		this->Constr_D().SetMode(CONSTRAINT_LOCK);
	}else
	{
		this->Constr_D().SetMode(CONSTRAINT_FREE);
	}
}




void ChLinkMaskLF::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkMask::StreamOUT(mstream);

		// stream out all member data

}

void ChLinkMaskLF::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkMask::StreamIN(mstream);

		// stream in all member data
}



} // END_OF_NAMESPACE____


