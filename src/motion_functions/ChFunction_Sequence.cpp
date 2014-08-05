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
//   ChFunction_Sequence.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Sequence.h"
#include "ChFunction_Fillet3.h"


namespace chrono
{


ChFseqNode::ChFseqNode(ChSharedPtr<ChFunction> myfx, double mdur)
{
	fx = myfx;
	duration = mdur;
	weight = 1;
	t_start = 0; t_end = t_start+ duration;
	Iy = Iydt = Iydtdt = 0.0;
	y_cont = ydt_cont = ydtdt_cont = FALSE;
}

ChFseqNode::~ChFseqNode()
{
	// no need to delete wrapped function, it is handled with shared pointer
}

void ChFseqNode::Copy(ChFseqNode* source)
{
	// fx = source->fx;		//***? shallow copy (now sharing same object)...
	fx = ChSharedPtr<ChFunction>(source->fx->new_Duplicate());	//***? ..or deep copy? make optional with flag?
	duration = source->duration;
	weight = source->weight;
	t_start = source->t_start;
	t_end = source->t_end;
	Iy = source->Iy;
	Iydt = source->Iydt;
	Iydtdt = source->Iydtdt;
	y_cont = source->y_cont;
	ydt_cont = source->ydt_cont;
	ydtdt_cont = source->ydtdt_cont;
}

void ChFseqNode::SetDuration(double mdur)
{
	duration = mdur;
	if (duration<0)
		duration =0;
	t_end = t_start+duration;
};

void ChFseqNode::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// stream out all member data
	mstream << this->duration;
	mstream << this->weight;
	mstream << this->t_start;
	mstream << this->t_end;
	mstream << this->Iy;
	mstream << this->Iydt;
	mstream << this->Iydtdt;
	mstream << this->y_cont;
	mstream << this->ydt_cont;
	mstream << this->ydtdt_cont;

	mstream.AbstractWrite(this->fx.get_ptr()); 
	//***TODO*** better direct management of shared pointers serialization
}

void ChFseqNode::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in all member data
	mstream >> this->duration;
	mstream >> this->weight;
	mstream >> this->t_start;
	mstream >> this->t_end;
	mstream >> this->Iy;
	mstream >> this->Iydt;
	mstream >> this->Iydtdt;
	mstream >> this->y_cont;
	mstream >> this->ydt_cont;
	mstream >> this->ydtdt_cont;

	ChFunction* fooshared;
	mstream.AbstractReadCreate(&fooshared);	 // instance new
	fx = ChSharedPtr<ChFunction>(fooshared); // swap old shared to new shared, may delete old
	//***TODO*** better direct management of shared pointers serialization
}


/////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Sequence> a_registration_sequence;



ChFunction_Sequence::ChFunction_Sequence ()
{
	start = 0;
}

ChFunction_Sequence::~ChFunction_Sequence ()
{
	functions.KillAll();
}

void ChFunction_Sequence::Copy (ChFunction_Sequence* source)
{
	start = source->start;
	functions.KillAll();
	ChFseqNode* mfxs;
	for (ChNode<ChFseqNode>* mnode = source->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mfxs = new ChFseqNode(ChSharedPtr<ChFunction>() , 0.0);
		mfxs->Copy(mnode->data);
		functions.AddTail(mfxs);
	}
}

ChFunction* ChFunction_Sequence::new_Duplicate ()
{
	ChFunction_Sequence* m_func;
	m_func = new ChFunction_Sequence;
	m_func->Copy(this);
	return (m_func);
}

int ChFunction_Sequence::InsertFunct (ChSharedPtr<ChFunction> myfx, double duration, double weight, bool c0, bool c1, bool c2, int position)
{
	ChFseqNode* mfxsegment = new ChFseqNode(myfx, duration);
	mfxsegment->y_cont = c0;
	mfxsegment->ydt_cont = c1;
	mfxsegment->ydtdt_cont = c2;
	mfxsegment->weight = weight;

	int inserted = FALSE;
	if (position == 0)
		{ functions.AddHead(mfxsegment); inserted = TRUE;}
	if (position == -1)
		{ functions.AddTail(mfxsegment); inserted = TRUE;}
	if (!inserted)
	{
		int ind = 1;
		for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
		{
			if (ind == position)
				{ functions.InsertAfter(mnode, mfxsegment); inserted = TRUE;  break; }
			ind ++;
		}
	}
	if (!inserted)
		{ functions.AddTail(mfxsegment);}
				// update the continuity offsets and timings
	this->Setup();
	return inserted;
}

// only for backward compatibility:
int ChFunction_Sequence::InsertFunct (ChFunction* myfx, double duration, double weight, bool c0, bool c1, bool c2, int position)
{
	ChSharedPtr<ChFunction> mysharedfx(myfx);
	return InsertFunct (mysharedfx, duration, weight, c0, c1, c2, position);
}

int ChFunction_Sequence::KillFunct (int position)
{
	int fcount = functions.Count();
	if (fcount == 0)
		return FALSE;
	if ((position == -1)||(position > fcount))
		{ functions.Kill(functions.GetTail()); return TRUE;}
	if (position == 0)
		{ functions.Kill(functions.GetHead()); return TRUE;}
	functions.Kill(functions.GetNum(position));

	this->Setup();
	return TRUE;
}

ChSharedPtr<ChFunction> ChFunction_Sequence::GetNthFunction (int position)
{
	int fcount = functions.Count();
	if (fcount == 0)
		return ChSharedPtr<ChFunction>();
	if ((position == -1)||(position > fcount))
		{ return functions.GetTail()->data->fx;}
	if (position == 0)
		{ return functions.GetHead()->data->fx;}
	return  functions.GetNum(position)->data->fx;
}

double ChFunction_Sequence::GetNthDuration(int position)
{
	double default_dur =0.0;
	int fcount = functions.Count();
	if (fcount == 0)
		return default_dur;
	if ((position == -1)||(position > fcount))
		{ return functions.GetTail()->data->duration;}
	if (position == 0)
		{ return functions.GetHead()->data->duration;}
	return  functions.GetNum(position)->data->duration;
}

ChFseqNode* ChFunction_Sequence::GetNthNode(int position)
{
	int fcount = functions.Count();
	if (fcount == 0)
		return NULL;
	if ((position == -1)||(position > fcount))
		{ return functions.GetTail()->data;}
	if (position == 0)
		{ return functions.GetHead()->data;}
	return  functions.GetNum(position)->data;
}

void ChFunction_Sequence::Setup()
{
	double basetime = this->start;
	double lastIy = 0;
	double lastIy_dt = 0;
	double lastIy_dtdt = 0;

	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mnode->data->t_start = basetime;
		mnode->data->t_end   = basetime + mnode->data->duration;
		mnode->data->Iy		= 0;
		mnode->data->Iydt	= 0;
		mnode->data->Iydtdt = 0;

		if (mnode->data->fx.IsType<ChFunction_Fillet3>())	// C0 C1 fillet
		{
			ChSharedPtr<ChFunction_Fillet3> mfillet = mnode->data->fx.DynamicCastTo<ChFunction_Fillet3>();
			mfillet->Set_y1(lastIy);
			mfillet->Set_dy1(lastIy_dt);
			if (mnode->next)
			{
				mfillet->Set_y2(mnode->next->data->fx->Get_y(0));
				mfillet->Set_dy2(mnode->next->data->fx->Get_y_dx(0));
			}else
			{
				mfillet->Set_y2(0);
				mfillet->Set_dy2(0);
			}
			mfillet->Set_end(mnode->data->duration);
			mnode->data->Iy = mnode->data->Iydt = mnode->data->Iydtdt = 0;
		}
		else	// generic continuity conditions
		{
			if (mnode->data->y_cont)
				mnode->data->Iy = lastIy - mnode->data->fx->Get_y(0);
			if (mnode->data->ydt_cont)
				mnode->data->Iydt = lastIy_dt - mnode->data->fx->Get_y_dx(0);
			if (mnode->data->ydtdt_cont)
				mnode->data->Iydtdt = lastIy_dtdt - mnode->data->fx->Get_y_dxdx(0);
		}

		lastIy = mnode->data->fx->Get_y(mnode->data->duration) +
				 mnode->data->Iy +
				 mnode->data->Iydt *  mnode->data->duration +
				 mnode->data->Iydtdt *  mnode->data->duration *  mnode->data->duration;
		lastIy_dt =
				 mnode->data->fx->Get_y_dx(mnode->data->duration) +
				 mnode->data->Iydt +
				 mnode->data->Iydtdt *  mnode->data->duration;
		lastIy_dtdt =
				 mnode->data->fx->Get_y_dxdx(mnode->data->duration) +
				 mnode->data->Iydtdt;

		basetime += mnode->data->duration;
	}
}

double ChFunction_Sequence::Get_y      (double x)
{
	double res = 0;
	double localtime;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			localtime = x - mnode->data->t_start;
			res = mnode->data->fx->Get_y(localtime) +
				  mnode->data->Iy +
				  mnode->data->Iydt *  localtime +
				  mnode->data->Iydtdt *  localtime *  localtime;
		}
	}
	return res;
}

double ChFunction_Sequence::Get_y_dx   (double x)
{
	double res = 0;
	double localtime;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			localtime = x - mnode->data->t_start;
			res = mnode->data->fx->Get_y_dx(localtime) +
				  mnode->data->Iydt +
				  mnode->data->Iydtdt *  localtime;
		}
	}
	return res;
}

double ChFunction_Sequence::Get_y_dxdx (double x)
{
	double res = 0;
	double localtime;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			localtime = x - mnode->data->t_start;
			res = mnode->data->fx->Get_y_dxdx(localtime) +
				  mnode->data->Iydtdt;
		}
	}
	return res;
}


double ChFunction_Sequence::Get_weight (double x)
{
	double res = 1.0;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			res = mnode->data->weight;
		}
	}
	return res;
}

void ChFunction_Sequence::Extimate_x_range (double& xmin, double& xmax)
{
	xmin = start;
	xmax = functions.GetTail()->data->t_end;
	if (xmin == xmax) xmax = xmin + 1.1;
}


int ChFunction_Sequence::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for all children..
	int cnt=1;
	char msubduration[50];
	char msubfunction[50];
	for (ChNode<ChFseqNode>* mnode = this->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		sprintf(msubduration,"node_n(%d).duration", cnt);

		chjs_propdata* mdataA = new chjs_propdata;
		strcpy(mdataA->propname, msubduration);
		strcpy(mdataA->label,    mdataA->propname);
		mdataA->haschildren = FALSE;
		mtree->AddTail(mdataA);
		i++;

		sprintf(msubfunction,"node_n(%d).fx", cnt);

		chjs_propdata* mdataB = new chjs_propdata;
		strcpy(mdataB->propname, msubfunction);
		strcpy(mdataB->label,    mdataB->propname);
		mdataB->haschildren = TRUE;
		mtree->AddTail(mdataB);

		i += mnode->data->fx->MakeOptVariableTree(&mdataB->children);

		cnt++;
	}

	return i;
}


int ChFunction_Sequence::HandleNumber()
{
	int tot = 1;
	for (ChNode<ChFseqNode>* mnode = this->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		tot++;
	}
	return tot;
}

int ChFunction_Sequence::HandleAccess(int handle_id, double mx, double my, bool set_mode)
{
	if (handle_id==0)
	{
		if (!set_mode)
			mx = this->Get_start();
		else
			this->Set_start(mx);
		return TRUE;
	}
	int tot = 1;
	for (ChNode<ChFseqNode>* mnode = this->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if (handle_id==tot)
		{
			if (!set_mode)
				mx = mnode->data->t_end;
			else
			{
				mnode->data->SetTend(mx);
				this->Setup();
			}
			return TRUE;
		}

	}

	return FALSE;
}


void ChFunction_Sequence::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	int stopID = 0;
	int goID = 1;

	mstream << Get_start();

	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mstream << goID;
		mstream << *mnode->data;
	}
	mstream << stopID;

}

void ChFunction_Sequence::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	double dfoo;
	mstream >> dfoo;		Set_start(dfoo);
	int mgoID;
	mstream >> mgoID;
	while (mgoID == 1)
	{
		ChFseqNode* mynode = new ChFseqNode(ChSharedPtr<ChFunction>(0), 0.0);
		mstream >> *mynode;
		functions.AddTail(mynode);
		mstream >> mgoID;
	}
}

void ChFunction_Sequence::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_SEQUENCE  \n";

	//***TO DO***
}






} // END_OF_NAMESPACE____


// eof
