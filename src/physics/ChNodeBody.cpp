///////////////////////////////////////////////////
//
//   ChNodeBody.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChNodeBody.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChNodeBody> a_registration_ChNodeBody;



//////////////////////////////////////
//////////////////////////////////////



ChNodeBody::ChNodeBody ()
{
	this->react= VNULL;
	this->cache_li_speed = VNULL;
	this->cache_li_pos = VNULL;

	this->nodes = 0;
	this->node_index = 0;
	this->body = 0;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
}


ChNodeBody::~ChNodeBody ()
{
	
}

void ChNodeBody::Copy(ChNodeBody* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy class data

	react = source->react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;

	this->nodes = 0;
	this->node_index = 0;
	this->body = 0;
}


int ChNodeBody::Initialize(ChSharedPtr<ChIndexedNodes> mnodes, ///< nodes container
						   unsigned int mnode_index, ///< index of the node to join
						   ChSharedPtr<ChBody>&  mbody,   ///< body to join 
						   ChVector<>* mattach 	
						   )
{
	ChIndexedNodes* mm1 = mnodes.get_ptr();
	ChBody* mm2 = mbody.get_ptr();
	assert(mm1 && mm2);
	assert(mm1->GetSystem() == mm2->GetSystem());

	this->nodes = mm1;
	this->node_index = mnode_index;
	this->body = mm2;

	this->constraint1.SetVariables(&(mm1->GetNode(node_index)->Variables()), &mm2->Variables());
	this->constraint2.SetVariables(&(mm1->GetNode(node_index)->Variables()), &mm2->Variables());
	this->constraint3.SetVariables(&(mm1->GetNode(node_index)->Variables()), &mm2->Variables());

	this->SetSystem(this->body->GetSystem());

	if (mattach)
	{
		this->attach_position = body->Point_World2Body(mattach);
	}
	else
	{
		ChVector<> temp= ((ChNodeXYZ*)(this->nodes->GetNode(this->node_index)))->GetPos(); // warning, downcast to ChNodeXYZ* 
		this->attach_position = body->Point_World2Body(&temp);
	}

	return true;
}


void ChNodeBody::Update (double mytime)
{
		// Inherit time changes of parent class
	ChPhysicsItem::Update(mytime);
	
		// update class data
	// ...
}




////////// LCP INTERFACES ////


void ChNodeBody::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint1);
	mdescriptor.InsertConstraint(&constraint2);
	mdescriptor.InsertConstraint(&constraint3);
}

void ChNodeBody::ConstraintsBiReset()
{
	constraint1.Set_b_i(0.);
	constraint2.Set_b_i(0.);
	constraint3.Set_b_i(0.);
}
 
void ChNodeBody::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	ChVector<> res = ((ChNodeXYZ*)(this->nodes->GetNode(this->node_index)))->GetPos() - this->body->Point_Body2World(&this->attach_position) ; 

	this->constraint1.Set_b_i(constraint1.Get_b_i() +  factor * res.x);
	this->constraint2.Set_b_i(constraint2.Get_b_i() +  factor * res.y);
	this->constraint3.Set_b_i(constraint3.Get_b_i() +  factor * res.z);
}

void ChNodeBody::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	// nothing
}


void ChNodeBody::ConstraintsLoadJacobians()
{
		// compute jacobians
	ChMatrix33<> Jxn;
	Jxn.Set33Identity();

	ChMatrix33<> Jxb;
	Jxb.Set33Identity();
	Jxb.MatrNeg();

	ChMatrix33<> atilde;
	atilde.Set_X_matrix(-this->attach_position);
	ChMatrix33<> Jrb;
	Jrb.MatrMultiply(*this->body->GetA(), atilde);

	this->constraint1.Get_Cq_a()->PasteClippedMatrix(&Jxn, 0,0, 1,3, 0,0);
	this->constraint2.Get_Cq_a()->PasteClippedMatrix(&Jxn, 1,0, 1,3, 0,0);
	this->constraint3.Get_Cq_a()->PasteClippedMatrix(&Jxn, 2,0, 1,3, 0,0);

	this->constraint1.Get_Cq_b()->PasteClippedMatrix(&Jxb, 0,0, 1,3, 0,0);
	this->constraint2.Get_Cq_b()->PasteClippedMatrix(&Jxb, 1,0, 1,3, 0,0);
	this->constraint3.Get_Cq_b()->PasteClippedMatrix(&Jxb, 2,0, 1,3, 0,0);
	this->constraint1.Get_Cq_b()->PasteClippedMatrix(&Jrb, 0,0, 1,3, 0,3);
	this->constraint2.Get_Cq_b()->PasteClippedMatrix(&Jrb, 1,0, 1,3, 0,3);
	this->constraint3.Get_Cq_b()->PasteClippedMatrix(&Jrb, 2,0, 1,3, 0,3);

}
 

void ChNodeBody::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	this->react.x = constraint1.Get_l_i() * factor; 
	this->react.y = constraint2.Get_l_i() * factor; 
	this->react.z = constraint3.Get_l_i() * factor; 
}

// Following functions are for exploiting the contact persistence

void  ChNodeBody::ConstraintsLiLoadSuggestedSpeedSolution()
{
	constraint1.Set_l_i(this->cache_li_speed.x);
	constraint2.Set_l_i(this->cache_li_speed.y);
	constraint3.Set_l_i(this->cache_li_speed.z);
}

void  ChNodeBody::ConstraintsLiLoadSuggestedPositionSolution()
{
	constraint1.Set_l_i(this->cache_li_pos.x);
	constraint2.Set_l_i(this->cache_li_pos.y);
	constraint3.Set_l_i(this->cache_li_pos.z);
}

void  ChNodeBody::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->cache_li_speed.x = (float)constraint1.Get_l_i();
	this->cache_li_speed.y = (float)constraint2.Get_l_i();
	this->cache_li_speed.z = (float)constraint3.Get_l_i();
}

void  ChNodeBody::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->cache_li_pos.x =  (float)constraint1.Get_l_i();
	this->cache_li_pos.y =  (float)constraint2.Get_l_i();
	this->cache_li_pos.z =  (float)constraint3.Get_l_i();
}



//////// FILE I/O

void ChNodeBody::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream << this->node_index;
	mstream << this->attach_position;
	mstream << this->react;
}

void ChNodeBody::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize class
	mstream >> this->node_index;
	mstream >> this->attach_position;
	mstream >> this->react;
}






} // END_OF_NAMESPACE____


/////////////////////
