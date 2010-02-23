#ifndef CHSTACK_H
#define CHSTACK_H

//////////////////////////////////////////////////
//  
//   ChStack.h
//
//   Class for stack of pointers to items (items
//   are automatically deleted on stack deletion
//   or item popping).
//   If you need a traditional stack, just use the 
//   stack of STL template library.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChLists.h"


namespace chrono 
{


// CLASS FOR STACKS 
//
/// Class for generic stack of pointers to items. 
/// Stacks are used for formula evaluations, for example.
/// Note that, on stack deletion, also the objects pointed by
/// the stack items are deleted (this is different from STL stacks).
/// Also, popping an element, will also delete it, etc.
/// If you need a 'classical' stack, use the STL template stack instead.

template<class dt>
class ChStack {
private: 
	long maxsize; 
	long used;
	ChList<dt>* slist;
public: 
	ChStack(long mmaxsize); 
	~ChStack();
	void Clear();

	void Set_maxsize(long mm) {maxsize = mm;};
	int Get_maxsize() {return maxsize;};
	int Get_used() {return used;};

	ChList<dt>* GetList() {return slist;};
	dt* GetTop() {return slist->GetHeadData();};
	dt* GetTail() {return slist->GetHeadData();};

		// Basic stack operations; return NULL if overflow
	int Push(dt* mp); // loads a data into stack top (no copy of pointed data)
	int Pop();
	int Dup();
	int Swap();	
};

template<class dt>
ChStack<dt>::ChStack(long nmaxsize)
{
	slist = new ChList<dt>;
	maxsize = nmaxsize;
	used = 0;
}
template<class dt>
ChStack<dt>::~ChStack()
{
	slist->KillAll(); // remove all pushed nodes and delete also the pointed data!  
	delete slist;
}
template<class dt>
void ChStack<dt>::Clear()
{
	slist->KillAll();
	delete slist;
	slist = new ChList<dt>;
	used = 0;
}
template<class dt>
int ChStack<dt>::Push(dt* mp)
{
	if (used >= maxsize) return FALSE;
	slist->AddHead(mp);
	used++;
	return TRUE;
}
template<class dt>
int ChStack<dt>::Pop()
{
	if (used <= 0) return FALSE;
	delete GetTop();	// delete data pointed by top
	slist->RemHead();
	used--;
	return TRUE;
}
template<class dt>
int ChStack<dt>::Dup()
{
	dt* newtop = new dt;
	newtop->Copy(GetTop());
	return Push(newtop);
}
template<class dt>
int ChStack<dt>::Swap()
{
	dt* mpnt0;
	dt* mpnt1;
	ChNode<dt>* mn0;
	ChNode<dt>* mn1;
	mn0 = slist->GetNum(1);
	mn1 = slist->GetNum(2);
	if (mn0 && mn1)
	{
		mpnt0 = mn0->data; 
		mpnt1 = mn1->data;
		mn0->data = mpnt1; 
		mn1->data = mpnt0;
	}
	else return NULL;
}




} // END_OF_NAMESPACE____

#endif  // END of ChFormule.h 
