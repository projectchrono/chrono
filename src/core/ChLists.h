#ifndef CHLISTS_H
#define CHLISTS_H

//////////////////////////////////////////////////
// 
//   ChLists.h
//
//   Base class for Chrono lists of pointers to 
//   objects. This has a different meaning from STL
//   lists (which are lists of objects), because ours
//   can manage the deletion of all pointed objects.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <memory.h>


namespace chrono 
{


#define TRUE  1
#define FALSE 0

//////////////////////////////////////////////////////////////////
//
// GENERIC LISTS AND NODES
//
 

/// Node for linked list. 
/// This will be used by ChList<>

template<class dt>
class ChNode {
public:
	dt* data;			///< pointer to data
	ChNode* prev;		///< previous node
	ChNode* next;		///< next node
	
			/// Constructor for node
	ChNode() {data = NULL; next = prev = NULL;}; 

			/// Constructor for node
	ChNode(dt* mydata)	{data = mydata; next = prev = NULL;}; 

	//~Ch_node();
};


///
/// Class for linked list. 
///
/// This linked list class can be used to 
/// build lists of pointers to objects.
/// This has a different meaning from STL
/// lists (which are lists of objects), because ours
/// can manage the deletion of all pointed objects.
///

template<class dt>
class ChList {
private:
	ChNode<dt>* head;  
	ChNode<dt>* tail;
public: 
			/// Constructor
	ChList();
			/// Deletion
	~ChList();

			/// Returns the data at the head of the list
	dt* GetHeadData() {if (head) return head->data; else return NULL;}

			/// Returns the data at the tail of the list
	dt* GetTailData() {if (tail) return tail->data; else return NULL;}

			/// Returns the head node
	ChNode<dt>* GetHead() {return head;}

			/// Returns the tail node
	ChNode<dt>* GetTail() {return tail;}

			/// Returns a node at given position in list. Returns null 
			/// if num exceeds num of nodes. Note: num=1 gets first	element,
	ChNode<dt>* GetNum(int num); 

			/// Insert data at the head of the list
	int AddHead(dt* mdata);

			/// Insert data at the tail of the list
	int AddTail(dt* mdata);

			/// Removes the head of list.
	int RemHead();

			/// Removes the tail of list.
	int RemTail();

	int InsertAfter(ChNode<dt>* mnode, ChNode<dt>* newnode);
	int InsertBefore(ChNode<dt>* mnode, ChNode<dt>* newnode);
	int InsertAfter(ChNode<dt>* mnode, dt* mdata);
	int InsertBefore(ChNode<dt>* mnode, dt* mdata);

			/// Removes a node.
			/// Note the Remove() command delete just the Ch_node, not the pointed 'data' object
	int Remove(ChNode<dt>* mnode);
		
			/// Removes all nodes.
	int RemoveAll();

			/// Removes a node and deletes its data.
			/// Note the Kill() commands remove and also use delete(data) to free the objects pointed by nodes!,
	int Kill(ChNode<dt>* mnode);
			
			/// Kills all nodes.
	int KillAll();

			/// Returns number of elements in list.
	int Count();
};





template<class dt>
ChList<dt>::ChList()
	{ head = tail = NULL;}; 
template<class dt> 
ChList<dt>::~ChList()
	{ RemoveAll();};

template <class dt> int ChList<dt>::AddHead(dt* mdata)
{
	ChNode<dt>* nnode = new ChNode<dt>; 
	nnode->data = mdata;
	nnode->prev = NULL; 
	nnode->next = head;
	if (head == NULL) // first elem 
		{head = nnode; tail = nnode;} 
	else
		{head->prev = nnode; head = nnode;}
	return TRUE;
};

template <class dt> int ChList<dt>::AddTail(dt* mdata)
{ 
	ChNode<dt>* nnode = new ChNode<dt>; 
	nnode->data = mdata;
	nnode->prev = tail; 
	nnode->next = NULL;
	if (head == NULL) // first elem 
		{head = nnode; tail = nnode;} 
	else
		{tail->next = nnode; tail = nnode;}
	return TRUE;
}; 

template <class dt> int ChList<dt>::RemHead()
{ 
	if (head == NULL) return FALSE;
	return Remove(head);
}; 

template <class dt> int ChList<dt>::RemTail()
{ 
	if (tail == NULL) return FALSE;
	return Remove(tail);
}; 

template <class dt> int ChList<dt>::InsertAfter(ChNode<dt>* mnode, ChNode<dt>* newnode)
{ 
	newnode->next = mnode->next;
	newnode->prev = mnode;
	if (mnode->next)
		mnode->next->prev = newnode;
	else tail = newnode;
	mnode->next = newnode;
	return TRUE;
}; 

template <class dt> int ChList<dt>::InsertBefore(ChNode<dt>* mnode, ChNode<dt>* newnode)
{ 
	newnode->next = mnode;
	newnode->prev = mnode->prev;
	if (mnode->prev)
		mnode->prev->next = newnode;
	else head = newnode;
	mnode->prev = newnode;
	return TRUE;
};

template <class dt> int ChList<dt>::InsertAfter(ChNode<dt>* mnode, dt* mdata)
{ 
	ChNode<dt>* nenode = new ChNode<dt>(mdata);
	return InsertAfter(mnode, nenode);
};
template <class dt> int ChList<dt>::InsertBefore(ChNode<dt>* mnode, dt* mdata)
{ 
	ChNode<dt>* nenode = new ChNode<dt>(mdata);
	return InsertBefore(mnode, nenode);
};

template <class dt> int ChList<dt>::Remove(ChNode<dt>* mnode)
{ 
	if (mnode == head)
		head = mnode->next;
	if (mnode == tail)
		tail = mnode->prev;
	if (mnode->next)
		mnode->next->prev = mnode->prev;
	if (mnode->prev)
		mnode->prev->next = mnode->next;
	delete mnode;
	return TRUE;
};

template <class dt> int ChList<dt>::RemoveAll()
{ 
	for (ChNode<dt>* mnode = head; mnode != NULL; mnode= head)
		Remove(mnode);
	return TRUE;
};

template <class dt> int ChList<dt>::Kill(ChNode<dt>* mnode)
{
	if (mnode->data)
		delete(mnode->data);
	Remove(mnode);
	return TRUE;
};

template <class dt> int ChList<dt>::KillAll()
{ 
	for (ChNode<dt>* mnode = head; mnode != NULL; mnode= head)
	{
		if (mnode->data) delete(mnode->data);
		Remove(mnode);
	}
	return TRUE;
}; 

template <class dt> int ChList<dt>::Count()
{ 
	int sum = 0;
	for (ChNode<dt>* mnode = head; mnode != NULL; mnode= mnode->next)
	{
		sum++;
	}
	return sum;
}; 

template <class dt> ChNode<dt>* ChList<dt>::GetNum(int num)
{ 
	int sum = 1;
	for (ChNode<dt>* mnode = head; mnode != NULL; mnode= mnode->next)
	{
		if (sum == num) return mnode;
		sum++;
	}
	return NULL;
}; 


} // END_OF_NAMESPACE____

#endif

