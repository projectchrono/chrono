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

#ifndef CHLISTS_H
#define CHLISTS_H

#include <cstdlib>
#include <memory.h>

namespace chrono {

//////////////////////////////////////////////////////////////////
//
// GENERIC LISTS AND NODES
// ***OBSOLETE***

/// Node for linked list.
/// This will be used by ChList<>

template <class dt>
class ChNode {
  public:
    dt* data;      ///< pointer to data
    ChNode* prev;  ///< previous node
    ChNode* next;  ///< next node

    /// Constructor for node
    ChNode() {
        data = NULL;
        next = prev = NULL;
    };

    /// Constructor for node
    ChNode(dt* mydata) {
        data = mydata;
        next = prev = NULL;
    };

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

template <class dt>
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
    dt* GetHeadData() {
        if (head)
            return head->data;
        else
            return NULL;
    }

    /// Returns the data at the tail of the list
    dt* GetTailData() {
        if (tail)
            return tail->data;
        else
            return NULL;
    }

    /// Returns the head node
    ChNode<dt>* GetHead() { return head; }

    /// Returns the tail node
    ChNode<dt>* GetTail() { return tail; }

    /// Returns a node at given position in list. Returns null
    /// if num exceeds num of nodes. Note: num=1 gets first	element,
    ChNode<dt>* GetNum(int num);

    /// Insert data at the head of the list
    void AddHead(dt* mdata);

    /// Insert data at the tail of the list
    void AddTail(dt* mdata);

    /// Removes the head of list.
    bool RemHead();

    /// Removes the tail of list.
    bool RemTail();

    void InsertAfter(ChNode<dt>* mnode, ChNode<dt>* newnode);
    void InsertBefore(ChNode<dt>* mnode, ChNode<dt>* newnode);
    void InsertAfter(ChNode<dt>* mnode, dt* mdata);
    void InsertBefore(ChNode<dt>* mnode, dt* mdata);

    /// Removes a node.
    /// Note the Remove() command delete just the Ch_node, not the pointed 'data' object
    void Remove(ChNode<dt>* mnode);

    /// Removes all nodes.
    void RemoveAll();

    /// Removes a node and deletes its data.
    /// Note the Kill() commands remove and also use delete(data) to free the objects pointed by nodes!,
    void Kill(ChNode<dt>* mnode);

    /// Kills all nodes.
    void KillAll();

    /// Returns number of elements in list.
    int Count();
};

template <class dt>
ChList<dt>::ChList() {
    head = tail = NULL;
}
template <class dt>
ChList<dt>::~ChList() {
    RemoveAll();
}

template <class dt>
void ChList<dt>::AddHead(dt* mdata) {
    ChNode<dt>* nnode = new ChNode<dt>;
    nnode->data = mdata;
    nnode->prev = NULL;
    nnode->next = head;
    if (head == NULL)  // first elem
    {
        head = nnode;
        tail = nnode;
    } else {
        head->prev = nnode;
        head = nnode;
    }
}

template <class dt>
void ChList<dt>::AddTail(dt* mdata) {
    ChNode<dt>* nnode = new ChNode<dt>;
    nnode->data = mdata;
    nnode->prev = tail;
    nnode->next = NULL;
    if (head == NULL)  // first elem
    {
        head = nnode;
        tail = nnode;
    } else {
        tail->next = nnode;
        tail = nnode;
    }
}

template <class dt>
bool ChList<dt>::RemHead() {
    if (head == NULL)
        return false;
    Remove(head);
    return true;
}

template <class dt>
bool ChList<dt>::RemTail() {
    if (tail == NULL)
        return false;
    Remove(tail);
    return true;
}

template <class dt>
void ChList<dt>::InsertAfter(ChNode<dt>* mnode, ChNode<dt>* newnode) {
    newnode->next = mnode->next;
    newnode->prev = mnode;
    if (mnode->next)
        mnode->next->prev = newnode;
    else
        tail = newnode;
    mnode->next = newnode;
}

template <class dt>
void ChList<dt>::InsertBefore(ChNode<dt>* mnode, ChNode<dt>* newnode) {
    newnode->next = mnode;
    newnode->prev = mnode->prev;
    if (mnode->prev)
        mnode->prev->next = newnode;
    else
        head = newnode;
    mnode->prev = newnode;
}

template <class dt>
void ChList<dt>::InsertAfter(ChNode<dt>* mnode, dt* mdata) {
    ChNode<dt>* nenode = new ChNode<dt>(mdata);
    InsertAfter(mnode, nenode);
}

template <class dt>
void ChList<dt>::InsertBefore(ChNode<dt>* mnode, dt* mdata) {
    ChNode<dt>* nenode = new ChNode<dt>(mdata);
    InsertBefore(mnode, nenode);
}

template <class dt>
void ChList<dt>::Remove(ChNode<dt>* mnode) {
    if (mnode == head)
        head = mnode->next;
    if (mnode == tail)
        tail = mnode->prev;
    if (mnode->next)
        mnode->next->prev = mnode->prev;
    if (mnode->prev)
        mnode->prev->next = mnode->next;
    delete mnode;
}

template <class dt>
void ChList<dt>::RemoveAll() {
    for (ChNode<dt>* mnode = head; mnode != NULL; mnode = head)
        Remove(mnode);
}

template <class dt>
void ChList<dt>::Kill(ChNode<dt>* mnode) {
    if (mnode->data)
        delete mnode->data;
    Remove(mnode);
}

template <class dt>
void ChList<dt>::KillAll() {
    for (ChNode<dt>* mnode = head; mnode != NULL; mnode = head) {
        if (mnode->data)
            delete mnode->data;
        Remove(mnode);
    }
}

template <class dt>
int ChList<dt>::Count() {
    int sum = 0;
    for (ChNode<dt>* mnode = head; mnode != NULL; mnode = mnode->next) {
        sum++;
    }
    return sum;
}

template <class dt>
ChNode<dt>* ChList<dt>::GetNum(int num) {
    int sum = 1;
    for (ChNode<dt>* mnode = head; mnode != NULL; mnode = mnode->next) {
        if (sum == num)
            return mnode;
        sum++;
    }
    return NULL;
}

}  // end namespace chrono

#endif
