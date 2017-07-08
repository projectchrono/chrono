// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//   Part of the code is a modified version of the
//   retro_sweepnprune.h code from the OpenTissue
//   project, by K.Erleben:
//
//   "OpenTissue, A toolbox for physical based simulation and animation.
//   Copyright (C) 2004 Department of Computer Science, University of Copenhagen"
// ==============================================================================


#ifndef CHC_SWEEPANDPRUNE_H
#define CHC_SWEEPANDPRUNE_H

#if (_MSC_VER >= 1200)
#pragma warning(4 : 4786)
#endif

#include "chrono/collision/edgetempest/ChCBroadPhaseCollider.h"
#include "chrono/geometry/ChGeometry.h"

#ifdef COMPILER_GCC
#include <ext/hash_map>
namespace chronohash = ::__gnu_cxx;
#else
#include <hash_map>
namespace chronohash = ::stdext;  // NOTE: in Visual C++ Toolkit 2003 is ::std;
#endif

namespace chrono {
namespace collision {

///
/// Class for the 'sweep and prune' broad-phase collision.
///  This phase organizes all the min-max endpoints of the
/// AABB boxes of bodies into three sorted lists for the
/// x,y,z world axes. Overlapping pairs are roughly
/// detected in quasi-linear time during the sorting operation,
/// which is almost linear especially if a previous sorting is
/// available (i.e.: please exploit coherence as much as possible)
///  This algorithm uses an hash search to store/retrieve the
/// partial overlaps, for maximum speed efficiency - however it
/// may need lot of memory for storing the partial overlaps, when
/// number of objects is high (>1000).
///

template <class model_type>
class ChSweepAndPrune : public ChBroadPhaseCollider<model_type> {
  protected:
    typedef typename ChIntervalEndpoint<model_type> ChEndPoint;
    typedef typename std::list<ChEndPoint*> ChAxisOfCoordinates;
    typedef typename chronohash::hash_map<unsigned int, ChMates<model_type>*> ChMatesPtrMap;

    ChAxisOfCoordinates m_axisX;  ///< X-Axis. The x-coordinates of AABB interval endpoints.
    ChAxisOfCoordinates m_axisY;  ///< Y-Axis. The y-coordinates of AABB interval endpoints.
    ChAxisOfCoordinates m_axisZ;  ///< Z-Axis. The z-coordinates of AABB interval endpoints.

    ChMatesPtrList m_reported;  ///< A list containing the currently reported overlaps.

    ChMatesPtrMap m_mates;  ///< An hash map with partial overlaps (only true overlaps if their counter =3)

  public:
    /// Constructor
    ChSweepAndPrune(){};

    /// Destructor
    virtual ~ChSweepAndPrune(void) {
        removeAllChMates();  // this because mates were instanced
    }

    /// Clears all data instanced by this algorithm.
    virtual void Clear(void) {
        m_axisX.clear();
        m_axisY.clear();
        m_axisZ.clear();
        m_reported.clear();
        removeAllChMates();
    };

    /// Adds a collision model to the broad-phase
    /// engine (custom data may be allocated).
    virtual void Add(model_type* model) {
        m_axisX.push_back(&model->m_absoluteAABB.m_beginX);
        m_axisX.push_back(&model->m_absoluteAABB.m_endX);
        m_axisY.push_back(&model->m_absoluteAABB.m_beginY);
        m_axisY.push_back(&model->m_absoluteAABB.m_endY);
        m_axisZ.push_back(&model->m_absoluteAABB.m_beginZ);
        m_axisZ.push_back(&model->m_absoluteAABB.m_endZ);
    };

    /// Removes a collision model from the broad-phase
    /// engine (custom data may be deallocated).
    virtual void Remove(model_type* model) {
        // assert(m_configuration);
        m_axisX.remove(&model->m_absoluteAABB.m_beginX);
        m_axisX.remove(&model->m_absoluteAABB.m_endX);
        m_axisY.remove(&model->m_absoluteAABB.m_beginY);
        m_axisY.remove(&model->m_absoluteAABB.m_endY);
        m_axisZ.remove(&model->m_absoluteAABB.m_beginZ);
        m_axisZ.remove(&model->m_absoluteAABB.m_endZ);

        // Delete references in reported pairs
        ChMatesPtrList::iterator repmates = m_reported.begin();
        while (repmates != m_reported.end()) {
            ChMates<model_type>* mymates = *repmates;
            if ((mymates->GetModelA() == model) || (mymates->GetModelB() == model)) {
                ChMatesPtrList::iterator delmates = repmates;
                repmates++;
                m_reported.erase(delmates);
            } else {
                repmates++;
            }
        }

        // Delete references in partial pairs
        ChMatesPtrMap::iterator mates = m_mates.begin();
        while (mates != m_mates.end()) {
            if ((mates->second->GetModelA() == model) || (mates->second->GetModelB() == model)) {
                ChMatesPtrMap::iterator delmates = mates;
                mates++;
                delete delmates->second;  // also delete allocated mates data!
                m_mates.erase(delmates);
            } else {
                mates++;
            }
        }
    };

    /// Run the 'sweep and prune' algorithm, which finds the
    /// overlapping pairs of bounding boxes using three sort operations.
    /// If you call this function mutiple times, the execution
    /// time will be really small, because x,y,z lists usually change
    /// few intervals, so sorting takes few operations (exploits time coherency)
    /// After the algorithm has run, you can look into the list of
    /// reported pairs using the GetReportedMates()
    virtual void Run() {
        sort(m_axisX);
        sort(m_axisY);
        sort(m_axisZ);

        // purgeUnusedChMates();
    };

    /// After the broad phase algorithm has run, you can use
    /// this method to access the list with the reported pairs of
    /// overlapping models.
    virtual ChMatesPtrList& GetReportedMates() { return m_reported; }

  private:
    /// Get the connection between two bodies A and B, using the
    /// hash mapping, to avoid linear time complexity.
    ChMates<model_type>* getChMates(model_type* A, model_type* B) {
        assert(A);
        assert(B);
        assert(A != B);
        typename ChMatesPtrMap::iterator mates = m_mates.find(ChMates<model_type>::getHashkey(A, B));
        if (mates == m_mates.end())
            return 0;
        return mates->second;
    };

    /// Store the connection between two bodies A and B, using the
    /// hash mapping, to avoid linear time complexity.
    ChMates<model_type>* addChMates(model_type* A, model_type* B) {
        assert(A);
        assert(B);
        assert(A != B);
        assert(m_mates.find(ChMates<model_type>::getHashkey(A, B)) == m_mates.end());
        ChMates<model_type>* mates = new ChMates<model_type>(A, B);
        m_mates.insert(std::pair<unsigned int, ChMates<model_type>*>(mates->getHashkey(), mates));
        return mates;
    };

    void removeAllChMates() {
        ChMatesPtrMap::iterator mates = m_mates.begin();
        while (mates != m_mates.end()) {
            delete mates->second;
            mates++;
        }

        m_mates.clear();

        //*****TO CHECK****
    }

    void purgeUnusedChMates() {
        ChMatesPtrMap::iterator mates = m_mates.begin();
        while (mates != m_mates.end()) {
            if (mates->second->m_snp_axis_overlap_count == 0) {
                ChMatesPtrMap::iterator delmates = mates;
                mates++;
                delete delmates->second;
                m_mates.erase(delmates);
            } else {
                mates++;
            }
        }
    }

  protected:
    /// Coordinate Sorting Algorithm.
    /// Performs custom (non STL) list sorting
    void sort(ChAxisOfCoordinates& axis) {
        if (axis.empty())
            return;
        typename ChAxisOfCoordinates::iterator left;
        typename ChAxisOfCoordinates::iterator right;
        typename ChAxisOfCoordinates::iterator scan = axis.begin();
        left = scan;
        ++scan;

        //--- Scan list from left to right
        for (; scan != axis.end();) {
            right = scan;
            ++right;
            //--- Check if we encountered an element that was smaller
            //--- than its left neighbor
            if (isWrong(*left, *scan)) {
                //--- If so we continuously swap the element to the left
                //--- in the list until its left neighbor is no longer
                //--- bigger than itself
                typename ChAxisOfCoordinates::iterator _right;
                typename ChAxisOfCoordinates::iterator _left;
                _right = scan;
                _left = left;
                do {
                    // Boxes was crossing: test if broad phase collision
                    // must be reported
                    swapAction(*_left, *_right);
                    // std::iter_swap(_right,_left); //***vc6
                    std::swap(*_right, *_left);  //***vc2003
                    _right = _left;
                    if (_left == axis.begin())
                        break;
                    --_left;
                } while (isWrong(*_left, *_right));
            }
            //--- Now we can pick up the overall left-to-right scan again
            left = scan;
            scan = right;
        }
    };

    /// Wrongly Sorted Query Method.
    /// Computes whatever the two end
    /// points appears in the proper order along a
    /// coordinate axis.
    ///
    /// @param left     A pointer to the endpoint that has the
    ///                 left position in the list (head is assumed
    ///                 to be left most and tail rightmost).
    /// @param right    A pointer to the endpoint that has the
    ///                 right position in the list.
    ///
    /// @return         If left and right should be swapped
    ///                 then the return value it true otherwise
    ///                 it is false.

    const bool isWrong(ChEndPoint* left, ChEndPoint* right) {
        //--- Values must be sorted in increasing order from left to right
        if (right->m_value < left->m_value)
            return true;
        //--- Endpoints must be sorted so ``begin '' comes before ``end''
        if (right->m_value == left->m_value) {
            if ((right->m_type == 0) && (left->m_type == 1))
                return true;
        }
        return false;
    };

    /// Swap Action.
    /// NOTE: The method is called before the actual swap!!!
    ///
    /// The method is assumed to perform all the appropriate
    /// semantic actions, corresponding to the swap.
    ///
    /// @param left     A pointer to the endpoint that has the
    ///                 left position in the list (head is assumed
    ///                 to be left most and tail rightmost).
    /// @param right    A pointer to the endpoint that has the
    ///                 right position in the list.

    void swapAction(ChEndPoint* left, ChEndPoint* right) {
        ChMates<model_type>* mates = this->getChMates(left->m_model, right->m_model);
        if (mates == 0) {
            mates = this->addChMates(left->m_model, right->m_model);
        }

        if ((left->m_type == 1) && (right->m_type == 0)) {
            ++(mates->m_snp_axis_overlap_count);
            if (mates->m_snp_axis_overlap_count == 3) {
                mates->m_snp_reported_overlap = m_reported.insert(m_reported.end(), mates);
                mates->m_snp_overlap_reported = true;
            }
        }

        if ((left->m_type == 0) && (right->m_type == 1)) {
            --(mates->m_snp_axis_overlap_count);
            if (mates->m_snp_axis_overlap_count == 2) {
                if (mates->m_snp_overlap_reported) {
                    m_reported.erase(mates->m_snp_reported_overlap);
                    mates->m_snp_overlap_reported = false;
                }
            }
        }
    };

  public:
    const double getMin(ChAxisOfCoordinates& axis) { return (axis.front())->m_value; };
    const double getMax(ChAxisOfCoordinates& axis) { return (axis.back())->m_value; };

    /// Debugging test tool:  consistency test method (for a single axis)
    const bool isConsistent(ChAxisOfCoordinates& axis) {
        if (axis.empty())
            return true;
        typename ChAxisOfCoordinates::iterator scan = axis.begin();
        typename ChAxisOfCoordinates::iterator right = scan;
        ++right;
        for (; right != axis.end();) {
            if ((*scan)->m_value > (*right)->m_value)
                return false;
            scan = right;
            ++right;
        }
        return true;
    };

    const bool isConsistentX() { return isConsistent(this->m_axisX); }
    const bool isConsistentY() { return isConsistent(this->m_axisY); }
    const bool isConsistentZ() { return isConsistent(this->m_axisZ); }
    int nEndpointsX() { return m_axisX.size(); }
    int nEndpointsY() { return m_axisY.size(); }
    int nEndpointsZ() { return m_axisZ.size(); }
    int nReported() { return m_reported.size(); }
    int nMates() { return m_mates.size(); }

};  // end of s&p class

}  // end namespace collision
}  // end namespace chrono

#endif
