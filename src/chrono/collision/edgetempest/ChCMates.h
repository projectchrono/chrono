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

#ifndef CHC_MATES_H
#define CHC_MATES_H

namespace chrono {
namespace collision {

/// Class for couple of 'overlapping' models, i.e. neighboring graph edge
/// (connecting two collision models).
/// Mostly used by broadphase colliders, for example like in
/// ChSweepAndPrune() engine.

template <class model_type>
class ChMates {
  private:
    model_type* m_A;
    model_type* m_B;

  public:
    /// Axis overlap counter
    /// (0=no overlaps, 1=partial overlap,only on an axis, 2=.., 3=full overlap on XYZ axes
    short m_snp_axis_overlap_count;

    /// Boolean flag indicating whether a overlap have been reported.
    bool m_snp_overlap_reported;

    /// Iterator to already-reported overlap.
    typename std::list<ChMates<model_type>*>::iterator m_snp_reported_overlap;

    /// Constructor. Arguments are the A and B models linked by this
    /// graph edge of mates.
    ChMates(model_type* mA = 0, model_type* mB = 0)
        : m_A(mA), m_B(mB), m_snp_axis_overlap_count(0), m_snp_overlap_reported(false){};

    /// Return the hash key of this mates
    const unsigned int getHashkey(void) const { return ChMates::getHashkey(m_A, m_B); };

    /// Static function: return hash key for a generic mates item.
    static const unsigned int getHashkey(model_type* A, model_type* B) {
        assert(A);
        assert(B);
        assert(A->GetBody()->GetIdentifier() != B->GetBody()->GetIdentifier());
        if (A->GetBody()->GetIdentifier() < B->GetBody()->GetIdentifier())
            return static_cast<unsigned int>(
                ((A->GetBody()->GetIdentifier() << 16) | (B->GetBody()->GetIdentifier() & 0x0000FFFF)));
        return static_cast<unsigned int>(
            ((B->GetBody()->GetIdentifier() << 16) | (A->GetBody()->GetIdentifier() & 0x0000FFFF)));
    };

    /// Returns the first collision model
    model_type* GetModelA() { return m_A; }

    /// Returns the second collision model
    model_type* GetModelB() { return m_B; }
};

}  // end namespace collision
}  // end namespace chrono

#endif
