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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSHAFTSCOUPLE_H
#define CHSHAFTSCOUPLE_H

#include "chrono/physics/ChShaft.h"

namespace chrono {

/// Base class for defining constraints between a couple of two one-degree-of-freedom parts.
class ChApi ChShaftsCouple : public ChPhysicsItem {
  public:
    ChShaftsCouple() : shaft1(nullptr), shaft2(nullptr) {}
    ChShaftsCouple(const ChShaftsCouple& other) : ChPhysicsItem(other) {}
    virtual ~ChShaftsCouple() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsCouple* Clone() const override { return new ChShaftsCouple(*this); }

    /// Get the number of scalar variables affected by constraints in this couple.
    virtual unsigned int GetNumAffectedCoords() { return 2; }

    /// Initialize this shafts couple, given two shafts to join.
    /// Both shafts must belong to the same ChSystem.
    virtual bool Initialize(std::shared_ptr<ChShaft> shaft_1,  ///< first  shaft to join
                            std::shared_ptr<ChShaft> shaft_2   ///< second shaft to join
    ) {
        shaft1 = shaft_1.get();
        shaft2 = shaft_2.get();

        assert(shaft1 && shaft2);
        assert(shaft1 != shaft2);
        assert(shaft1->GetSystem() == shaft2->GetSystem());

        SetSystem(shaft1->GetSystem());
        return true;
    }

    /// Get the first (input) shaft.
    ChShaft* GetShaft1() const { return shaft1; }

    /// Get the second (output) shaft.
    ChShaft* GetShaft2() const { return shaft2; }

    /// Get the reaction (torque or force) exchanged between the two shafts, considered as applied to the 1st axis.
    virtual double GetReaction1() const { return 0; }

    /// Get the reaction (torque or force) exchanged between the two shafts, considered as applied to the 2nd axis.
    virtual double GetReaction2() const { return 0; }

    /// Get the actual relative position (angle or displacement) in terms of phase of shaft 1 with respect to 2.
    double GetRelativePos() const { return (shaft1->GetPos() - shaft2->GetPos()); }

    /// Get the actual relative speed (angle or displacement) of shaft 1 with respect to 2.
    double GetRelativePosDt() const { return (shaft1->GetPosDt() - shaft2->GetPosDt()); }

    /// Get the actual relative acceleration (angle or displacement) of shaft 1 with respect to 2.
    double GetRelativePosDt2() const { return (shaft1->GetPosDt2() - shaft2->GetPosDt2()); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override {
        // version number
        archive_out.VersionWrite<ChShaftsCouple>();

        // serialize parent class
        ChPhysicsItem::ArchiveOut(archive_out);

        // serialize all member data:
        archive_out << CHNVP(shaft1);  //// TODO  serialize, with shared ptr
        archive_out << CHNVP(shaft2);  //// TODO  serialize, with shared ptr
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override {
        // version number
        /*int version =*/archive_in.VersionRead<ChShaftsCouple>();

        // deserialize parent class:
        ChPhysicsItem::ArchiveIn(archive_in);

        // deserialize all member data:
        archive_in >> CHNVP(shaft1);  //// TODO  serialize, with shared ptr
        archive_in >> CHNVP(shaft2);  //// TODO  serialize, with shared ptr
    }

  protected:
    ChShaft* shaft1;  ///< first shaft
    ChShaft* shaft2;  ///< second shaft
};

CH_CLASS_VERSION(ChShaftsCouple, 0)

}  // end namespace chrono

#endif
