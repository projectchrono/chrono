// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

/// Base class for defining constraints between a couple of two one-degree-of-freedom
/// parts; i.e., shafts that can be used to build 1D models of powertrains.

class ChApi ChShaftsCouple : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsCouple)

  protected:
    ChShaft* shaft1;  ///< first shaft
    ChShaft* shaft2;  ///< second shaft

  public:
    ChShaftsCouple() : shaft1(NULL), shaft2(NULL) {}
    ChShaftsCouple(const ChShaftsCouple& other) : ChPhysicsItem(other) {}
    ~ChShaftsCouple() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsCouple* Clone() const override { return new ChShaftsCouple(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() { return 2; }

    /// Use this function after gear creation, to initialize it, given two shafts to join.
    /// Each shaft must belong to the same ChSystem.
    /// Derived classes might overload this (here, basically it only sets the two pointers)
    virtual bool Initialize(std::shared_ptr<ChShaft> mshaft1,  ///< first  shaft to join
                            std::shared_ptr<ChShaft>
                                mshaft2  ///< second shaft to join
                            ) {
        ChShaft* mm1 = mshaft1.get();
        ChShaft* mm2 = mshaft2.get();
        assert(mm1 && mm2);
        assert(mm1 != mm2);
        assert(mm1->GetSystem() == mm2->GetSystem());
        shaft1 = mm1;
        shaft2 = mm2;
        SetSystem(shaft1->GetSystem());
        return true;
    }

    /// Get the first (input) shaft
    ChShaft* GetShaft1() { return shaft1; }
    /// Get the second (output) shaft
    ChShaft* GetShaft2() { return shaft2; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    /// Children classes might overload this.
    virtual double GetTorqueReactionOn1() const { return 0; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    /// Children classes might overload this.
    virtual double GetTorqueReactionOn2() const { return 0; }

    /// Get the actual reative angle in terms of phase of shaft 1 respect to 2.
    double GetRelativeRotation() const { return (this->shaft1->GetPos() - this->shaft2->GetPos()); }
    /// Get the actual relative speed in terms of speed of shaft 1 respect to 2.
    double GetRelativeRotation_dt() const { return (this->shaft1->GetPos_dt() - this->shaft2->GetPos_dt()); }
    /// Get the actual relative acceleration in terms of speed of shaft 1 respect to 2.
    double GetRelativeRotation_dtdt() const { return (this->shaft1->GetPos_dtdt() - this->shaft2->GetPos_dtdt()); }

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChShaftsCouple>();

        // serialize parent class
        ChPhysicsItem::ArchiveOUT(marchive);

        // serialize all member data:
        // marchive << CHNVP(shaft1);  //***TODO*** serialize, with shared ptr
        // marchive << CHNVP(shaft2);  //***TODO*** serialize, with shared ptr
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChShaftsCouple>();

        // deserialize parent class:
        ChPhysicsItem::ArchiveIN(marchive);

        // deserialize all member data:
        // marchive >> CHNVP(shaft1);  //***TODO*** serialize, with shared ptr
        // marchive >> CHNVP(shaft2);  //***TODO*** serialize, with shared ptr
    }
};

CH_CLASS_VERSION(ChShaftsCouple,0)

}  // end namespace chrono

#endif
