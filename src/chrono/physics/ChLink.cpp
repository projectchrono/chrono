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

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChLink)   // NO! abstract class!

ChLink::ChLink(const ChLink& other) : ChLinkBase(other) {
    m_body1 = nullptr;
    m_body2 = nullptr;

    react_force = other.react_force;
    react_torque = other.react_torque;
}

// -----------------------------------------------------------------------------

ChFramed ChLink::GetFrame1Abs() const {
    return GetFrame1Rel() >> *m_body1;
}

ChFramed ChLink::GetFrame2Abs() const {
    return GetFrame2Rel() >> *m_body2;
}

// -----------------------------------------------------------------------------

// The default ChLink implementation assumes that react_force and react_torque represent the reaction wrench on the 2nd
// body, expressed in the link frame 2. A derived class may interpret react_force and react_torque differently, in which
// case it must override GetReaction1() and GetReaction2().

ChWrenchd ChLink::GetReaction1() const {
    auto w1_abs = GetFrame2Abs().TransformWrenchLocalToParent({-react_force, -react_torque});
    return GetFrame1Abs().TransformWrenchParentToLocal(w1_abs);
}

ChWrenchd ChLink::GetReaction2() const {
    return {react_force, react_torque};
}

// -----------------------------------------------------------------------------

void ChLink::Update(double time, bool update_assets) {
    ChPhysicsItem::Update(time, update_assets);
}

void ChLink::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLink>();

    // serialize parent class
    ChLinkBase::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_body1);
    archive_out << CHNVP(m_body2);
}

void ChLink::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLink>();

    // deserialize parent class
    ChLinkBase::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_body1);
    archive_in >> CHNVP(m_body2);
}

}  // end namespace chrono
