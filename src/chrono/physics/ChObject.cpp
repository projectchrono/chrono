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

#include <atomic>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChObject.h"

namespace chrono {

ChObj::ChObj() : m_tag(-1), ChTime(0) {
    m_identifier = GenerateUniqueIdentifier();
}

ChObj::ChObj(const ChObj& other) {
    m_identifier = GenerateUniqueIdentifier();

    m_name = other.m_name;
    ChTime = other.ChTime;
}

int ChObj::GenerateUniqueIdentifier() {
    static std::atomic<std::uint32_t> uid{0};
    return ++uid;
}

void ChObj::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChObj>();

    // stream out all member data
    archive_out << CHNVP(m_name);
    archive_out << CHNVP(m_tag);
    archive_out << CHNVP(ChTime);
}

void ChObj::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChObj>();

    // stream out all member data
    archive_in >> CHNVP(m_name);
    archive_in >> CHNVP(m_tag);
    archive_in >> CHNVP(ChTime);
}

}  // end namespace chrono
