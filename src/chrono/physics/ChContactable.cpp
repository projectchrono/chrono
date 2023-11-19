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

#include "chrono/physics/ChContactable.h"

namespace chrono {

CH_UPCASTING_SANITIZED(ChContactable_1vars<6>, ChContactable, ChContactable_1vars_6_ChContactable)


class my_enum_mappers : public ChContactable {
  public:
    CH_ENUM_MAPPER_BEGIN(eChContactableType);
    CH_ENUM_VAL(eChContactableType::CONTACTABLE_UNKNOWN);
    CH_ENUM_VAL(eChContactableType::CONTACTABLE_6);
    CH_ENUM_VAL(eChContactableType::CONTACTABLE_3);
    CH_ENUM_VAL(eChContactableType::CONTACTABLE_333);
    CH_ENUM_VAL(eChContactableType::CONTACTABLE_666);
    CH_ENUM_MAPPER_END(eChContactableType);
};



void ChContactable::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContactable>();

    // serialize parent class
    //marchive << CHNVP(m_data); // cannot serialize as it is
}

void ChContactable::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChContactable>();

    // deserialize parent class
    //marchive >> CHNVP(m_data);  // cannot serialize as it is

}



}  // end namespace chrono
