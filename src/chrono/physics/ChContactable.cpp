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

ChContactable::ChContactable() : collision_model(nullptr) {}

void ChContactable::AddCollisionModel(std::shared_ptr<ChCollisionModel> model) {
    collision_model = model;
    model->SetContactable(this);
}

void ChContactable::AddCollisionShape(std::shared_ptr<ChCollisionShape> shape, const ChFrame<>& frame) {
    if (!collision_model) {
        auto model = chrono_types::make_shared<ChCollisionModel>();
        AddCollisionModel(model);
    }
    collision_model->AddShape(shape, frame);
}

std::shared_ptr<ChCollisionModel> ChContactable::GetCollisionModel() const {
    return collision_model;
}

void ChContactable::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContactable>();

    // serialize parent class
    // archive_out << CHNVP(m_data); // cannot serialize as it is
}

void ChContactable::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContactable>();

    // deserialize parent class
    // archive_in >> CHNVP(m_data);  // cannot serialize as it is
}

}  // end namespace chrono
