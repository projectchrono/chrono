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

#include "chrono/fea/ChBeamSection.h"

namespace chrono {
namespace fea {

    void ChBeamSection::ArchiveOut(ChArchiveOut& archive_out) {
        // version number
        archive_out.VersionWrite<ChBeamSection>();

        // serialize parent class
        //...::ArchiveOut(archive_out);

        // serialize all member data:
        archive_out << CHNVP(this->draw_shape);
    }

    /// Method to allow de serialization of transient data from archives.
    void ChBeamSection::ArchiveIn(ChArchiveIn& archive_in) {
        // version number
        /*int version =*/archive_in.VersionRead<ChBeamSection>();

        // deserialize parent class:
        //...::ArchiveIn(archive_in);

        // deserialize all member data:
        archive_in >> CHNVP(this->draw_shape);
    }

}  // end namespace fea
}  // end namespace chrono
