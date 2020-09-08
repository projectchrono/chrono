// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include "chrono_distributed/physics/ChDomainDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono_parallel/ChDataManager.h"

#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"

#include <mpi.h>
#include <stdlib.h>
#include <iostream>
#include <memory>

using namespace chrono;

ChDomainDistributed::ChDomainDistributed(ChSystemDistributed* sys) {
    this->my_sys = sys;
    split_axis = 0;
    split = false;
    axis_set = false;
}

ChDomainDistributed::~ChDomainDistributed() {}

void ChDomainDistributed::SetSplitAxis(int i) {
    assert(!split);
    if (i == 0 || i == 1 || i == 2) {
        split_axis = i;
        axis_set = true;
    } else {
        GetLog() << "Invalid axis\n";
    }
}

void ChDomainDistributed::SetSimDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi) {
    assert(!split);

    boxlo.Set(xlo, ylo, zlo);
    boxhi.Set(xhi, yhi, zhi);

    double len_x = boxhi.x() - boxlo.x();
    double len_y = boxhi.y() - boxlo.y();
    double len_z = boxhi.z() - boxlo.z();

    if (len_x <= 0 || len_y <= 0 || len_z <= 0)
        my_sys->ErrorAbort("Invalid domain dimensions.");

    if (!axis_set) {
        // Index of the longest domain axis 0=x, 1=y, 2=z
        split_axis = (len_x >= len_y) ? 0 : 1;
        split_axis = (len_z >= boxhi[split_axis] - boxlo[split_axis]) ? 2 : split_axis;
    }

    SplitDomain();
}

void ChDomainDistributed::SplitDomain() {
    // Length of this subdomain along the long axis
    double sub_len = (boxhi[split_axis] - boxlo[split_axis]) / my_sys->num_ranks;

    for (int i = 0; i < 3; i++) {
        if (split_axis == i) {
            sublo[i] = boxlo[i] + my_sys->my_rank * sub_len;
            subhi[i] = sublo[i] + sub_len;
        } else {
            sublo[i] = boxlo[i];
            subhi[i] = boxhi[i];
        }
    }
    split = true;
}

int ChDomainDistributed::GetRank(ChVector<double> pos) {
    double sub_len = subhi[split_axis] - sublo[split_axis];
    double pt = pos[split_axis] - boxlo[split_axis];

    return (int)(pt / sub_len);
}

distributed::COMM_STATUS ChDomainDistributed::GetRegion(double pos) {
    int num_ranks = my_sys->num_ranks;
    if (num_ranks == 1) {
        return distributed::OWNED;
    }
    int my_rank = my_sys->my_rank;
    double ghost_layer = my_sys->GetGhostLayer();
    double high = subhi[split_axis];
    double low = sublo[split_axis];

    if (my_rank != 0 && my_rank != num_ranks - 1) {
        if (pos >= low + ghost_layer && pos < high - ghost_layer) {
            return distributed::OWNED;
        } else if (pos >= high && pos < high + ghost_layer) {
            return distributed::GHOST_UP;
        } else if (pos >= high - ghost_layer && pos < high) {
            return distributed::SHARED_UP;
        } else if (pos >= low && pos < low + ghost_layer) {
            return distributed::SHARED_DOWN;
        } else if (pos >= low - ghost_layer && pos < low) {
            return distributed::GHOST_DOWN;
        } else if (pos >= high + ghost_layer) {
            return distributed::UNOWNED_UP;
        } else if (pos < low - ghost_layer) {
            return distributed::UNOWNED_DOWN;
        }
    }

    else if (my_rank == 0) {
        if (pos >= low && pos < high - ghost_layer) {
            return distributed::OWNED;
        } else if (pos >= high && pos < high + ghost_layer) {
            return distributed::GHOST_UP;
        } else if (pos >= high - ghost_layer && pos < high) {
            return distributed::SHARED_UP;
        } else if (pos >= high + ghost_layer) {
            return distributed::UNOWNED_UP;
        } else if (pos < low) {
            return distributed::UNOWNED_DOWN;
        }
    }

    else if (my_rank == num_ranks - 1) {
        if (pos >= low + ghost_layer && pos < high) {
            return distributed::OWNED;
        } else if (pos >= low && pos < low + ghost_layer) {
            return distributed::SHARED_DOWN;
        } else if (pos >= low - ghost_layer && pos < low) {
            return distributed::GHOST_DOWN;
        } else if (pos >= high) {
            return distributed::UNOWNED_UP;
        } else if (pos < low - ghost_layer) {
            return distributed::UNOWNED_DOWN;
        }
    }

    GetLog() << "Error classifying body\n";
    return distributed::UNDEFINED;
}

distributed::COMM_STATUS ChDomainDistributed::GetBodyRegion(int index) {
    return GetRegion(my_sys->data_manager->host_data.pos_rigid[index][split_axis]);
}

distributed::COMM_STATUS ChDomainDistributed::GetBodyRegion(std::shared_ptr<ChBody> body) {
    return GetRegion(body->GetPos()[split_axis]);
}

void ChDomainDistributed::PrintDomain() {
    GetLog() << "Domain:\n"
                "Box:\n"
                "\tX: "
             << boxlo.x() << " to " << boxhi.x()
             << "\n"
                "\tY: "
             << boxlo.y() << " to " << boxhi.y()
             << "\n"
                "\tZ: "
             << boxlo.z() << " to " << boxhi.z()
             << "\n"
                "Subdomain: Rank "
             << my_sys->my_rank
             << "\n"
                "\tX: "
             << sublo.x() << " to " << subhi.x()
             << "\n"
                "\tY: "
             << sublo.y() << " to " << subhi.y()
             << "\n"
                "\tZ: "
             << sublo.z() << " to " << subhi.z() << "\n";
}
