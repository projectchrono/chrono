// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: This class contains manages all data associated with a parallel
// System. Rather than passing in individual data parameters to different parts
// of the code like the collision detection and the solver, passing a pointer to
// a data manager is more convenient from a development perspective.
//
// =============================================================================

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"
#include "chrono_parallel/collision/ChCollision.h"

#include "chrono/core/ChStream.h"
#include "chrono/physics/ChMaterialSurface.h"

using namespace chrono;
using namespace chrono::collision;

ChParallelDataManager::ChParallelDataManager()
    : num_rigid_contacts(0),
      num_rigid_fluid_contacts(0),
      num_fluid_contacts(0),
      num_rigid_shapes(0),
      num_rigid_bodies(0),
      num_fluid_bodies(0),
      num_unilaterals(0),
      num_bilaterals(0),
      num_constraints(0),
      num_shafts(0),
      num_motors(0),
      num_linmotors(0),
      num_rotmotors(0),
      num_dof(0),
      num_fea_nodes(0),
      num_fea_tets(0),
      num_rigid_tet_contacts(0),
      num_rigid_tet_node_contacts(0),
      num_marker_tet_contacts(0),
      nnz_bilaterals(0),
      add_contact_callback(nullptr),
      composition_strategy(new ChMaterialCompositionStrategy<real>) {
    node_container = std::make_shared<Ch3DOFContainer>();
    fea_container = std::make_shared<Ch3DOFContainer>();
    node_container->data_manager = this;
    fea_container->data_manager = this;

    broadphase = new ChCBroadphase;
    narrowphase = new ChCNarrowphaseDispatch;
    aabb_generator = new ChCAABBGenerator;
    broadphase->data_manager = this;
    narrowphase->data_manager = this;
    aabb_generator->data_manager = this;
}

ChParallelDataManager::~ChParallelDataManager() {
    delete narrowphase;
    delete broadphase;
    delete aabb_generator;
}

int ChParallelDataManager::OutputBlazeVector(DynamicVector<real> src, std::string filename) {
    const char* numformat = "%.16g";
    ChStreamOutAsciiFile stream(filename.c_str());
    stream.SetNumFormat(numformat);

    for (int i = 0; i < src.size(); i++)
        stream << src[i] << "\n";

    return 0;
}

int ChParallelDataManager::OutputBlazeMatrix(CompressedMatrix<real> src, std::string filename) {
    const char* numformat = "%.16g";
    ChStreamOutAsciiFile stream(filename.c_str());
    stream.SetNumFormat(numformat);

    stream << src.rows() << " " << src.columns() << "\n";
    for (int i = 0; i < src.rows(); ++i) {
        for (CompressedMatrix<real>::Iterator it = src.begin(i); it != src.end(i); ++it) {
            stream << i << " " << it->index() << " " << it->value() << "\n";
        }
    }

    return 0;
}

int ChParallelDataManager::ExportCurrentSystem(std::string output_dir) {
    int offset = 0;
    if (settings.solver.solver_mode == SolverMode::NORMAL) {
        offset = num_rigid_contacts;
    } else if (settings.solver.solver_mode == SolverMode::SLIDING) {
        offset = 3 * num_rigid_contacts;
    } else if (settings.solver.solver_mode == SolverMode::SPINNING) {
        offset = 6 * num_rigid_contacts;
    }

    // fill in the information for constraints and friction
    DynamicVector<real> fric(num_constraints, -2.0);
    for (unsigned int i = 0; i < num_rigid_contacts; i++) {
        if (settings.solver.solver_mode == SolverMode::NORMAL) {
            fric[i] = host_data.fric_rigid_rigid[i].x;
        } else if (settings.solver.solver_mode == SolverMode::SLIDING) {
            fric[3 * i] = host_data.fric_rigid_rigid[i].x;
            fric[3 * i + 1] = -1;
            fric[3 * i + 2] = -1;
        } else if (settings.solver.solver_mode == SolverMode::SPINNING) {
            fric[6 * i] = host_data.fric_rigid_rigid[i].x;
            fric[6 * i + 1] = -1;
            fric[6 * i + 2] = -1;
            fric[6 * i + 3] = -1;
            fric[6 * i + 4] = -1;
            fric[6 * i + 5] = -1;
        }
    }

    // output r
    std::string filename = output_dir + "dump_r.dat";
    OutputBlazeVector(host_data.R, filename);

    // output b
    filename = output_dir + "dump_b.dat";
    OutputBlazeVector(host_data.b, filename);

    // output friction data
    filename = output_dir + "dump_fric.dat";
    OutputBlazeVector(fric, filename);

    CompressedMatrix<real> D_T;
    uint nnz_total = nnz_bilaterals;

    filename = output_dir + "dump_D.dat";
    OutputBlazeMatrix(host_data.D_T, filename);

    // output M_inv
    filename = output_dir + "dump_Minv.dat";
    OutputBlazeMatrix(host_data.M_inv, filename);

    return 0;
}

void ChParallelDataManager::PrintMatrix(CompressedMatrix<real> src) {
    const char* numformat = "%.16g";
    std::cout << src.rows() << " " << src.columns() << "\n";
    for (int i = 0; i < src.rows(); ++i) {
        std::cout << i << " ";
        for (int j = 0; j < src.columns(); j++) {
            std::cout << src(i, j) << " ";
        }
        std::cout << "\n";
    }
}
