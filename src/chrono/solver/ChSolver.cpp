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

#include "chrono/solver/ChSolver.h"

namespace chrono {

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'my_enum_mappers', just to avoid avoiding cluttering of the parent class.
class my_enum_mappers : public ChSolver {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::PSOR);
    CH_ENUM_VAL(Type::PSSOR);
    CH_ENUM_VAL(Type::PJACOBI);
    CH_ENUM_VAL(Type::PMINRES);
    CH_ENUM_VAL(Type::BARZILAIBORWEIN);
    CH_ENUM_VAL(Type::APGD);
    CH_ENUM_VAL(Type::SPARSE_LU);
    CH_ENUM_VAL(Type::SPARSE_QR);
    CH_ENUM_VAL(Type::PARDISO_MKL);
    CH_ENUM_VAL(Type::MUMPS);
    CH_ENUM_VAL(Type::GMRES);
    CH_ENUM_VAL(Type::MINRES);
    CH_ENUM_VAL(Type::BICGSTAB);
    CH_ENUM_VAL(Type::CUSTOM);
    CH_ENUM_MAPPER_END(Type);
};

void ChSolver::EnableWrite(bool val, const std::string& frame, const std::string& out_dir) {
    write_matrix = val;
    output_dir = out_dir;
    frame_id = frame;
}

void ChSolver::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSolver>();
    // solver type:
    my_enum_mappers::Type_mapper typemapper;
    Type type = GetType();
    marchive << CHNVP(typemapper(type), "solver_type");
    // serialize all member data:
    marchive << CHNVP(verbose);
}

void ChSolver::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChSolver>();
    // solver type:
    my_enum_mappers::Type_mapper typemapper;
    Type type = GetType();
    marchive >> CHNVP(typemapper(type), "solver_type");
    // stream in all member data:
    marchive >> CHNVP(verbose);
}

}  // end namespace chrono
