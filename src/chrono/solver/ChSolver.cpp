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
// enclose macros in local 'ChSolver_Type_enum_mapper', just to avoid avoiding cluttering of the parent class.
class ChSolver_Type_enum_mapper : public ChSolver {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::PSOR);
    CH_ENUM_VAL(Type::PSSOR);
    CH_ENUM_VAL(Type::PJACOBI);
    CH_ENUM_VAL(Type::PMINRES);
    CH_ENUM_VAL(Type::BARZILAIBORWEIN);
    CH_ENUM_VAL(Type::APGD);
    CH_ENUM_VAL(Type::ADMM);
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

void ChSolver::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSolver>();
    // solver type:
    ChSolver_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    archive_out << CHNVP(typemapper(type), "solver_type");
    // serialize all member data:
    archive_out << CHNVP(verbose);
}

void ChSolver::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSolver>();
    // solver type:
    ChSolver_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    archive_in >> CHNVP(typemapper(type), "solver_type");
    // stream in all member data:
    archive_in >> CHNVP(verbose);
}

std::string ChSolver::GetTypeAsString(Type type) {
    switch (type) {
        case Type::PSOR:
            return "PSOR";
        case Type::PSSOR:
            return "PSSOR";
        case Type::PJACOBI:
            return "PJACOBI";
        case Type::PMINRES:
            return "PMINRES";
        case Type::BARZILAIBORWEIN:
            return "BARZILAIBORWEIN";
        case Type::APGD:
            return "APGD";
        case Type::ADMM:
            return "ADMM";
        case Type::SPARSE_LU:
            return "SPARSE_LU";
        case Type::SPARSE_QR:
            return "SPARSE_QR";
        case Type::PARDISO_MKL:
            return "PARDISO_MKL";
        case Type::MUMPS:
            return "MUMPS";
        case Type::GMRES:
            return "GMRES";
        case Type::MINRES:
            return "MINRES";
        case Type::BICGSTAB:
            return "BICGSTAB";
        case Type::CUSTOM:
            return "CUSTOM";
    }

    return "UNKNOWN";
}

}  // end namespace chrono
