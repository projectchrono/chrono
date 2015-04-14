#include "chrono_parallel/ChDataManager.h"
#include "core/ChFileutils.h"
#include "core/ChStream.h"
using namespace chrono;

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
      num_dof(0),
      nnz_bilaterals(0) {
}

ChParallelDataManager::~ChParallelDataManager() {
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
  if (settings.solver.solver_mode == NORMAL) {
    offset = num_rigid_contacts;
  } else if (settings.solver.solver_mode == SLIDING) {
    offset = 3 * num_rigid_contacts;
  } else if (settings.solver.solver_mode == SPINNING) {
    offset = 6 * num_rigid_contacts;
  }

  // fill in the information for constraints and friction
  DynamicVector<real> fric(num_constraints, -2.0);
  for (int i = 0; i < num_rigid_contacts; i++) {
    if (settings.solver.solver_mode == NORMAL) {
      fric[i] = host_data.fric_rigid_rigid[i].x;
    } else if (settings.solver.solver_mode == SLIDING) {
      fric[3 * i] = host_data.fric_rigid_rigid[i].x;
      fric[3 * i + 1] = -1;
      fric[3 * i + 2] = -1;
    } else if (settings.solver.solver_mode == SPINNING) {
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

  // output D_T

  int nnz_normal = 6 * 2 * num_rigid_contacts;
  int nnz_tangential = 6 * 4 * num_rigid_contacts;
  int nnz_spinning = 6 * 3 * num_rigid_contacts;

  int num_normal = 1 * num_rigid_contacts;
  int num_tangential = 2 * num_rigid_contacts;
  int num_spinning = 3 * num_rigid_contacts;

  CompressedMatrix<real> D_T;
  uint nnz_total = nnz_bilaterals;

  switch (settings.solver.solver_mode) {
    case NORMAL: {
      nnz_total += nnz_normal;
      D_T.reserve(nnz_total);
      D_T.resize(num_constraints, num_dof, false);
      SubMatrixType D_n_T_sub = blaze::submatrix(D_T, 0, 0, num_rigid_contacts, num_dof);
      D_n_T_sub = host_data.D_n_T;
    } break;
    case SLIDING: {
      nnz_total += nnz_normal + num_tangential;
      D_T.reserve(nnz_total);
      D_T.resize(num_constraints, num_dof, false);

      SubMatrixType D_n_T_sub = blaze::submatrix(D_T, 0, 0, num_rigid_contacts, num_dof);
      D_n_T_sub = host_data.D_n_T;

      SubMatrixType D_t_T_sub = blaze::submatrix(D_T, num_rigid_contacts, 0, 2 * num_rigid_contacts, num_dof);
      D_t_T_sub = host_data.D_t_T;
    } break;
    case SPINNING: {
      nnz_total += nnz_normal + num_tangential + num_spinning;
      D_T.reserve(nnz_total);
      D_T.resize(num_constraints, num_dof, false);

      SubMatrixType D_n_T_sub = blaze::submatrix(D_T, 0, 0, num_rigid_contacts, num_dof);
      D_n_T_sub = host_data.D_n_T;

      SubMatrixType D_t_T_sub = blaze::submatrix(D_T, num_rigid_contacts, 0, 2 * num_rigid_contacts, num_dof);
      D_t_T_sub = host_data.D_t_T;

      SubMatrixType D_s_T_sub = blaze::submatrix(D_T, 3 * num_rigid_contacts, 0, 3 * num_rigid_contacts, num_dof);
      D_s_T_sub = host_data.D_t_T;
    } break;
  }

  SubMatrixType D_b_T = blaze::submatrix(D_T, num_unilaterals, 0, num_bilaterals, num_dof);
  D_b_T = host_data.D_b_T;

  filename = output_dir + "dump_D.dat";
  OutputBlazeMatrix(D_T, filename);

  // output M_inv
  filename = output_dir + "dump_Minv.dat";
  OutputBlazeMatrix(host_data.M_inv, filename);

  return 0;
}
