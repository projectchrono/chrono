#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"
#include "chrono_parallel/collision/ChCollision.h"

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"

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
      num_dof(0),
      num_fea_nodes(0),
      num_fea_tets(0),
      num_rigid_tet_contacts(0),
      num_rigid_tet_node_contacts(0),
      num_marker_tet_contacts(0),
      nnz_bilaterals(0) {
    node_container = new Ch3DOFContainer();
    fea_container = new Ch3DOFContainer();

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
    delete node_container;
    delete fea_container;
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

void ChParallelDataManager::Add3DOFContainer(Ch3DOFContainer* container) {
    delete node_container;
    node_container = container;
}
void ChParallelDataManager::AddFEAContainer(ChFEAContainer* container) {
    delete fea_container;
    fea_container = container;
}
