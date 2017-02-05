#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

using namespace chrono;
using namespace chrono::collision;

ChSystemParallelDEM::ChSystemParallelDEM(unsigned int max_objects) : ChSystemParallel(max_objects) {
    solver_speed = std::make_shared<ChIterativeSolverParallelDEM>(data_manager);

    data_manager->settings.collision.collision_envelope = 0;

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SystemType::SYSTEM_DEM;

    data_manager->system_timer.AddTimer("ChIterativeSolverParallelDEM_ProcessContact");
}

ChSystemParallelDEM::ChSystemParallelDEM(const ChSystemParallelDEM& other) : ChSystemParallel(other) {
    //// TODO
}

ChBody* ChSystemParallelDEM::NewBody() {
    if (collision_system_type == CollisionSystemType::COLLSYS_PARALLEL)
        return new ChBody(std::make_shared<collision::ChCollisionModelParallel>(), ChMaterialSurfaceBase::DEM);

    return new ChBody(ChMaterialSurfaceBase::DEM);
}

ChBodyAuxRef* ChSystemParallelDEM::NewBodyAuxRef() {
    if (collision_system_type == CollisionSystemType::COLLSYS_PARALLEL)
        return new ChBodyAuxRef(std::make_shared<collision::ChCollisionModelParallel>(), ChMaterialSurfaceBase::DEM);

    return new ChBodyAuxRef(ChMaterialSurfaceBase::DEM);
}

void ChSystemParallelDEM::AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) {
    assert(newbody->GetContactMethod() == ChMaterialSurfaceBase::DEM);

    // Reserve space for material properties for the specified body. Not that the
    // actual data is set in UpdateMaterialProperties().
    data_manager->host_data.mu.push_back(0);
    data_manager->host_data.cohesion_data.push_back(0);
    data_manager->host_data.adhesionMultDMT_data.push_back(0);

    data_manager->host_data.mass_rigid.push_back(0);

    if (data_manager->settings.solver.use_material_properties) {
        data_manager->host_data.elastic_moduli.push_back(real2(0, 0));
        data_manager->host_data.cr.push_back(0);
    } else {
        data_manager->host_data.dem_coeffs.push_back(real4(0, 0, 0, 0));
    }

    if (data_manager->settings.solver.tangential_displ_mode == ChSystemDEM::TangentialDisplacementModel::MultiStep) {
        for (int i = 0; i < max_shear; i++) {
            data_manager->host_data.shear_neigh.push_back(vec3(-1, -1, -1));
            data_manager->host_data.shear_disp.push_back(real3(0, 0, 0));
        }
    }
}

void ChSystemParallelDEM::UpdateMaterialSurfaceData(int index, ChBody* body) {
    custom_vector<real>& mass = data_manager->host_data.mass_rigid;
    custom_vector<real2>& elastic_moduli = data_manager->host_data.elastic_moduli;
    custom_vector<real>& adhesion = data_manager->host_data.cohesion_data;
    custom_vector<real>& adhesionMult = data_manager->host_data.adhesionMultDMT_data;
    custom_vector<real>& mu = data_manager->host_data.mu;
    custom_vector<real>& cr = data_manager->host_data.cr;
    custom_vector<real4>& dem_coeffs = data_manager->host_data.dem_coeffs;

    // Since this function is called in a parallel for loop, we must access the
    // material properties in a thread-safe manner (we cannot use the function
    // ChBody::GetMaterialSurfaceDEM since that returns a copy of the reference
    // counted shared pointer).
    std::shared_ptr<ChMaterialSurfaceBase>& mat = body->GetMaterialSurfaceBase();
    ChMaterialSurfaceDEM* mat_ptr = static_cast<ChMaterialSurfaceDEM*>(mat.get());

    mass[index] = body->GetMass();
    mu[index] = mat_ptr->GetSfriction();
    adhesion[index] = mat_ptr->GetAdhesion();
    adhesionMult[index] = mat_ptr->GetAdhesionMultDMT();

    if (data_manager->settings.solver.use_material_properties) {
        elastic_moduli[index] = real2(mat_ptr->GetYoungModulus(), mat_ptr->GetPoissonRatio());
        cr[index] = mat_ptr->GetRestitution();
    } else {
        dem_coeffs[index] = real4(mat_ptr->GetKn(), mat_ptr->GetKt(), mat_ptr->GetGn(), mat_ptr->GetGt());
    }
}

void ChSystemParallelDEM::Setup() {
    // First, invoke the base class method
    ChSystemParallel::Setup();

    // Ensure that the collision envelope is zero.
    data_manager->settings.collision.collision_envelope = 0;
}

void ChSystemParallelDEM::ChangeCollisionSystem(CollisionSystemType type) {
    ChSystemParallel::ChangeCollisionSystem(type);
    data_manager->settings.collision.collision_envelope = 0;
}

real3 ChSystemParallelDEM::GetBodyContactForce(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_force[index];
}

real3 ChSystemParallelDEM::GetBodyContactTorque(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_torque[index];
}

void ChSystemParallelDEM::PrintStepStats() {
    double timer_solver_setup = data_manager->system_timer.GetTime("ChIterativeSolverParallel_Setup");
    double timer_solver_stab = data_manager->system_timer.GetTime("ChIterativeSolverParallel_Stab");

    std::cout << std::endl;
    std::cout << "System Information" << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout << "  Number of bodies     " << GetNumBodies() << std::endl;
    std::cout << "  Number of contacts   " << GetNumContacts() << std::endl;
    std::cout << "  Number of bilaterals " << GetNumBilaterals() << std::endl;
    std::cout << std::endl;
    std::cout << "Timing Information" << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout << "Simulation time        " << GetTimerStep() << std::endl;
    std::cout << "  Collision detection    " << GetTimerCollision() << std::endl;
    std::cout << "    broad phase            " << GetTimerCollisionBroad() << std::endl;
    std::cout << "    narrow phase           " << GetTimerCollisionNarrow() << std::endl;
    std::cout << "  Update                 " << GetTimerUpdate() << std::endl;
    std::cout << "  Solver                 " << GetTimerSolver() << std::endl;
    std::cout << "    contact force calc     " << GetTimerProcessContact() << std::endl;
    std::cout << "    setup                  " << timer_solver_setup << std::endl;
    std::cout << "    stabilization          " << timer_solver_stab << std::endl;
    std::cout << std::endl;
}
