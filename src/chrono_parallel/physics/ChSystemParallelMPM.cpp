#include "chrono_parallel/physics/ChSystemParallel.h"

using namespace chrono;

ChSystemParallelMPM::ChSystemParallelMPM(unsigned int max_objects) : ChSystemParallel(max_objects) {
    LCP_solver_speed = new ChLcpSolverParallelDVI(data_manager);

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SYSTEM_DVI;

    data_manager->system_timer.AddTimer("ChSolverParallel_solverA");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverB");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverC");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverD");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverE");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverF");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverG");
    data_manager->system_timer.AddTimer("ChSolverParallel_Project");
    data_manager->system_timer.AddTimer("ChSolverParallel_Solve");
    data_manager->system_timer.AddTimer("ShurProduct");
    data_manager->system_timer.AddTimer("ChLcpSolverParallel_D");
    data_manager->system_timer.AddTimer("ChLcpSolverParallel_E");
    data_manager->system_timer.AddTimer("ChLcpSolverParallel_R");
    data_manager->system_timer.AddTimer("ChLcpSolverParallel_N");
}

ChBody* ChSystemParallelMPM::NewBody() {
    if (collision_system_type == COLLSYS_PARALLEL)
        return new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI);

    return new ChBody(ChMaterialSurfaceBase::DVI);
}

void ChSystemParallelMPM::AddMaterialSurfaceData(ChSharedPtr<ChBody> newbody) {
    assert(newbody->GetContactMethod() == ChMaterialSurfaceBase::DVI);

    // Reserve space for material properties for the specified body. Not that the
    // actual data is set in UpdateMaterialProperties().
    data_manager->host_data.fric_data.push_back(real3(0));
    data_manager->host_data.cohesion_data.push_back(0);
    data_manager->host_data.compliance_data.push_back(real4(0));
}

void ChSystemParallelMPM::UpdateMaterialSurfaceData(int index, ChBody* body) {
    custom_vector<real>& cohesion = data_manager->host_data.cohesion_data;
    custom_vector<real3>& friction = data_manager->host_data.fric_data;
    custom_vector<real4>& compliance = data_manager->host_data.compliance_data;

    // Since this function is called in a parallel for loop, we must access the
    // material properties in a thread-safe manner (we cannot use the function
    // ChBody::GetMaterialSurface since that returns a copy of the reference
    // counted shared pointer).
    ChSharedPtr<ChMaterialSurfaceBase>& mat = body->GetMaterialSurfaceBase();
    ChMaterialSurface* mat_ptr = static_cast<ChMaterialSurface*>(mat.get_ptr());

    friction[index] = real3(mat_ptr->GetKfriction(), mat_ptr->GetRollingFriction(), mat_ptr->GetSpinningFriction());
    cohesion[index] = mat_ptr->GetCohesion();
    compliance[index] = real4(mat_ptr->GetCompliance(), mat_ptr->GetComplianceT(), mat_ptr->GetComplianceRolling(),
                              mat_ptr->GetComplianceSpinning());
}

void ChSystemParallelMPM::CalculateContactForces() {}
real3 ChSystemParallelMPM::GetBodyContactForce(uint body_id) const {
    return real3(0);
}
real3 ChSystemParallelMPM::GetBodyContactTorque(uint body_id) const {
    return real3(0);
}
