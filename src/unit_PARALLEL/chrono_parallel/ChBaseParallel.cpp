#include "chrono_parallel/ChBaseParallel.h"
using namespace chrono;

void ChBaseParallel::Initialize() {
   num_bodies = data_container->num_bodies;
   num_contacts = data_container->num_contacts;
   num_unilaterals = data_container->num_unilaterals;
   num_bilaterals = data_container->num_bilaterals;
   num_constraints = data_container->num_constraints;

   step_size = data_container->settings.solver.step_size;
   tol_speed = step_size * data_container->settings.solver.tolerance;
   alpha = data_container->settings.solver.alpha;
   contact_recovery_speed = data_container->settings.solver.contact_recovery_speed;
}
