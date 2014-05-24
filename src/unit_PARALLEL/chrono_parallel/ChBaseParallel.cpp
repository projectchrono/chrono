#include "chrono_parallel/ChBaseParallel.h"
using namespace chrono;

void ChBaseParallel::Initialize() {
   num_bodies = data_container->num_bodies;
   num_contacts = data_container->num_contacts;
   num_unilaterals = data_container->num_unilaterals;
   num_bilaterals = data_container->num_bilaterals;
   num_constraints = data_container->num_constraints;

   step_size = data_container->step_size;
   alpha = data_container->alpha;
   contact_recovery_speed = data_container->contact_recovery_speed;
}
