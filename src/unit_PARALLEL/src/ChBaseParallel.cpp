#include "ChBaseParallel.h"
using namespace chrono;

void ChBaseParallel::Initialize() {
   number_of_rigid = data_container->num_bodies;
   number_of_rigid_rigid = data_container->num_contacts;
   num_unilaterals = data_container->num_unilaterals;
   num_bilaterals = data_container->num_bilaterals;
   number_of_constraints = data_container->num_unilaterals + data_container->num_bilaterals;

   number_of_updates = 0;

   step_size = data_container->step_size;
   alpha = data_container->alpha;
   contact_recovery_speed = data_container->contact_recovery_speed;

   inv_hpa = 1.0 / (step_size + alpha);
   inv_hhpa = 1.0 / (step_size * (step_size + alpha));
}
