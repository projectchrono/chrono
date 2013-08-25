#include "ChBaseGPU.h"
using namespace chrono;

void ChBaseGPU::Initialize() {
	number_of_objects = data_container->number_of_objects;
	number_of_contacts = data_container->number_of_contacts;
	number_of_bilaterals = data_container->number_of_bilaterals;
	number_of_constraints = data_container->number_of_contacts * 3 + data_container->number_of_bilaterals;

	step_size = data_container->step_size;
//	compliance = data_container->compliance;
//	complianceT = data_container->complianceT;
	alpha = data_container->alpha;
	contact_recovery_speed = data_container->contact_recovery_speed;

	inv_hpa = 1.0 / (step_size + alpha);
	inv_hhpa = 1.0 / (step_size * (step_size + alpha));

}
