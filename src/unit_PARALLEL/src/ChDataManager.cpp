#include "ChDataManager.h"
using namespace chrono;
ChGPUDataManager::ChGPUDataManager() {
	number_of_rigid_rigid = 0;
	number_of_models = 0;
	number_of_rigid = 0;
	number_of_bilaterals = 0;
	number_of_contacts_possible = 0;
	copyContacts = false;
}
ChGPUDataManager::~ChGPUDataManager() {
}
