#include "ChDataManager.h"

using namespace chrono;

ChParallelDataManager::ChParallelDataManager()
:	num_contacts(0),
	number_of_models(0),
	number_of_rigid(0),
  num_unilaterals(0),
	num_bilaterals(0),
	erad_is_set(false)
{
}

ChParallelDataManager::~ChParallelDataManager()
{
}
