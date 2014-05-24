#include "ChDataManager.h"

using namespace chrono;

ChParallelDataManager::ChParallelDataManager()
: num_contacts(0),
  num_models(0),
  num_bodies(0),
  num_unilaterals(0),
  num_bilaterals(0),
  num_constraints(0),
  erad_is_set(false)
{
}

ChParallelDataManager::~ChParallelDataManager()
{
}
