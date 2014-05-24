#ifndef CHC_NARROWPHASE_R_H
#define CHC_NARROWPHASE_R_H

#include "collision/ChCNarrowphase.h"

namespace chrono {
namespace collision {


class CH_PARALLEL_API ChCNarrowphaseR : public ChCNarrowphase {
public:
	ChCNarrowphaseR() {}

	virtual void Process(ChParallelDataManager* data_container);

	virtual void Update(ChParallelDataManager* data_container)
	{
		//// TODO
	}

private:
	void host_process(ChParallelDataManager* data_container,
	                  uint                   num_potentialCollisions,
	                  custom_vector<uint>&   contact_index,
	                  custom_vector<uint>&   contact_flag);
	void host_count(ChParallelDataManager* data_container,
                    uint                   num_potentialCollisions,
                    custom_vector<uint>&   max_contacts);
};


} // end namespace collision
} // end namespace chrono


#endif

