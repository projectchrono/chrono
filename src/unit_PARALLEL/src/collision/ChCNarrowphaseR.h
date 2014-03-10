#ifndef CHC_NARROWPHASE_R_H
#define CHC_NARROWPHASE_R_H

#include "collision/ChCNarrowphase.h"

namespace chrono {
namespace collision {


class ChApiGPU ChCNarrowphaseR : public ChCNarrowphase {
public:
	ChCNarrowphaseR() {}

	virtual void Process(ChParallelDataManager* data_container);

	virtual void Update(ChParallelDataManager* data_container)
	{
		//// TODO
	}

private:
	void host_process(ChParallelDataManager* data_container,
	                  custom_vector<uint>&   contact_active);
};


} // end namespace collision
} // end namespace chrono


#endif

