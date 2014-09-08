#ifndef CHC_NARROWPHASE_H
#define CHC_NARROWPHASE_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {
namespace collision {


class CH_PARALLEL_API ChCNarrowphase {
public:
	         ChCNarrowphase() :  total_possible_contacts(0) {}
	virtual ~ChCNarrowphase() {}

	virtual void Process(ChParallelDataManager* data_container) = 0;
	virtual void Update(ChParallelDataManager* data_container) = 0;

	void SetCollisionEnvelope(const real &envelope) {collision_envelope = envelope;}
	real GetCollisionEnvelope()                     {return collision_envelope;}

protected:
	uint total_possible_contacts;
	real collision_envelope;
};


} // end namespace collision
} // end namespace chrono


#endif

