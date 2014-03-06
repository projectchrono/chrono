#ifndef CHC_NARROWPHASE_H
#define CHC_NARROWPHASE_H

#include "ChParallelDefines.h"
#include "ChDataManager.h"

namespace chrono {
namespace collision {


class ChApiGPU ChCNarrowphase {
public:
	         ChCNarrowphase() : collision_envelope(0), total_possible_contacts(0) {}
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

