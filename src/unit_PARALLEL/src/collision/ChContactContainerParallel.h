#ifndef CHCONTACTCONTAINERGPUSIMPLE_H
#define CHCONTACTCONTAINERGPUSIMPLE_H

///////////////////////////////////////////////////
//
//   ChContactContainerGPUsimple.h
//
//   Class for container of many contacts, as CPU
//   typical linked list of ChContactGPUsimple objects
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "physics/ChContactContainer.h"
#include <list>
#include "ChApiParallel.h"
#include "ChDataManager.h"

namespace chrono {
/// Class representing a container of many contacts,
/// implemented as a typical linked list of ChContactGPUsimple
/// objects.
/// This contact container must be used for the preliminar CUDA solver
/// that was developed by Ale & Dan, but in future will be
/// replaced by ChContactContainerGPU, and advanced container
/// that does not use linked lists of cpu objects but rather
/// keeps all contact data as GPU buffers on the GPU device.

class CH_PARALLEL_API ChContactContainerParallel: public ChContactContainer {
	CH_RTTI(ChContactContainerParallel, ChContactContainer)
		;

	protected:

	public:

		ChContactContainerParallel();

		virtual ~ChContactContainerParallel();
		int GetNcontacts() {return data_container->num_contacts;}

		ChParallelDataManager* data_container;
};


}

#endif

