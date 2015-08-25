/*
Author: Charles Ricchio

A callback for user input
*/

#pragma once

#include "../ChOgre.h"
#include "../Core/ChOgreCallback.h"
#include "ChOgreInputDataStructures.h"

namespace ChOgre {

	typedef std::function<void()> ChOgreInputCall;

	class CHOGRE_DLL_TAG ChOgreInputCallback : public ChOgreCallback {

	public:

		ChOgreInputCallback() {}
		~ChOgreInputCallback() {}

		ChOgreInputCall call;

	protected:



	private:



	};

}