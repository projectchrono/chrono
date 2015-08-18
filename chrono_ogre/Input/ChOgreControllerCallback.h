/*
Author: Charles Ricchio

Base class for controller event callbacks
*/

#pragma once

#include "../ChOgre.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

	typedef std::function<void(const ChOgreControllerState&)> ChOgreControllerCall;

	class CHOGRE_DLL_TAG ChOgreControllerCallback : public ChOgreInputCallback {

	public:

		ChOgreControllerCallback() {}
		~ChOgreControllerCallback() {}

		ChOgreControllerCall call;

	protected:



	private:



	};

}