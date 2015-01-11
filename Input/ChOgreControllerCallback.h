/*
Author: Charles Ricchio

Base class for controller event callbacks
*/

#pragma once

#include "../ChOgre.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreControllerCallback : public ChOgreInputCallback {

	public:

		ChOgreControllerCallback() {}
		~ChOgreControllerCallback() {}

		virtual void call(const ChOgreControllerState& ControllerState) = 0;

	protected:



	private:



	};

}