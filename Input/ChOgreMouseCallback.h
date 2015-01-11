/*
Author: Charles Ricchio

Base class for mouse event callbacks
*/

#pragma once

#include "../ChOgre.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreMouseCallback : public ChOgreInputCallback {

	public:

		ChOgreMouseCallback() {}
		~ChOgreMouseCallback() {}

		virtual void call(const ChOgreMouseState& MouseState) = 0;
		
	protected:



	private:



	};

}