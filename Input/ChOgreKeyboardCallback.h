/*
Author: Charles Ricchio

Base class for keyboard event callbacks
*/

#pragma once

#include "../ChOgre.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreKeyboardCallback : public ChOgreInputCallback {

	public:

		ChOgreKeyboardCallback() {}
		~ChOgreKeyboardCallback() {}

		virtual void call(keycode_t KeyCode, const ChOgreKeyState& KeyState) = 0;

	protected:



	private:



	};

}