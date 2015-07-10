#pragma once

#include "../ChOgre.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreWindowCallback : public ChOgreInputCallback {

	public:

		ChOgreWindowCallback() {}
		~ChOgreWindowCallback() {}

		void call() {}

	};

}