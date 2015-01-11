/*
Author: Charles Ricchio

The base class for giving callback points to ChOgre
*/

#pragma once

#include "../ChOgre.h"

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreCallback {

	public:

		ChOgreCallback() {}
		~ChOgreCallback() {}

		virtual void call() = 0;

	protected:



	private:



	};

}