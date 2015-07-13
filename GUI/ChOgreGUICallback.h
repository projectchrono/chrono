#pragma once

#include "../Core/ChOgreCallback.h"
#include <MYGUI/MyGUI.h>

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreGUICallback : public ChOgreCallback {

	public:

		ChOgreGUICallback() {}
		~ChOgreGUICallback() {}

		virtual void call(MyGUI::WidgetPtr Sender) {}

	};

}