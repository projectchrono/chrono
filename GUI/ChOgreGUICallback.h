#pragma once

#include "../Core/ChOgreCallback.h"
#include <MYGUI/MyGUI.h>

namespace ChOgre {

	class ChOgreGUICallback : public ChOgreCallback {

	public:

		ChOgreGUICallback() {}
		~ChOgreGUICallback() {}

		virtual void call(MyGUI::WidgetPtr Sender) = 0;

	};

}