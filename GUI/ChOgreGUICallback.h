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

	class CHOGRE_DLL_TAG ChOgreGUIClickCallback : public ChOgreGUICallback {

	public:

		ChOgreGUIClickCallback() {}
		~ChOgreGUIClickCallback() {}

		virtual void call(MyGUI::WidgetPtr Sender) {}

	};

	class CHOGRE_DLL_TAG ChOgreGUIPressCallback : public ChOgreGUICallback {

	public:

		ChOgreGUIPressCallback() {}
		~ChOgreGUIPressCallback() {}

		virtual void call(MyGUI::WidgetPtr Sender, int x, int y, MyGUI::MouseButton Button) {}

	};

	class CHOGRE_DLL_TAG ChOgreGUIReleaseCallback : public ChOgreGUICallback {

	public:

		ChOgreGUIReleaseCallback() {}
		~ChOgreGUIReleaseCallback() {}

		virtual void call(MyGUI::WidgetPtr Sender) {}

	};

}