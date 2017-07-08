#pragma once

#include "chrono_ogre/Core/ChOgreCallback.h"
#include <MYGUI/MyGUI.h>

namespace ChOgre {

typedef std::function<void(MyGUI::WidgetPtr)> ChOgreGUICall;
typedef std::function<void(MyGUI::WidgetPtr, int, int, MyGUI::MouseButton)> ChOgreGUIMouseButtonCall;

class CHOGRE_DLL_TAG ChOgreGUICallback : public ChOgreCallback {
  public:
    ChOgreGUICallback() {}
    ~ChOgreGUICallback() {}

    ChOgreGUICall call;

  private:
    friend class ChOgreGUIElement;

    void _c(MyGUI::WidgetPtr w) { call(w); }
};

class CHOGRE_DLL_TAG ChOgreGUIClickCallback : public ChOgreGUICallback {
  public:
    ChOgreGUIClickCallback() {}
    ~ChOgreGUIClickCallback() {}

    ChOgreGUICall call;

  private:
    friend class ChOgreGUIButton;

    void _c(MyGUI::WidgetPtr w) { call(w); }
};

class CHOGRE_DLL_TAG ChOgreGUIPressCallback : public ChOgreGUICallback {
  public:
    ChOgreGUIPressCallback() {}
    ~ChOgreGUIPressCallback() {}

    ChOgreGUIMouseButtonCall call;

  private:
    friend class ChOgreGUIButton;

    void _c(MyGUI::WidgetPtr w, int x, int y, MyGUI::MouseButton b) { call(w, x, y, b); }
};

class CHOGRE_DLL_TAG ChOgreGUIReleaseCallback : public ChOgreGUICallback {
  public:
    ChOgreGUIReleaseCallback() {}
    ~ChOgreGUIReleaseCallback() {}

    ChOgreGUIMouseButtonCall call;

  private:
    friend class ChOgreGUIButton;

    void _c(MyGUI::WidgetPtr w, int x, int y, MyGUI::MouseButton b) { call(w, x, y, b); }
};
}