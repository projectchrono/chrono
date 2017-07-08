/*
Author: Charles Ricchio

Base class for keyboard event callbacks
*/

#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

typedef std::function<void(scancode_t, keycode_t, const ChOgreKeyState&)> ChOgreKeyboardCall;

class CHOGRE_DLL_TAG ChOgreKeyboardCallback : public ChOgreInputCallback {
  public:
    ChOgreKeyboardCallback() {}
    ~ChOgreKeyboardCallback() {}

    ChOgreKeyboardCall call;

  protected:
  private:
};
}