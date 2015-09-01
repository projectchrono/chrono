/*
Author: Charles Ricchio

Base class for mouse event callbacks
*/

#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

typedef std::function<void(const ChOgreMouseState&)> ChOgreMouseCall;

class CHOGRE_DLL_TAG ChOgreMouseCallback : public ChOgreInputCallback {
  public:
    ChOgreMouseCallback() {}
    ~ChOgreMouseCallback() {}

    ChOgreMouseCall call;

  protected:
  private:
};
}