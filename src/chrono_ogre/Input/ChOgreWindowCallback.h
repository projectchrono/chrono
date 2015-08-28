#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include "ChOgreInputCallback.h"

namespace ChOgre {

typedef std::function<void()> ChOgreWindowCall;

class CHOGRE_DLL_TAG ChOgreWindowCallback : public ChOgreInputCallback {
  public:
    ChOgreWindowCallback() {}
    ~ChOgreWindowCallback() {}

    ChOgreWindowCall call;
};
}