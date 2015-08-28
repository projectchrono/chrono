/*
Author: Charles Ricchio

A callback for user input
*/

#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include "chrono_ogre/Core/ChOgreCallback.h"
#include "ChOgreInputDataStructures.h"

namespace ChOgre {

typedef std::function<void()> ChOgreInputCall;

class CHOGRE_DLL_TAG ChOgreInputCallback : public ChOgreCallback {
  public:
    ChOgreInputCallback() {}
    ~ChOgreInputCallback() {}

    ChOgreInputCall call;

  protected:
  private:
};
}