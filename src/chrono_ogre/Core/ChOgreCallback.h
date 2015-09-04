/*
Author: Charles Ricchio

The base class for giving callback points to ChOgre
*/

#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include <functional>

namespace ChOgre {

typedef std::function<void()> ChOgreCall;

class CHOGRE_DLL_TAG ChOgreCallback {
  public:
    ChOgreCallback() {}
    ~ChOgreCallback() {}

    // virtual void call() {};
    ChOgreCall call;

  protected:
  private:
};
}