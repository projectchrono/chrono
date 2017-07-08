#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include "chrono_ogre/Graphics/ChOgreLight.h"

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreLightHandle {
  public:
    ChOgreLightHandle();
    ChOgreLightHandle(const ChOgreLightHandle& other);
    ChOgreLightHandle(ChOgreLightHandle&& other);
    ChOgreLightHandle(ChOgreLight& Light);
    ChOgreLightHandle(ChOgreLightSharedPtr& LightPtr);
    ~ChOgreLightHandle();

    ChOgreLightHandle& operator=(const ChOgreLightHandle& other);
    ChOgreLightHandle& operator=(ChOgreLightHandle&& other);

    ChOgreLightSharedPtr operator->();
    ChOgreLightSharedPtr light();
    void setLightPtr(ChOgreLightSharedPtr& LightPtr);

  private:
    ChOgreLightSharedPtr m_pLight;
};
}