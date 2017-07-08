#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include <core/ChVector.h>
#include <Ogre.h>
#include <cmath>

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreCamera {
  public:
    ChOgreCamera(Ogre::Camera* Camera);
    ChOgreCamera(const ChOgreCamera& rhs);
    ChOgreCamera(ChOgreCamera&& lhs);
    ~ChOgreCamera();

    ChOgreCamera& operator=(const ChOgreCamera& rhs);
    ChOgreCamera& operator=(ChOgreCamera&& lhs);

    void lookAt(float x, float y, float z);
    void lookAt(const chrono::ChVector<>& dir);
    void orient(float pitch, float yaw);

    void setPosition(float x, float y, float z);
    void setPosition(const chrono::ChVector<>& position);

    void setName(const std::string& Name);

    Ogre::Camera* getCamera() { return m_pCamera; }
    chrono::ChVector<> getPosition() {
        return chrono::ChVector<>(m_pCamera->getPosition().x, m_pCamera->getPosition().y, m_pCamera->getPosition().z);
    }
    const std::string& getName() { return m_pCamera->getName(); }

  protected:
    Ogre::Camera* m_pCamera;

  private:
};
}