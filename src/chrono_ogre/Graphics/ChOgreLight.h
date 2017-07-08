#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include <Ogre.h>
#include <core/ChVector.h>
#include <assets/ChColor.h>
#include <memory>

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreLight {
  public:
    enum LightTypes { POINT = 0, DIRECTIONAL, SPOTLIGHT };

  public:
    ChOgreLight(Ogre::SceneManager* SceneManager);
    ChOgreLight(Ogre::SceneManager* SceneManager, const std::string& Name);
    ~ChOgreLight();

    void setType(LightTypes Type);

    void setDiffuse(const chrono::ChColor& color);
    void setDiffuse(float r, float g, float b);

	void setSpecular(const chrono::ChColor& color);
    void setSpecular(float r, float g, float b);

	void setPosition(const chrono::ChVector<>& position);
    void setPosition(float x, float y, float z);

    void setDirection(const chrono::ChVector<>& direction);
    void setDirection(float x, float y, float z);

    void pointAt(const chrono::ChVector<>& position);
    void pointAt(float x, float y, float z);

    void setIntensity(float i);

    LightTypes getType();
	chrono::ChColor getDiffuse();
	chrono::ChColor getSpecular();
    chrono::ChVector<> getPosition();
    chrono::ChVector<> getDirection();
    float getIntensity();
    std::string getName();

  protected:
    Ogre::Light* m_pLight;

  private:
};

typedef ChOgreLight* ChOgreLightPtr;
typedef std::shared_ptr<ChOgreLight> ChOgreLightSharedPtr;
}