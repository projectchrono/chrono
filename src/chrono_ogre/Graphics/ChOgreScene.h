/*
Author: Charles Ricchio

ChOgreScene is designed to be a layer of abstraction from the Ogre lighting system, as well as a way of creating physics
bodies for actual simulation
*/

#pragma once

#include <Ogre.h>
#include <physics/ChSystem.h>

#include "ChOgreBody.h"
#include "ChOgreSprite.h"
#include "chrono_ogre/Util/ChOgreBodyHandle.h"
#include "chrono_ogre/Util/ChOgreLightHandle.h"

namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreScene {

  public:

    ChOgreScene(Ogre::SceneManager* SceneManager, chrono::ChSystem* System);
    ~ChOgreScene();

	////////
	// Sprite Management
	////////

	ChOgreSprite* createSprite();
	void removeSprite(ChOgreSprite* Sprite);

    ////////
    // Lighting Abstration
    ////////

    void setAmbientLight(Ogre::ColourValue Color);
    void setAmbientLight(float r, float g, float b);

    ChOgreLightHandle createLight();
    ChOgreLightHandle createLight(const std::string& Name);

    void removeLight(ChOgreLightHandle Light);
    void removeLight(const std::string& Name);

    ////////
    // Body Creation
    ////////

    ChOgreBodyHandle createBody(const std::string& Name = "");

    ChOgreBodyHandle getBody(const std::string& Name);

    void removeBody(ChOgreBodyHandle& Body);
    void removeBody(const std::string& Name);

	void update();

    ////////
    // Convenience functions
    ////////

    ChOgreBodyHandle spawnBox(const std::string& Name = "",
                                      double mass = 1.0,
                                      const chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
                                      const chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
                                      const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
                                      bool fixed = false);

    ChOgreBodyHandle spawnCapsule(const std::string& Name = "");  // TODO: Actually implement the capsule

    ChOgreBodyHandle spawnCone(const std::string& Name = "",
                                       double mass = 1.0,
                                       const chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
                                       const chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
                                       const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
                                       bool fixed = false);

    ChOgreBodyHandle spawnCylinder(const std::string& Name = "",
                                           double mass = 1.0,
                                           const chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
                                           const chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
                                           const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
                                           bool fixed = false);

    ChOgreBodyHandle spawnEllipsoid(const std::string& Name = "",
                                            double mass = 1.0,
                                            const chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
                                            const chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
                                            const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
                                            bool fixed = false);

    ChOgreBodyHandle spawnSphere(const std::string& Name = "",
                                         double mass = 1.0,
                                         const chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
                                         double radius = 1.0,
                                         bool fixed = false);

    ChOgreBodyHandle spawnMesh(const std::string& Name = "",
                                       double mass = 1.0,
                                       const chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
                                       const chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
                                       const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
                                       const std::string& FileName = "",
                                       const std::string& Path = "",
                                       bool fixed = false);

    ////////
    // Various world management functions
    ////////

    void setLowerLimit(double height);

    double getLowerLimit();

    void setSkyBox(const std::string& FilePath);
    void disableSkyBox();

    ChOgreBodyHandle loadHeightMap(const std::string& FilePath,
                                           const chrono::ChVector<>& Scale = chrono::ChVector<>(1, 1, 1));

  protected:
    Ogre::SceneManager* m_pSceneManager;

    chrono::ChSystem* m_pChSystem;

    ////////
    // Body Management/Storage
    ////////

    std::vector<ChOgreBodySharedPtr> m_ChOgreBodies;

	////////
	// Billboard/Sprite Management Space
	////////

	std::vector<ChOgreSprite*> m_ChOgreSprites;

    double m_LowerLimit;

    static unsigned int m_LightCount;
    static unsigned int m_TerrainMeshCount;

  private:
};
}
}
