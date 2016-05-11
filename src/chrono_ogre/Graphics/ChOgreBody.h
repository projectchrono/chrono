/*
Author: Charles Ricchio

ChOgreBody is the basic physical body class. It will set up Ogre scene nodes to display the assets of its ChBody every
time refresh() is called.
*/

#pragma once

#include <vector>
#include <memory>

#include <Ogre.h>

#include <physics/ChSystem.h>
#include <physics/ChBody.h>
#include <core/ChSmartpointers.h>
#include <assets/ChVisualization.h>
#include <assets/ChBoxShape.h>
#include <assets/ChCapsuleShape.h>
#include <assets/ChConeShape.h>
#include <assets/ChCylinderShape.h>
#include <assets/ChEllipsoidShape.h>
#include <assets/ChRoundedBoxShape.h>
#include <assets/ChRoundedConeShape.h>
#include <assets/ChRoundedCylinderShape.h>
#include <assets/ChSphereShape.h>
#include <assets/ChTriangleMeshShape.h>
#include <assets/ChTexture.h>

#include "chrono_ogre/ChOgreApi.h"
#include "ChOgreModel.h"

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreBody {
  public:
    ChOgreBody(Ogre::SceneManager* SceneManager, chrono::ChSystem* System);
    ~ChOgreBody();

    virtual void update();
    virtual void refresh();
    virtual void setMesh(Ogre::ManualObject* Mesh, const chrono::ChVector<>& Scale = chrono::ChVector<>(1, 1, 1));

    virtual chrono::ChSharedPtr<ChBody> getChBody();

    virtual chrono::ChSharedPtr<ChBody> operator->();  // Operator magic. Allows a refrence to an ChOgreBody to offer
                                                       // members as an ChOgreBody object, and as a ChBody pointer

    std::string name;
    bool deletable;
    bool isStaticMesh;

  protected:
    chrono::ChSharedPtr<ChBody> m_pBody;

    std::vector<ChOgreModel> m_Models;

    Ogre::SceneManager* m_pSceneManager;
    chrono::ChSystem* m_pChSystem;

    static unsigned int m_MeshCount;

  private:
};

typedef ChOgreBody* ChOgreBodyPtr;
typedef std::shared_ptr<ChOgreBody> ChOgreBodySharedPtr;
}
