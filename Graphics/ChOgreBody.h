/*
Author: Charles Ricchio

ECBody is the basic physical body class. It will set up Ogre scene nodes to display the assets of its ChBody every time refresh() is called.
*/

#pragma once

#include <vector>

#include <OGRE\Ogre.h>

#include <physics\ChSystem.h>
#include <physics\ChBody.h>
#include <core\ChSmartpointers.h>
#include <assets\ChVisualization.h>
#include <assets\ChBoxShape.h>
#include <assets\ChCapsuleShape.h>
#include <assets\ChConeShape.h>
#include <assets\ChCylinderShape.h>
#include <assets\ChEllipsoidShape.h>
#include <assets\ChRoundedBoxShape.h>
#include <assets\ChRoundedConeShape.h>
#include <assets\ChRoundedCylinderShape.h>
#include <assets\ChSphereShape.h>
#include <assets\ChTriangleMeshShape.h>

#include "../ChOgre.h"

namespace EnvironmentCore {

	class CHOGRE_DLL_TAG ECBody {

	public:

		ECBody(Ogre::SceneManager* SceneManager, chrono::ChSystem* System);
		~ECBody();

		virtual void update();
		virtual void refresh();
		virtual void setMesh(Ogre::ManualObject* Mesh, chrono::ChVector<>& Scale = chrono::ChVector<>(1, 1, 1));

		virtual chrono::ChSharedBodyPtr getChBody();

		virtual chrono::ChSharedBodyPtr operator-> (); //Operator magic. Allows a refrence to an ECBody to offer members as an ECBody object, and as a ChBody pointer

		std::string name;
		bool deletable;
		bool isStaticMesh;

	protected:

		chrono::ChSharedBodyPtr m_pBody;

		std::vector<Ogre::SceneNode*> m_SceneNodes;

		Ogre::SceneManager* m_pSceneManager;
		chrono::ChSystem* m_pChSystem;

		static unsigned int m_MeshCount;

	private:



	};

}