/*
Author: Charles Ricchio

ChOgreScene is designed to be a layer of abstraction from the Ogre lighting system, as well as a way of creating physics bodies for actual simulation
*/

#pragma once

#include <OGRE\Ogre.h>
#include <physics\ChSystem.h>

#include "ChOgreBody.h"

namespace ChOgre {

	typedef Ogre::Light CHOGRE_DLL_TAG ChOgreLight;

	typedef Ogre::Light::LightTypes CHOGRE_DLL_TAG ChOgreLightTypes;

	class CHOGRE_DLL_TAG ChOgreScene {

	public:

		ChOgreScene(Ogre::SceneManager* SceneManager, chrono::ChSystem* System);
		~ChOgreScene();

		////////
		//Lighting Abstration
		///////

		virtual void setAmbientLight(Ogre::ColourValue Color);
		virtual void setAmbientLight(float r, float g, float b);

		virtual ChOgreLight& createLight();
		virtual ChOgreLight& createLight(std::string Name);

		virtual void removeLight(ChOgreLight& Light);
		virtual void removeLight(std::string Name);

		////////
		//Body Creation
		///////

		virtual ChOgreBody& createBody(std::string Name="");

		virtual ChOgreBody& getBody(std::string Name);

		virtual void removeBody(ChOgreBody& Body);
		virtual void removeBody(std::string Name);

		virtual void update();

		////////
		//Convenience functions
		////////

		virtual ChOgreBody& spawnBox(	std::string Name = "", 
									double mass = 1.0, 
									chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
									chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1), 
									chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0), 
									bool fixed = false);

		virtual ChOgreBody& spawnCapsule(std::string Name = ""); // TODO: Actually implement the capsule

		virtual ChOgreBody& spawnCone(	std::string Name = "", 
									double mass = 1.0, 
									chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0), 
									chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
									chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0), 
									bool fixed = false);

		virtual ChOgreBody& spawnCylinder(	std::string Name = "", 
										double mass = 1.0, 
										chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0), 
										chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1), 
										chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
										bool fixed = false);

		virtual ChOgreBody& spawnEllipsoid(	std::string Name = "", 
										double mass = 1.0, 
										chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
										chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1), 
										chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0), 
										bool fixed = false);

		virtual ChOgreBody& spawnSphere(	std::string Name = "", 
										double mass = 1.0, 
										chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0), 
										double radius = 1.0,
										bool fixed = false);

		virtual ChOgreBody& spawnMesh(	std::string Name = "",
									double mass = 1.0,
									chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
									chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
									chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
									std::string FileName = "",
									std::string Path = "",
									bool fixed = false);

		////////
		//Various world management functions
		////////

		virtual void setLowerLimit(double height);

		virtual double getLowerLimit();

		virtual void setSkyBox(std::string FilePath);
		virtual void disableSkyBox();

		virtual ChOgreBody& loadHeightMap(std::string FilePath, chrono::ChVector<>& Scale = chrono::ChVector<>(1, 1, 1));

	protected:

		Ogre::SceneManager* m_pSceneManager;

		chrono::ChSystem* m_pChSystem;


		////////
		//Body Management/Storage
		////////

		std::vector<ChOgreBody*> m_ChOgreBodies;

		double m_LowerLimit;

		static unsigned int m_LightCount;
		static unsigned int m_TerrainMeshCount;

	private:



	};

}