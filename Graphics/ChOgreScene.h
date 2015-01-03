/*
Author: Charles Ricchio

ECScene is designed to be a layer of abstraction from the Ogre lighting system, as well as a way of creating physics bodies for actual simulation
*/

#pragma once

#include <OGRE\Ogre.h>
#include <physics\ChSystem.h>

#include "ECBody.h"

namespace EnvironmentCore {

	typedef Ogre::Light ECLight;

	typedef Ogre::Light::LightTypes ECLightTypes;

	class ECScene {

	public:

		ECScene(Ogre::SceneManager* SceneManager, chrono::ChSystem* System);
		~ECScene();

		////////
		//Lighting Abstration
		///////

		virtual void setAmbientLight(Ogre::ColourValue Color);
		virtual void setAmbientLight(float r, float g, float b);

		virtual ECLight& createLight();
		virtual ECLight& createLight(std::string Name);

		virtual void removeLight(ECLight& Light);
		virtual void removeLight(std::string Name);

		////////
		//Body Creation
		///////

		virtual ECBody& createBody(std::string Name="");

		virtual ECBody& getBody(std::string Name);

		virtual void removeBody(ECBody& Body);
		virtual void removeBody(std::string Name);

		virtual void update();

		////////
		//Convenience functions
		////////

		virtual ECBody& spawnBox(	std::string Name = "", 
									double mass = 1.0, 
									chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
									chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1), 
									chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0), 
									bool fixed = false);

		virtual ECBody& spawnCapsule(std::string Name = ""); // TODO: Actually implement the capsule

		virtual ECBody& spawnCone(	std::string Name = "", 
									double mass = 1.0, 
									chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0), 
									chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1),
									chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0), 
									bool fixed = false);

		virtual ECBody& spawnCylinder(	std::string Name = "", 
										double mass = 1.0, 
										chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0), 
										chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1), 
										chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0),
										bool fixed = false);

		virtual ECBody& spawnEllipsoid(	std::string Name = "", 
										double mass = 1.0, 
										chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0),
										chrono::ChVector<>& size = chrono::ChVector<>(1, 1, 1), 
										chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1, 0, 0, 0), 
										bool fixed = false);

		virtual ECBody& spawnSphere(	std::string Name = "", 
										double mass = 1.0, 
										chrono::ChVector<>& position = chrono::ChVector<>(0, 0, 0), 
										double radius = 1.0,
										bool fixed = false);

		virtual ECBody& spawnMesh(	std::string Name = "",
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

		virtual ECBody& loadHeightMap(std::string FilePath, chrono::ChVector<>& Scale = chrono::ChVector<>(1, 1, 1));

	protected:

		Ogre::SceneManager* m_pSceneManager;

		chrono::ChSystem* m_pChSystem;


		////////
		//Body Management/Storage
		////////

		std::vector<ECBody*> m_ECBodies;

		double m_LowerLimit;

		static unsigned int m_LightCount;
		static unsigned int m_TerrainMeshCount;

	private:



	};

}