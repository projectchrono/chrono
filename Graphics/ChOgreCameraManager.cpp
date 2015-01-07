/*
Author: Charles Ricchio

All the defines for the ECCameraManager
*/

#include "ChOgreCameraManager.h"

namespace EnvironmentCore {

	unsigned int ECCameraManager::g_CameraCount = 0;

	ECCameraManager::ECCameraManager() {
	}


	ECCameraManager::~ECCameraManager() {
		for (unsigned int i = 0; i < m_CameraList.size(); i++) {
			if (m_CameraList[i]) {
				delete m_CameraList[i];
			}
		}
	}

	ECCamera* ECCameraManager::createCamera(std::string Name) {
		ECCamera* l_pCamera = new ECCamera;
		l_pCamera->name = Name;
		m_CameraList.push_back(l_pCamera);

		g_CameraCount = m_CameraList.size();

		return l_pCamera;
	}

	ECCamera* ECCameraManager::getCamera(unsigned int iterator) {
		return m_CameraList[iterator];
	}

	ECCamera* ECCameraManager::getCamera(std::string Name) {
		for (unsigned int i = 0; i < m_CameraList.size(); i++) {
			if (m_CameraList[i]->name == Name) {
				return m_CameraList[i];
			}
		}
		return nullptr;
	}

	ECCamera* ECCameraManager::operator[] (unsigned int iterator) {
		return getCamera(iterator);
	}

	ECCamera* ECCameraManager::operator[] (std::string Name) {
		return getCamera(Name);
	}

}