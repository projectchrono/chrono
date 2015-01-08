/*
Author: Charles Ricchio

All the defines for the ChOgreCameraManager
*/

#include "ChOgreCameraManager.h"

namespace ChOgre {

	unsigned int ChOgreCameraManager::g_CameraCount = 0;

	ChOgreCameraManager::ChOgreCameraManager() {
	}


	ChOgreCameraManager::~ChOgreCameraManager() {
		for (unsigned int i = 0; i < m_CameraList.size(); i++) {
			if (m_CameraList[i]) {
				delete m_CameraList[i];
			}
		}
	}

	ChOgreCamera* ChOgreCameraManager::createCamera(std::string Name) {
		ChOgreCamera* l_pCamera = new ChOgreCamera;
		l_pCamera->name = Name;
		m_CameraList.push_back(l_pCamera);

		g_CameraCount = m_CameraList.size();

		return l_pCamera;
	}

	ChOgreCamera* ChOgreCameraManager::getCamera(unsigned int iterator) {
		return m_CameraList[iterator];
	}

	ChOgreCamera* ChOgreCameraManager::getCamera(std::string Name) {
		for (unsigned int i = 0; i < m_CameraList.size(); i++) {
			if (m_CameraList[i]->name == Name) {
				return m_CameraList[i];
			}
		}
		return nullptr;
	}

	ChOgreCamera* ChOgreCameraManager::operator[] (unsigned int iterator) {
		return getCamera(iterator);
	}

	ChOgreCamera* ChOgreCameraManager::operator[] (std::string Name) {
		return getCamera(Name);
	}

}