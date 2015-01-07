/*
Author: Charles Ricchio

The actual definitions for EnvironmentCoreApplication. Most of the code in the constructor is from a tutorial, and could probably be
written more elegantly.
*/

#include "ChOgreApplication.h"

namespace EnvironmentCore {

	EnvironmentCoreApplication::EnvironmentCoreApplication() {
		m_pRoot = new Ogre::Root("", "", "EnvironmentCore.log");

		//NOTE: Probably terrible practice. Do better
		{
			typedef std::vector<std::string> __Strings_t__;

			__Strings_t__ l_Plugins;

			l_Plugins.push_back("RenderSystem_GL");
			l_Plugins.push_back("Plugin_ParticleFX");
			l_Plugins.push_back("Plugin_CgProgramManager");
			l_Plugins.push_back("Plugin_OctreeSceneManager");
			//l_tPlugins.push_back("Plugin_PCZSceneManager");
			//l_tPlugins.push_back("Plugin_OctreeZone");
			//l_tPlugins.push_back("Plugin_BSPSceneManager");

			//NOTE: Again, this is straight from a tutorial, so this could probably be done better
			{
				__Strings_t__::iterator l_Iterator = l_Plugins.begin();
				__Strings_t__::iterator l_IteratorEnd = l_Plugins.end();

				for (l_Iterator; l_Iterator != l_IteratorEnd; l_Iterator++) {
					std::string& l_PluginName = (*l_Iterator);

					bool l_IsInDebugMode = OGRE_DEBUG_MODE;

					if (l_IsInDebugMode) {
						l_PluginName.append("_d");
					}
					m_pRoot->loadPlugin(l_PluginName);
				}
			}
		}

		//NOTE: Just rewrite this entire function

		{
			const Ogre::RenderSystemList& l_RenderSystemList = m_pRoot->getAvailableRenderers();
			if (l_RenderSystemList.size() == 0) {
				logMessage("Sorry, no rendersystem was found.");
			}

			Ogre::RenderSystem* l_RenderSystem = l_RenderSystemList[0];
			m_pRoot->setRenderSystem(l_RenderSystem);
		}

		{
			m_pRoot->initialise(false, "", "");

			m_pSceneManager = m_pRoot->createSceneManager(Ogre::ST_GENERIC, "MainSceneManager");

			m_pCameraManager = new ECCameraManager;
			m_pChSystem = new chrono::ChSystem;
			m_pScene = new ECScene(m_pSceneManager, m_pChSystem);
		}

		{
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/materials", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/materials/programs", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/materials/scripts", "FileSystem");
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/materials/textures", "FileSystem");
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/fonts/minecraftia", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/materials/textures/nvidia", "FileSystem");
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/models", "FileSystem");
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/skyboxes/sky", "FileSystem");
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/heightmaps", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/particle", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/DeferredShadingMedia", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/RTShaderLib", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/RTShaderLib/materials", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/materials/scripts/SSAO", "FileSystem");
			//Ogre::ResourceGroupManager::getSingleton().addResourceLocation("assets/materials/textures/SSAO", "FileSystem");

			Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

		}

		timestep_max = 0.05;
		timestep_min = 0.001;
		timestep = 0;
		isRealTime = true;
		isRunning = false;
		WriteToFile = false;
	}

	EnvironmentCoreApplication::~EnvironmentCoreApplication() {
		delete m_pCameraManager;
		delete m_pScene;
		delete m_pChSystem;
		closeWindow();
	}

	int EnvironmentCoreApplication::startLoop(std::function<int()> _func) {
		int l_run = 0;
		
		isRunning = true;

		double l_systemTimeIncriment = 0.0;

		int l_frame = 0;
		Ogre::TexturePtr rtt_texture = Ogre::TextureManager::getSingleton().createManual("RttTex", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, m_pRenderWindow->getWidth(), m_pRenderWindow->getHeight(), 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);
		Ogre::RenderTexture* renderTexture = rtt_texture->getBuffer()->getRenderTarget();

		renderTexture->addViewport(m_pCamera);
		renderTexture->getViewport(0)->setClearEveryFrame(true);
		renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
		renderTexture->getViewport(0)->setOverlaysEnabled(false);

		std::chrono::high_resolution_clock l_time;
		auto l_start = l_time.now();
		auto l_last = l_start;

		while (l_run == 0) {

			try {
				m_pChSystem->DoFrameDynamics((l_systemTimeIncriment / 2.0));
			}
			catch (std::exception e) {

			}
			try {
				m_pChSystem->DoFrameDynamics((l_systemTimeIncriment / 2.0));
			}
			catch (std::exception e) {

			}

			m_pInputManager->update();

			if (m_pInputManager->WindowClose) {
				l_run++;
				break;
			}

			//m_pGUIManager->update();

			l_run = _func();

			m_pScene->update();

			m_pViewport->update();

			m_pRenderWindow->update(false);
			m_pRenderWindow->swapBuffers();

			m_pRoot->renderOneFrame();

			m_pCamera->setAspectRatio((((float)(m_pViewport->getActualWidth())) / ((float)(m_pViewport->getActualHeight()))));


			if (!isRealTime) {
				l_systemTimeIncriment += timestep_max;
				timestep = timestep_max;
			}
			else {
				l_systemTimeIncriment = ((double)(std::chrono::duration_cast<std::chrono::milliseconds>(l_time.now() - l_start).count())) / 1000.0; //converts standard library time difference to a double for Chrono

				l_systemTimeIncriment = l_systemTimeIncriment > timestep_max ? l_systemTimeIncriment+=timestep_max : l_systemTimeIncriment;
				l_systemTimeIncriment = l_systemTimeIncriment < timestep_min ? l_systemTimeIncriment+=timestep_min : l_systemTimeIncriment;

				timestep = ((double)(std::chrono::duration_cast<std::chrono::milliseconds>(l_time.now() - l_last).count())) / 1000.0;
				l_last = l_time.now();
			}

			if (WriteToFile) {
				std::string name = "out/frame" + std::to_string(l_frame) + "time" + std::to_string(l_systemTimeIncriment) + ".png";
				renderTexture->update();
				renderTexture->writeContentsToFile(name);
				l_frame++;
			}

			Ogre::WindowEventUtilities::messagePump();

			
		}
		isRunning = false;

		return l_run;
	}


	void EnvironmentCoreApplication::chronoThread() {
		
		while (isRunning) {
			
		}
	}

	Ogre::RenderWindow* EnvironmentCoreApplication::createWindow(std::string Title, uint32_t Width, uint32_t Height, uint8_t FSAA_Level, bool VSync, bool Fullscreen) {
		Ogre::NameValuePairList l_Params;
		l_Params["FSAA"] = "0";
		if (VSync) {
			l_Params["vsync"] = "true";
		}
		else {
			l_Params["vsync"] = "false";
		}
		m_pRenderWindow = m_pRoot->createRenderWindow(Title, Width, Height, Fullscreen, &l_Params);

		m_pCamera = m_pSceneManager->createCamera("MainCamera");

		m_pViewport = m_pRenderWindow->addViewport(m_pCamera);
		m_pViewport->setAutoUpdated(false);
		m_pViewport->setBackgroundColour(Ogre::ColourValue(0.0f, 0.0f, 0.0f));

		m_pCamera->setAspectRatio( ( ( (float)(m_pViewport->getActualWidth()) ) / ( (float)(m_pViewport->getActualHeight()) ) ) );
		m_pCamera->setNearClipDistance(5.0f);

		m_pRenderWindow->setActive(true);
		m_pRenderWindow->setAutoUpdated(false);


		m_pRoot->clearEventTimes();

		m_pInputManager = new EC_SDL_InputManager(m_pRenderWindow);
		//m_pGUIManager = new ECGUIManager(m_pSceneManager, m_pInputManager);


		return m_pRenderWindow;
	}

	void EnvironmentCoreApplication::loadResourcePath(std::string Path, std::string Title) {
		Ogre::ResourceGroupManager::getSingleton().addResourceLocation(Path, Title);
		Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(Title);
	}

	void EnvironmentCoreApplication::setCamera(ECCamera* Camera) {
		m_pCamera->setPosition(Camera->x, Camera->y, Camera->z);
		if (!Camera->useAngles && !Camera->useQuaternions) {
			m_pCamera->lookAt(Camera->wx, Camera->wy, Camera->wz);
			//logMessage("Camera positioned at: " + std::to_string(Camera->x) + " " + std::to_string(Camera->y) + " " + std::to_string(Camera->z) + "  Looking at: " + std::to_string(Camera->wx) + " " + std::to_string(Camera->wy) + " " + std::to_string(Camera->wz));
		}
		else if (Camera->useAngles && !Camera->useQuaternions) {
			auto _yaw = Ogre::Degree::Degree(Camera->yaw);
			auto _pitch = Ogre::Degree::Degree(Camera->pitch);
			m_pCamera->yaw(Ogre::Radian::Radian(_yaw));
			m_pCamera->pitch(Ogre::Radian::Radian(_pitch));
			//logMessage("Camera positioned at: " + std::to_string(Camera->x) + " " + std::to_string(Camera->y) + " " + std::to_string(Camera->z) + "  Looking at: " + std::to_string(Camera->yaw) + " " + std::to_string(Camera->pitch));
		}
		else if (!Camera->useAngles && Camera->useQuaternions) {
			m_pCamera->rotate(Ogre::Quaternion(Camera->rot.e0, Camera->rot.e1, Camera->rot.e2, Camera->rot.e3));
		}
	}

	void EnvironmentCoreApplication::setVSync(bool VSync) {
		isVSyncEnabled = VSync;
		m_pRenderWindow->setVSyncEnabled(isVSyncEnabled);
		m_pRenderWindow->setVSyncInterval(60);
	}

	void EnvironmentCoreApplication::closeWindow() {
		if (m_pRenderWindow) {
			if (!m_pRenderWindow->isClosed()) {
				m_pSceneManager->destroyAllCameras();
				m_pSceneManager->destroyAllManualObjects();
				m_pSceneManager->destroyAllEntities();

				m_pRenderWindow->removeAllViewports();
				m_pRenderWindow->destroy();

			}
			delete m_pRenderWindow;
			delete m_pInputManager;
		}
	}

	ECCameraManager* EnvironmentCoreApplication::getCameraManager() {
		return m_pCameraManager;
	}

	ECScene* EnvironmentCoreApplication::getScene() {
		return m_pScene;
	}

	EC_SDL_InputManager* EnvironmentCoreApplication::getInputManager() {
		return m_pInputManager;
	}

	//ECGUIManager* EnvironmentCoreApplication::getGUIManager() {
	//	return m_pGUIManager;
	//}

	Ogre::RenderWindow* EnvironmentCoreApplication::getWindow() {
		return m_pRenderWindow;
	}

	Ogre::SceneManager* EnvironmentCoreApplication::getSceneManager() {
		return m_pSceneManager;
	}

	chrono::ChSystem* EnvironmentCoreApplication::getChSystem() {
		return m_pChSystem;
	}

	void EnvironmentCoreApplication::logMessage(const std::string& Message, Ogre::LogMessageLevel lml, bool maskDebug) {
		Ogre::LogManager::getSingleton().logMessage(Message, lml, maskDebug);
	}
}