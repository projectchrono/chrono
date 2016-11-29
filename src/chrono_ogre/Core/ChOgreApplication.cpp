/*
Author: Charles Ricchio

The actual definitions for ChOgreApplication. Most of the code in the constructor is from a tutorial, and could probably
be
written more elegantly.
*/

#include "ChOgreApplication.h"
#include "physics/ChGlobal.h"

#define SDL_MAIN_HANDLED

#include <SDL_syswm.h>

using namespace Ogre;

namespace chrono{
namespace ChOgre {


	ChOgreApplication::ChOgreApplication() {
		m_pRoot = new Ogre::Root("", "", "ChOgre.log");

		//NOTE: Probably terrible practice. Do better
		{
			std::vector<std::string> l_Plugins;

			l_Plugins.push_back("RenderSystem_GL");
			l_Plugins.push_back("Plugin_ParticleFX");
			l_Plugins.push_back("Plugin_CgProgramManager");
			l_Plugins.push_back("Plugin_OctreeSceneManager");
			//l_tPlugins.push_back("Plugin_PCZSceneManager");
			//l_tPlugins.push_back("Plugin_OctreeZone");
			//l_tPlugins.push_back("Plugin_BSPSceneManager");

			//NOTE: Again, this is straight from a tutorial, so this could probably be done better
			{
				// use configuration file in the future for better compatibility
#if defined(__linux__)
				std::string plugin_root = "/usr/local/lib/OGRE/";
#else
				std::string plugin_root = "";
#endif
				for (auto p : l_Plugins) {
					std::string& l_PluginName = p;

					bool l_IsInDebugMode = OGRE_DEBUG_MODE;

					if (l_IsInDebugMode) {
						l_PluginName += "_d";
					}
					m_pRoot->loadPlugin(plugin_root + l_PluginName);
				}
			}
			//const std::string pluginfile = "plugins.cfg";

			//m_pRoot->loadPlugin("RenderSystem_GL");
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

			m_pChSystem = new chrono::ChSystem;
			m_pScene = new ChOgreScene(m_pSceneManager, m_pChSystem);
		}

        {
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    chrono::GetChronoDataPath(), "FileSystem",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    chrono::GetChronoDataPath() + "/ogre", "FileSystem",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    chrono::GetChronoDataPath() + "/ogre/mygui_resources",
                    "FileSystem",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    chrono::GetChronoDataPath() + "/ogre/materials", "FileSystem",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    chrono::GetChronoDataPath() + "/ogre/materials/textures",
                    "FileSystem",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    chrono::GetChronoDataPath() + "/ogre/models", "FileSystem",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                    chrono::GetChronoDataPath() + "/ogre/skyboxes/sky",
                    "FileSystem",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);

			Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

		}

		timestep_max = 0.05;
		timestep_min = 0.001;
		timestep = 0;
		isRealTime = false;
		isRunning = false;
		WriteToFile = false;
	}

	ChOgreApplication::~ChOgreApplication() {
		delete m_pCameraManager;
		delete m_pScene;
		delete m_pChSystem;
		closeWindow();
	}

	int ChOgreApplication::startLoop(std::function<int()> _func) {
		int l_run = 0;

		isRunning = true;

		double l_systemTimeIncriment = 0.0;

		int l_frame = 0;

		std::chrono::high_resolution_clock l_time;
		auto l_start = l_time.now();
		auto l_last = l_start;

		while (l_run == 0) {

			try {
				m_pChSystem->DoFrameDynamics(l_systemTimeIncriment);
			}
			catch (...) {

			}

			pollInput();

			l_run = _func();

			if (m_pInputManager->isWindowToClose() == true) {
				std::cout << "Closing\n";
				l_run++;
				break;
			}

			//m_pGUIManager->update();

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
				std::string name;
				std::string frame_count = std::to_string(l_frame);
				std::string pad;

				for (unsigned int i = 0; i < (6 - frame_count.size()); i++) {
					pad += '0';
				}

				frame_count = pad + frame_count;

				if (!OutputImageFolder.empty()) {
					name = OutputImageFolder + "/frame" + frame_count + ".png";
				}
				else {
					name = "frame" + frame_count + ".png";
				}

				m_pRenderWindow->writeContentsToFile(name);

				l_frame++;
			}

			drawFrame();

		}
		isRunning = false;

		return l_run;
	}


	void ChOgreApplication::chronoThread() {

		while (isRunning) {

		}
	}

	Ogre::RenderWindow* ChOgreApplication::createWindow(const std::string& Title, uint32_t Width, uint32_t Height, uint8_t FSAA_Level, bool VSync, bool Fullscreen) {

        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            Ogre::LogManager::getSingleton().logMessage(
                    Ogre::LogMessageLevel::LML_CRITICAL,
                    "\n\nCould not initialize SDL video: " + std::string(SDL_GetError())
                            + "\n\n");
        }
        mSdlWindow = SDL_CreateWindow(Title.c_str(), 50, 50, Width, Height, SDL_WINDOW_SHOWN |
                (Fullscreen ? SDL_WINDOW_FULLSCREEN : 0) | SDL_WINDOW_RESIZABLE);
        SDL_SysWMinfo wmInfo;
        SDL_VERSION(&wmInfo.version);
        SDL_GetWindowWMInfo(mSdlWindow, &wmInfo);

#ifdef WIN32
		Ogre::String winHandle = Ogre::StringConverter::toString((uintptr_t)wmInfo.info.win.window);
#else
        Ogre::String winHandle = Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.x11.window );
#endif

		Ogre::NameValuePairList l_Params;

		l_Params["FSAA"] = std::to_string(FSAA_Level);

		if (VSync) {
			l_Params["vsync"] = "true";
		}
		else {
			l_Params["vsync"] = "false";
		}
        l_Params["parentWindowHandle"] = winHandle;

		m_pRenderWindow = m_pRoot->createRenderWindow(Title, Width, Height, Fullscreen, &l_Params);

		m_pCamera = m_pSceneManager->createCamera("MainCamera");

		m_pViewport = m_pRenderWindow->addViewport(m_pCamera);
		m_pViewport->setAutoUpdated(false);
		m_pViewport->setBackgroundColour(Ogre::ColourValue(0.0f, 0.0f, 0.0f));

		m_pCamera->setAspectRatio( ( ( (float)(m_pViewport->getActualWidth()) ) / ( (float)(m_pViewport->getActualHeight()) ) ) );
		m_pCamera->setNearClipDistance(5.0f);

		m_pRenderWindow->setActive(true);
		m_pRenderWindow->setAutoUpdated(false);

		m_pCameraManager = new ChOgreCameraManager(m_pSceneManager, m_pViewport);

		m_pRoot->clearEventTimes();

		m_pInputManager = new ChOgre_SDLInputHandler(mSdlWindow);
		m_pGUIManager = new ChOgreGUIManager(m_pRenderWindow, m_pSceneManager, m_pInputManager);

		m_pRenderWindow->setActive(true);

		return m_pRenderWindow;
	}

	void ChOgreApplication::loadResourcePath(const std::string& Path, const std::string& Title) {
		Ogre::ResourceGroupManager::getSingleton().addResourceLocation(Path, Title);
		Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(Title);
	}

	void ChOgreApplication::setCamera(ChOgreCamera* Camera) {
		m_pCameraManager->makeActive(Camera);
	}

	void ChOgreApplication::setVSync(bool VSync) {
		isVSyncEnabled = VSync;
		m_pRenderWindow->setVSyncEnabled(isVSyncEnabled);
		m_pRenderWindow->setVSyncInterval(60);
	}

	void ChOgreApplication::drawFrame() {
		m_pScene->update();

		m_pViewport->update();

		m_pRenderWindow->update(true);
		//m_pRenderWindow->swapBuffers();

		m_pRoot->renderOneFrame();

		m_pCamera->setAspectRatio((((float)(m_pViewport->getActualWidth())) / ((float)(m_pViewport->getActualHeight()))));

		Ogre::WindowEventUtilities::messagePump();
	}

	void ChOgreApplication::pollInput() {
		m_pInputManager->update();
	}

	void ChOgreApplication::closeWindow() {
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
			delete m_pGUIManager;
		}
	}

	ChOgreCameraManager* ChOgreApplication::getCameraManager() {
		return m_pCameraManager;
	}

	ChOgreScene* ChOgreApplication::getScene() {
		return m_pScene;
	}

	ChOgre_SDLInputHandler* ChOgreApplication::getInputManager() {
		return m_pInputManager;
	}

	ChOgreGUIManager* ChOgreApplication::getGUIManager() {
		return m_pGUIManager;
	}

	Ogre::RenderWindow* ChOgreApplication::getWindow() {
		return m_pRenderWindow;
	}

	Ogre::SceneManager* ChOgreApplication::getSceneManager() {
		return m_pSceneManager;
	}

	chrono::ChSystem* ChOgreApplication::getChSystem() {
		return m_pChSystem;
	}

	void ChOgreApplication::logMessage(const std::string& Message, Ogre::LogMessageLevel lml, bool maskDebug) {
		Ogre::LogManager::getSingleton().logMessage(Message, lml, maskDebug);
	}

}
}
