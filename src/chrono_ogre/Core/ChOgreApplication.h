/*
Author: Charles Ricchio

Contains the prototype for a basic application. Manages things such as window creation, the camera and input.
*/

#pragma once

#include <Ogre.h>
#include "OgreHardwarePixelBuffer.h"

#include <physics/ChSystem.h>

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include <memory>
#include <exception>
#include <thread>
#include <chrono>
#include <vector>
#include <random>

#include "chrono_ogre/ChOgreApi.h"

#include "chrono_ogre/Input/ChOgre_SDLInputHandler.h"
#include "chrono_ogre/Graphics/ChOgreCameraManager.h"
#include "chrono_ogre/Graphics/ChOgreScene.h"
#include "chrono_ogre/GUI/ChOgreGUIManager.h"

#define ChOgreFunc(N) [&](N)

namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreApplication {
  public:
    ChOgreApplication();
    ~ChOgreApplication();

    typedef std::function<int()> ChOgreLoopCallFunc;

    virtual int startLoop(ChOgreLoopCallFunc PerFrame);
    virtual Ogre::RenderWindow* createWindow(const std::string& Title,
                                             uint32_t Width,
                                             uint32_t Height,
                                             uint8_t FSAA_Level,
                                             bool VSync = false,
                                             bool Fullscreen = false);
	virtual void loadResourcePath(const std::string&, const std::string& Title = "FileSystem");
    virtual void setCamera(ChOgreCamera* Camera);
    virtual void setVSync(bool VSync);

	void drawFrame();
	void pollInput();

    virtual void chronoThread();

    virtual void closeWindow();

    ChOgreCameraManager* getCameraManager();
    ChOgreScene* getScene();
    ChOgre_SDLInputHandler* getInputManager();
    ChOgreGUIManager* getGUIManager();
    Ogre::RenderWindow* getWindow();
    Ogre::SceneManager* getSceneManager();
    chrono::ChSystem* getChSystem();

    static void logMessage(const std::string& Message,
                           Ogre::LogMessageLevel lml = Ogre::LML_NORMAL,
                           bool maskDebug = false);

    double timestep_max;
    double timestep_min;
    double timestep;

    bool isRealTime;

    bool isRunning;

    bool WriteToFile;
    std::string OutputImageFolder;

  protected:
    SDL_Window *mSdlWindow;

    Ogre::Root* m_pRoot;
    Ogre::RenderWindow* m_pRenderWindow;
    Ogre::SceneManager* m_pSceneManager;
    Ogre::Viewport* m_pViewport;
    Ogre::Camera* m_pCamera;

    chrono::ChSystem* m_pChSystem;

    std::thread m_ChronoThread;

    ChOgreCameraManager* m_pCameraManager;
    ChOgreScene* m_pScene;
    ChOgre_SDLInputHandler* m_pInputManager;
    ChOgreGUIManager* m_pGUIManager;

    bool isVSyncEnabled;

  private:
};
}
}
