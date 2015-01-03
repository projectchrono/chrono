#pragma once

#include <OGRE\Overlay\OgreOverlay.h>
#include <OGRE\Overlay\OgreOverlayManager.h>

namespace EnvironmentCore {

	class ECGUIElement {

	public:

		ECGUIElement(Ogre::Overlay* Overlay);
		~ECGUIElement();

		virtual void setName(std::string Name);
		virtual void setPosition(double x, double y) =0;
		virtual void setSize(double x, double y) =0;
		virtual void setColor(double r, double g, double b) =0;
		virtual void update() {};

		virtual std::string getName();

	protected:

		std::string m_Name;
		Ogre::Overlay* m_pOverlay;

	private:



	};

}