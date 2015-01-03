/*
Author: Charles Ricchio
*/

#pragma once

#include "ECGUIElement.h"
#include <core/ChVector.h>
#include <OGRE\Ogre.h>
#include <OGRE\Overlay\OgreFontManager.h>
#include <OGRE\Overlay\OgreFont.h>
#include <OGRE\Overlay\OgreOverlayContainer.h>
#include <OGRE\Overlay\OgreTextAreaOverlayElement.h>

namespace EnvironmentCore {

	class ECGUIText : public ECGUIElement {

	public:

		ECGUIText(Ogre::Overlay* Overlay);
		~ECGUIText();

		virtual void setName(std::string Name);
		virtual void setPosition(double x, double y);
		virtual void setSize(double x, double y);
		virtual void setColor(double r, double g, double b);

		virtual void setText(std::string Text);
		virtual void setFont(double CharacterHeight);

		chrono::ChVector<> getPosition();
		chrono::ChVector<> getSize();

	protected:

		Ogre::TextAreaOverlayElement* m_pText;
		Ogre::OverlayContainer* m_pPanel;
		Ogre::FontPtr m_Font;

		static unsigned int g_count;

	private:



	};

}