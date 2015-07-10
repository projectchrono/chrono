#pragma once

#include "../ChOgre.h"
#include <MYGUI/MyGUI.h>
#include <MYGUI/MyGUI_OgrePlatform.h>
#include <core/ChVector.h>
#include <memory>
#include <string>

namespace ChOgre {

	typedef chrono::ChVector<int> ChInt3;

	class CHOGRE_DLL_TAG ChOgreGUIElement {

	public:

		ChOgreGUIElement();
		ChOgreGUIElement(MyGUI::Gui* GUI);
		~ChOgreGUIElement();

		virtual void setName(std::string Name);
		virtual void setGUI(MyGUI::Gui* GUI);
		virtual void setPosition(const ChInt3& Position) =0;
		virtual void setSize(const ChInt3& Size) =0;
		virtual void setColor(float r, float g, float b) =0;
		virtual void update() {};

		virtual std::string getName();

	protected:

		std::string m_Name;
		MyGUI::Gui* m_pGUI;
		
	private:



	};

	typedef std::unique_ptr<ChOgreGUIElement> ChOgreGUIElementPtr;

}