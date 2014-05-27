#ifndef CHOPENGLMATERIAL_H
#define CHOPENGLMATERIAL_H

/*
Material Class 
Stores ambient, diffuse, specular and glow colors
Useful when sending material info to a renderable object
*/

#include "ChOpenGLShader.h"

namespace chrono {
	namespace utils{

		class ChOpenGLMaterial: public ChOpenGLBase
		{
		public:
	//constructor accepts 4 colors and sets them.
			ChOpenGLMaterial(glm::vec3 a,glm::vec3 d, glm::vec3 s,  glm::vec3 g){
				ambient_color = a;
				diffuse_color = d;
				specular_color = s;
				glow_color = g;
			}
	//Dont need to take anything down so this function is empty
			void TakeDown(){}
			glm::vec3 ambient_color;
			glm::vec3 diffuse_color;
			glm::vec3 specular_color;
			glm::vec3 glow_color;

		};
	}
}
#endif  // END of CHOPENGLMATERIAL_H