/*Based on code provided by: Perry Kivolowitz
Background object class
Draws a quad without any attached shader
Shader is specified manually (used for deferred shading)
*/


#pragma once
#include "utils/opengl/core/ChOpenGLObject.h"

namespace chrono {
	namespace utils{
		class ChOpenGLCloud : public ChOpenGLObject
		{
		public:
			ChOpenGLCloud();
			virtual bool Initialize(const std::vector<glm::vec3>& data);
	//Not used but needed because of inheritance
			virtual void Draw(const glm::mat4 & projection, const glm::mat4 & modelview);
			void TakeDown();

		private:
			std::vector<ChOpenGLVertexAttributesP> vertices;
			typedef ChOpenGLObject super;
		};
	}
}
