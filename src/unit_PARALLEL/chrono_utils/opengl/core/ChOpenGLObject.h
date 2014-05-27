#ifndef CHOPENGLOBJECT_H
#define CHOPENGLOBJECT_H

#include "ChOpenGLBase.h"
#include "ChOpenGLShader.h"
#include "ChOpenGLVertexAttributes.h"
#include "ChOpenGLMaterial.h"

namespace chrono {
	namespace utils{
		class CH_UTILS_OPENGL_API ChOpenGLObject: public ChOpenGLBase
		{
		public:
			ChOpenGLObject();
			virtual ~ChOpenGLObject();
			virtual void TakeDown();
			virtual bool Initialize();
			virtual bool PostGLInitialize(GLuint * vertex_array_handle, GLuint * vertex_coordinate_handle, GLsizeiptr sz, const GLvoid * ptr);
			virtual void Draw(const glm::mat4 & projection,const  glm::mat4 & modelview) = 0;
			void AttachShader(ChOpenGLShader * new_shader);

		protected:
			GLuint vertex_coordinate_handle;
			GLuint vertex_array_handle;
			std::vector<GLuint> vertex_indices;
			ChOpenGLShader * shader;
		private:
			void InternalInitialize();
		};

	}
}


#endif  // END of CHOPENGLOBJECT_H
