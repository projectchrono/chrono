#ifndef CHOPENGLOBJECT_H
#define CHOPENGLOBJECT_H

#include "ChOpenGLBase.h"
#include "ChOpenGLShader.h"
#include "ChOpenGLVertexAttributes.h"
#include "ChOpenGLMaterial.h"

namespace chrono {
namespace utils {
class CH_UTILS_OPENGL_API ChOpenGLObject : public ChOpenGLBase {
 public:
   ChOpenGLObject();
   virtual ~ChOpenGLObject();
   virtual void TakeDown();
   virtual bool Initialize();
   virtual bool PostGLInitialize(
         GLuint * position_ptr,
         GLuint * normal_ptr,
         GLuint * ambient_ptr,
         GLuint * diffuse_ptr,
         GLuint * specular_ptr,
         GLsizeiptr size);
   virtual void Draw(
         const glm::mat4 & projection,
         const glm::mat4 & modelview) = 0;
   void AttachShader(
         ChOpenGLShader * new_shader);

 protected:
   GLuint vertex_array_handle;
   GLuint vertex_coordinate_handle;
   GLuint vertex_position_handle;
   GLuint vertex_normal_handle;
   GLuint vertex_ambient_handle;
   GLuint vertex_diffuse_handle;
   GLuint vertex_specular_handle;
   GLuint vertex_element_handle;
   std::vector<GLuint> vertex_indices;
   ChOpenGLShader * shader;
 private:
   void InternalInitialize();
};

}
}

#endif  // END of CHOPENGLOBJECT_H
