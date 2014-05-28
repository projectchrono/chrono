#ifndef CHOPENGLOBJ_H
#define CHOPENGLOBJ_H



#include "chrono_utils/opengl/core/ChOpenGLObject.h"
#include "chrono_utils/opengl/shapes/ChOpenGLMesh.h"
#include "chrono_utils/opengl/shapes/obj/objloader.h"
namespace chrono {
namespace utils {
class CH_UTILS_OPENGL_API ChOpenGLOBJ : public ChOpenGLBase {
 public:
   ChOpenGLOBJ();
   bool Initialize(
         std::string filename,
         ChOpenGLMaterial mat,
         ChOpenGLShader * shader);
   bool PostInitialize();
   virtual void Draw(
         const glm::mat4 & projection,
         const glm::mat4 & modelview);
   void TakeDown();
 protected:

   std::vector<std::vector<glm::vec3> > vertices;
   std::vector<std::vector<glm::vec3> > normals;
   std::vector<std::vector<glm::vec2> > texcoords;
   std::vector<std::vector<GLuint> > indices;
   std::vector<std::string> names;
   std::vector<ChOpenGLMesh> meshes;
   OBJLoader loader;
};
}
}

#endif  // END of CHOPENGLOBJ_H
