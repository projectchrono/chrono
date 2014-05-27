#include <iostream>
#include "ChOpenGLCloud.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;

ChOpenGLCloud::ChOpenGLCloud() : ChOpenGLObject() {
}

bool ChOpenGLCloud::Initialize(const std::vector<glm::vec3>& data) {
	if (this->GLReturnedError("Background::Initialize - on entry"))
		return false;

	if (!super::Initialize()) {
		return false;
	}

	for(int i=0; i<data.size(); i++){
		this->vertices.push_back(ChOpenGLVertexAttributesP(data[i]));
	}

	//set up arrays on GPU
	if (!this->PostGLInitialize(&this->vertex_array_handle, &this->vertex_coordinate_handle, this->vertices.size() * sizeof(ChOpenGLVertexAttributesP), &this->vertices[0]))
		return false;

	//specify the locations of each input to the vertex shader
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesP), (GLvoid *) (sizeof(vec3) * 0));     //Position
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArrayAPPLE(0);

	if (this->GLReturnedError("Background::Initialize - on exit"))
		return false;

	return true;
}

void ChOpenGLCloud::TakeDown() {
	super::TakeDown();
}

void ChOpenGLCloud::Draw(const mat4 & projection, const mat4 & modelview) {
	if (this->GLReturnedError("Mesh::Draw - on entry"))
		return;
	glEnable(GL_DEPTH_TEST);
	//compute the mvp matrix and normal matricies
	mat4 mvp = projection * modelview;
	mat3 nm = inverse(transpose(mat3(modelview)));

	//Enable the shader
	shader->Use();
	this->GLReturnedError("Mesh::Draw - after use");
	//Send our common uniforms to the shader
	shader->CommonSetup(value_ptr(projection), value_ptr(modelview), value_ptr(mvp), value_ptr(nm));

	this->GLReturnedError("Mesh::Draw - after common setup");
	//Bind and draw! (in this case we draw as triangles)
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
	glDrawArrays(GL_POINTS, 0, this->vertices.size());
	//glBindVertexArrayAPPLE(0);
	glDisableClientState(GL_VERTEX_ARRAY);
	this->GLReturnedError("Mesh::Draw - after draw");
	glUseProgram(0);

	if (this->GLReturnedError("Mesh::Draw - on exit"))
		return;
}
