/*  Perry Kivolowitz - University of Wisconsin - Madison
Computer Sciences Department

A sample hello world like program demonstrating modern
OpenGL techniques.

Created:    2/25/13
Updates:
*/

#include <iostream>
#include "objloader.h"
#include <sstream>
#include <string>
using namespace std;
using namespace glm;
using namespace chrono::utils;
OBJLoader::OBJLoader() 
{
}


//load an obj mesh. Each mesh can have multiple sub meshes
void OBJLoader::LoadObject(string fname, 
						   vector<vector<glm::vec3> > &vertices,
						   vector<vector<glm::vec3> > &normals,
						   vector<vector<glm::vec2> > &texcoords,
						   vector<vector<GLuint> > &indices,
						   vector<string> & names
						   )
{
	std::vector<tinyobj::shape_t> shapes;

	std::string err = tinyobj::LoadObj(shapes, fname.c_str());
	 
	std::cout << "# of shapes : " << shapes.size() << std::endl;

	vertices.resize(shapes.size());
	normals.resize(shapes.size());
	texcoords.resize(shapes.size());
	indices.resize(shapes.size());
	names.resize(shapes.size());

	//convert between mesh loader data structure and vector data structure
	for (size_t i = 0; i < shapes.size(); i++) {

		vertices[i].resize(shapes[i].mesh.positions.size()/3);
		normals[i].resize(shapes[i].mesh.normals.size()/3);
		texcoords[i].resize(shapes[i].mesh.texcoords.size()/2);
		indices[i].resize(shapes[i].mesh.indices.size());
		names[i] = shapes[i].name;

		cout<<shapes[i].mesh.positions.size()/3<<" "<<shapes[i].mesh.normals.size()/3<<" "<<shapes[i].mesh.texcoords.size()/2<<" "<<shapes[i].mesh.indices.size()<<endl;

		for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
			vertices[i][v] =  vec3(shapes[i].mesh.positions[3*v+0],shapes[i].mesh.positions[3*v+1],shapes[i].mesh.positions[3*v+2]);
		}
		for (size_t n = 0; n < shapes[i].mesh.normals.size() / 3; n++) {
			normals[i][n] =  vec3(shapes[i].mesh.normals[3*n+0],shapes[i].mesh.normals[3*n+1],shapes[i].mesh.normals[3*n+2]);
		}
		for (size_t t = 0; t < shapes[i].mesh.texcoords.size() / 2; t++) {
			texcoords[i][t] =  vec2(shapes[i].mesh.texcoords[2*t+0],shapes[i].mesh.texcoords[2*t+1]*-1);
		}
		for (size_t f = 0; f < shapes[i].mesh.indices.size(); f++) {
			indices[i][f]=shapes[i].mesh.indices[f];
		}

	}
}
