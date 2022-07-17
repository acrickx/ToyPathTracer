#include "mesh.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

using namespace std;

//load mesh info from OFF file
void Mesh::loadOFF(const string filename)
{
	AABB bbox{};
	std::cout << "Trying to open file..." << std::endl;
	ifstream in(filename, ifstream::in);	
	std::string line;
	getline(in, line); // OFF string (first line)
	if (in.is_open() && line == "OFF")
	{
		//read nb of vertices and triangles
		getline(in, line);
		Vec3i formatVector;
		parseLineToVec3(line,formatVector);		
		//initialize vectors accordingly
		m_vertices.clear(); m_vertices.resize(formatVector[0]);
		m_indices.clear(); m_indices.resize(formatVector[1]);
		//reading vertices data
		for (int i = 0; i < formatVector[0]; i++)
		{
			getline(in, line);
			parseLineToVec3(line, m_vertices[i]);
			bbox.compareAndUpdate(m_vertices[i]);
		}
		//reading triangles data
		for (int i = 0; i < formatVector[1]; i++)
		{
			getline(in, line);
			line = line.substr(2, string::npos) + ' ';
			parseLineToVec3(line, m_indices[i]);			
		}
		//compute Normals
		computeNormals();
		//add boundingbox
		m_boundingBox = bbox;
		std::cout << "Done loading model." << std::endl;
	}
	else std::cout << "-Error opening File" << std::endl;
	in.close();
}

void Mesh::loadOBJ(const string filename)
{
	tinyobj::ObjReader reader{};

	if (!reader.ParseFromFile(filename, tinyobj::ObjReaderConfig()))
	{
		if (!reader.Error().empty())
		{
			std::cerr << "TinyObjReader: " << reader.Error();
		}
		return;
	}

	if (!reader.Warning().empty()) 
	{
		std::cout << "TinyObjReader: " << reader.Warning();
		return;
	}

	auto& attrib = reader.GetAttrib();
	auto& shapes = reader.GetShapes();

	std::cout << "shape size : " << shapes.size() << std::endl;

	// Just consider it's one mesh for now
	size_t nbFace = shapes[0].mesh.num_face_vertices.size();

	
	AABB bbox{};

	// indices
    int face_offset = 0;
	for (size_t f = 0; f < nbFace; f++)
	{
		size_t fv = size_t(shapes[0].mesh.num_face_vertices[f]);
		vector<size_t> indices(fv);

		// Loop over faces
		for (size_t v = 0; v < fv; v++)
		{
			tinyobj::index_t idx = shapes[0].mesh.indices[face_offset + v];
			indices[v] = size_t(idx.vertex_index);
		}

		// update offset
		face_offset += fv;

		// add indices
		m_indices.push_back(Vec3i(indices[0], indices[1], indices[2]));
	}

	//vertex data
	for (size_t v = 0; v < attrib.vertices.size()/3; v++)
	{
		// Positions
		tinyobj::real_t vx = attrib.vertices[v * 3];
		tinyobj::real_t vy = attrib.vertices[v * 3 + 1];
		tinyobj::real_t vz = attrib.vertices[v * 3 + 2];

		// Add vertex and update bbox
		Vec3f vertexPos{ float(vx), float(vy), float(vz) };
		m_vertices.push_back(vertexPos);
		bbox.compareAndUpdate(vertexPos);
	}

	// Normal Data
	for (size_t v = 0; v < attrib.normals.size() / 3; v++)
	{
		tinyobj::real_t nx = attrib.normals[v * 3];
		tinyobj::real_t ny = attrib.normals[v * 3 + 1];
		tinyobj::real_t nz = attrib.normals[v * 3 + 2];
		m_normals.push_back(normalize(Vec3f(float(nx), float(ny), float(nz))));
	}
	if (m_normals.size() == 0)
	{
		std::cout << "TinyObjLoader: Normals not found, computing normals" << std::endl;
		computeNormals();
	}

	// Assign bounding box
	m_boundingBox = bbox;
}

//util to convert OFF document line in Vec3f
template <class T>
void Mesh::parseLineToVec3(std::string in, Vec3<T>& const vec)
{
	Vec3<T> out;
	for (int i = 0; i < 3; i++)
	{
		size_t space_index = in.find(' ');
		if (space_index != string::npos)
		{
			T value = std::stod(in.substr(0, space_index));
			out[i] = value;
			in = in.substr(space_index + 1, string::npos);
		}
	}
	vec = out;
}

//compute normals for all mesh (uniform normals)
void Mesh::computeNormals()
{
	m_normals.clear();
	m_normals.resize(m_vertices.size());
	vector<size_t> neighboursCount(m_vertices.size());
	for (size_t i = 0; i < m_indices.size(); i++)
	{
		Vec3f normal = computeNormal(triangle(m_indices[i]));
		for (int j = 0; j < 3; j++)
		{
			m_normals[m_indices[i][j]] += normal;
			neighboursCount[m_indices[i][j]] += 1;
		}
	}
	//take normal average
	for (int i = 0; i < m_normals.size(); i++)
	{
		m_normals[i] = normalize(m_normals[i]);
	}
}