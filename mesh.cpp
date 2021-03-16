#include "mesh.h"

using namespace std;

//load mesh info from OFF file
void Mesh::loadOFF(const string filename)
{
	AABB bbox{};
	std::cout << "-Trying to open file..." << std::endl;
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
		std::cout << "-Done loading model" << std::endl;
	}
	else std::cout << "-Error opening File" << std::endl;
	in.close();
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
		m_normals[i] = normalize(m_normals[i]/neighboursCount[i]);
	}
}