#pragma once
#include<vector>
#include<fstream>
#include<string>
#include <omp.h>
#include "Vec3.h"
#include "mesh.h"

class BVHnode;

class Scene
{
	private:
		Camera m_cam;
		std::vector<Mesh> m_sceneMesh;	
		std::vector<lightPtr> m_lights;		
	public:
		Scene(Camera _cam, std::vector<Mesh> _mesh, lightPtr _light) : m_cam(_cam), m_sceneMesh(_mesh), m_lights(std::vector<lightPtr> { _light }) { };
		Scene(Camera _cam, std::vector<Mesh> _mesh, std::vector<lightPtr> _lights) : m_cam(_cam), m_sceneMesh(_mesh), m_lights(_lights) {};		
		inline Camera camera() const { return m_cam; }		
		inline std::vector<lightPtr> lightSources() const { return m_lights; }
		inline std::vector<Mesh> meshes() const { return m_sceneMesh; }
};

