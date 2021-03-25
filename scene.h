#pragma once
#include<vector>
#include<fstream>
#include<string>
#include <omp.h>
#include "Vec3.h"
#include "mesh.h"
#include "BVHnode.h"

class BSHnode;

class Scene
{
	private:
		Camera m_cam;
		std::vector<Mesh> m_meshes;	
		std::vector<lightPtr> m_lights;		
		BVHroot m_root;
	public:
		Scene(Camera _cam, std::vector<Mesh> _mesh, lightPtr _light) : m_cam(_cam), m_meshes(_mesh), m_lights(std::vector<lightPtr> { _light }) {};
		Scene(Camera _cam, std::vector<Mesh> _mesh, std::vector<lightPtr> _lights) : m_cam(_cam), m_meshes(_mesh), m_lights(_lights) {};		
		inline void computeBVH() { m_root = BVHroot(m_meshes); }
		inline const BVHroot& getBVHroot() const { return m_root; }
		inline const Camera& camera() const { return m_cam; }		
		inline const std::vector<lightPtr>& lightSources() const { return m_lights; }
		inline const std::vector<Mesh>& meshes() const { return m_meshes; }
};

