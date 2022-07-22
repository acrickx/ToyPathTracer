#pragma once
#include <vector>
#include <fstream>
#include <string>
#include <omp.h>
#include "boundingVolume.h"
#include "material.h"
#include "GeometryHelper.h"

typedef std::shared_ptr<Material> MaterialPtr;

class Mesh
{
	protected:
		std::vector<Vec3f> m_vertices;
		std::vector<Vec3i> m_indices;
		std::vector<Vec3f> m_normals;
		MaterialPtr m_mat;		
		AABB m_boundingBox;
		template <class T>
		void parseLineToVec3(std::string in, Vec3<T>& const vec);
	public:	
		Mesh() : m_mat(MaterialPtr(new MaterialGGX(Vec3f(1, 0, 1), 1.0f, 0.0f, 0.0f))) {};
		Mesh(MaterialPtr _material) : m_mat(_material) {};
		//loading model
		void loadOFF(const std::string filepath);
		void loadOBJ(const std::string filepath);
		void scale(float scale) { for (int i = 0; i < m_vertices.size(); i++) { m_vertices[i] *= scale; } m_boundingBox.min() *= scale; m_boundingBox.max() *= scale; }
		void translate(Vec3f translate) { for (int i = 0; i < m_vertices.size(); i++) { m_vertices[i] += translate; } m_boundingBox.min() += translate; m_boundingBox.max() += translate; }
		//normal computations
		void computeNormals();
		inline const Vec3f interpPos(Vec3f barCoord, Vec3i triangleIndices) const { return (barCoord[2] * m_vertices[triangleIndices[0]] + barCoord[0] * m_vertices[triangleIndices[1]] + barCoord[1] * m_vertices[triangleIndices[2]]); } 
		inline const Vec3f interpNorm(Vec3f barCoord, Vec3i triangleIndices) const { return normalize(barCoord[2] * m_normals[triangleIndices[0]] + barCoord[0] * m_normals[triangleIndices[1]] + barCoord[1] * m_normals[triangleIndices[2]]); } 
		//accessors
		inline std::vector<Vec3f>& vertices() { return m_vertices; }
		inline std::vector<Vec3i>& indices() { return m_indices; }		
		inline std::vector<Vec3f>& normals() { return m_normals; }
		inline AABB& boundingBox() { return m_boundingBox; }
		inline MaterialPtr material() { return m_mat; }
		//const accessors
		inline const std::vector<Vec3f>& vertices() const { return m_vertices; }
		inline const std::vector<Vec3i>& indices() const { return m_indices; }
		inline const Vec3<Vec3f> triangle(Vec3i triangleIndices) const { return Vec3<Vec3f>(m_vertices[triangleIndices[0]], m_vertices[triangleIndices[1]], m_vertices[triangleIndices[2]]); }
		inline const std::vector<Vec3f>& normals() const { return m_normals; }
		inline const AABB& boundingBox() const { return m_boundingBox; }
		inline const MaterialPtr material() const { return m_mat; }	
		//Cornell Box initializer		
};


//Primitives
static Mesh Plane(Vec3f center, Vec3f n, Vec3f u, float sideLength, MaterialPtr mat)
{
	Mesh plane(mat);
	n.normalize();
	u.normalize();
	//add an offset so that the aabb isn't flat
	Vec3f v = normalize(cross(n, u));
	Vec3f A = center + 0.0001f*n - (u + v) * sideLength / 2.f;
	Vec3f B = center + (v - u) * sideLength / 2.f;
	Vec3f C = center - 0.0001f*n + (u + v) * sideLength / 2.f;
	Vec3f D = center + (u - v) * sideLength / 2.f;
	plane.vertices() = std::vector<Vec3f>{ A,B,C,D };
	plane.indices() = std::vector<Vec3i>{ Vec3i(0,1,2), Vec3i(0,2,3) };
	plane.normals() = std::vector<Vec3f>{ n, n, n, n };	
	plane.boundingBox() = AABB(plane.vertices());	
	return plane;
}





