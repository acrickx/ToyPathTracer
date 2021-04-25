#pragma once
#include<vector>
#include<fstream>
#include<string>
#include <omp.h>
#include"boundingVolume.h"
#include"material.h"

class Mesh
{
	protected:
		std::vector<Vec3f> m_vertices;
		std::vector<Vec3i> m_indices;
		std::vector<Vec3f> m_normals;
		Material m_mat;		
		AABB m_boundingBox;
		inline Vec3f computeNormal(Vec3<Vec3f> triangle) { return normalize(cross(triangle[1] - triangle[0], triangle[2] - triangle[0]));}
		template <class T>
		void parseLineToVec3(std::string in, Vec3<T>& const vec);
	public:	
		Mesh() {}
		Mesh(Material _material) : m_mat(_material) {};
		//loading model
		void loadOFF(std::string filepath);
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
		inline Material& material() { return m_mat; }
		//const accessors
		inline const std::vector<Vec3f>& vertices() const { return m_vertices; }
		inline const std::vector<Vec3i>& indices() const { return m_indices; }
		inline const Vec3<Vec3f> triangle(Vec3i triangleIndices) const { return Vec3<Vec3f>(m_vertices[triangleIndices[0]], m_vertices[triangleIndices[1]], m_vertices[triangleIndices[2]]); }
		inline const std::vector<Vec3f>& normals() const { return m_normals; }
		inline const AABB& boundingBox() const { return m_boundingBox; }
		inline const Material& material() const { return m_mat; }	
		//Cornell Box initializer		
};


//Primitives
static Mesh Plane(Vec3f center, Vec3f n, Vec3f u, float sideLength, Material mat)
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

static std::vector<Mesh> CornellBox(Vec3f origin, float scale)
{
	{
		Mesh Left = Plane(origin - scale * Vec3f(1, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 0, -1), scale, Material(Vec3f(1, 0, 0), 1.f, 1.f, 0));
		Mesh Right = Plane(origin + scale * Vec3f(1, 0, 0), Vec3f(-1, 0, 0), Vec3f(0, 0, 1), scale, Material(Vec3f(0, 1, 0), 1.f, 1.f, 0));
		Mesh Top = Plane(origin + scale * Vec3f(0, 1, 0), Vec3f(0, -1, 0), Vec3f(1, 0, 0), scale, Material(Vec3f(1, 1, 1), 1.f, 1.f, 0));
		Mesh Bottom = Plane(origin - scale * Vec3f(0, 1, 0), Vec3f(0, 1, 0), Vec3f(1, 0, 0), scale, Material(Vec3f(1, 1, 1), 1.f, 1.f, 0));
		Mesh Back = Plane(origin - scale * Vec3f(0, 0, 1), Vec3f(0, 0, 1), Vec3f(1, 0, 0), scale, Material(Vec3f(1, 1, 1), 1.f, 1.f, 0));
		return std::vector<Mesh> {Back, Left, Right, Top, Bottom };
	}
}





