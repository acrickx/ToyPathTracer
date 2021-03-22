#pragma once
#include<vector>
#include<fstream>
#include<string>
#include <omp.h>
#include"lightSource.h"
#include"camera.h"
#include"boundingVolume.h"


struct Material
{
public:
	Vec3f albedo;
	float diffuse;
	float roughness;
	float metallic;
	Material() { albedo = Vec3f(0.5f, 0.5f, 0.5f); diffuse = 1.f; roughness = 0.5f; metallic = 0.5f; }
	Material(Vec3f _albedo, float _diffuse, float _roughness, float _specular) : albedo(_albedo), diffuse(_diffuse), roughness(_roughness), metallic(_specular) {};
	Vec3f evaluateColorResponse(const Vec3f& position, const Vec3f& normal, lightPtr light, Camera eye) const {
		float diffuseResponse(diffuse / (float)M_PI);
		Vec3f wi = normalize(light->getPosition() - position);
		//Vec3f w0 = normalize(eye.getPosition() - position);
		//Vec3f wh = normalize(wi + w0);
		//Vec3f specularResponse = reflectance(wi, w0, normal);
		Vec3f colorResponse = light->colorResponse() * (Vec3f(diffuseResponse)) * std::max(dot(normal, wi), 0.f);
		return colorResponse;
	};
	Vec3f evaluateColorResponse(const Vec3f& position, const Vec3f& normal, Vec3f direction) const {
		float diffuseResponse(diffuse / (float)M_PI);
		Vec3f wi = direction;
		//Vec3f w0 = normalize(eye.getPosition() - position);
		//Vec3f wh = normalize(wi + w0);
		//Vec3f specularResponse = reflectance(wi, w0, normal);
		Vec3f colorResponse = (Vec3f(diffuseResponse)) * std::max(dot(normal, wi), 0.f);
		return colorResponse;
	};
private:
	inline Vec3f reflectance(Vec3f wi, Vec3f wo, Vec3f n) const
	{
		float alpha = roughness * roughness;
		Vec3f wh = normalize(wi + wo);
		Vec3f F0(0.04f);
		F0 = mix(F0, albedo, metallic);
		return (G_GGX(wi, wo, alpha, n) * F(std::max(dot(wi, wh), 0.f), F0) * D(alpha, wh, n)) / (4 * dot(n, wi) * dot(n, wo));
	}
	float G(Vec3f w, Vec3f n, float alpha) const
	{
		return 2.0 * (dot(n, w)) / (dot(n, w) + sqrt(pow(alpha, 2) + (1 - pow(alpha, 2)) * pow(dot(n, w), 2)));
	}

	float G_GGX(Vec3f wi, Vec3f wo, float alpha, Vec3f n) const
	{
		return G(wi, n, alpha) * G(wo, n, alpha);
	}

	Vec3f F(float cosTheta, Vec3f f0) const
	{
		return f0 + (Vec3f(1.f, 1.f, 1.f) - f0) * pow(1.0 - cosTheta, 5.0);
	}
	float D(float alpha, Vec3f m, Vec3f n) const
	{
		return (pow(alpha, 2)) / (3.1415926 * pow((1 + pow(dot(n, m), 2) * (pow(alpha, 2) - 1)), 2));
	}
};

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
		void scale(float scale) { for (int i = 0; i < m_vertices.size(); i++) { m_vertices[i] *= scale; } }
		//normal computations
		void computeNormals();
		inline const Vec3f interpPos(Vec3f barCoord, Vec3i triangleIndices) const { return (barCoord[2] * m_vertices[triangleIndices[0]] + barCoord[0] * m_vertices[triangleIndices[1]] + barCoord[1] * m_vertices[triangleIndices[2]]); } 
		inline const Vec3f interpNorm(Vec3f barCoord, Vec3i triangleIndices) const { return normalize(barCoord[2] * m_normals[triangleIndices[0]] + barCoord[0] * m_normals[triangleIndices[1]] + barCoord[1] * m_normals[triangleIndices[2]]); } 
		//accessors
		inline std::vector<Vec3f>& vertices() { return m_vertices; }
		inline std::vector<Vec3i>& indices() { return m_indices; }
		inline Vec3<Vec3f> triangle(Vec3i triangleIndices) { return Vec3<Vec3f>(m_vertices[triangleIndices[0]], m_vertices[triangleIndices[1]], m_vertices[triangleIndices[2]]);}
		inline std::vector<Vec3f>& normals() { return m_normals; }
		inline AABB& boundingBox() { return m_boundingBox; }
		inline Material material() { return m_mat; }
		//const accessors
		inline const std::vector<Vec3f>& vertices() const { return m_vertices; }
		inline const std::vector<Vec3i>& indices() const { return m_indices; }
		inline const Vec3<Vec3f> triangle(Vec3i triangleIndices) const { return Vec3<Vec3f>(m_vertices[triangleIndices[0]], m_vertices[triangleIndices[1]], m_vertices[triangleIndices[2]]); }
		inline const std::vector<Vec3f>& normals() const { return m_normals; }
		inline const AABB& boundingBox() const { return m_boundingBox; }
		inline const Material material() const { return m_mat; }	
		//Cornell Box initializer		
};


//Primitives
static Mesh Plane(Vec3f center, Vec3f n, Vec3f u, float sideLength, Material mat)
{
	Mesh plane(mat);
	n.normalize();
	u.normalize();
	Vec3f v = normalize(cross(n, u));
	plane.vertices() = std::vector<Vec3f>{ center - (u + v) * sideLength / 2.f, center + (v - u) * sideLength / 2.f, center + (u + v) * sideLength / 2.f, center + (u - v) * sideLength / 2.f };
	plane.indices() = std::vector<Vec3i>{ Vec3i(2,1,0), Vec3i(2,0,3) };
	plane.normals() = std::vector<Vec3f>{ n, n, n, n };	
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





