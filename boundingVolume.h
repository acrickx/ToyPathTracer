#pragma once
#include"Vec3.h"
#include"ray.h"
#include<vector>

class AABB
{
private:
	Vec3f m_minCorner;
	Vec3f m_maxCorner;
public:
	AABB() : m_minCorner(Vec3f(1, 1, 1)*std::numeric_limits<float>::max()), m_maxCorner(-m_minCorner) {};
	AABB(Vec3f minC, Vec3f maxC) : m_minCorner(minC), m_maxCorner(maxC) {};
	AABB(std::vector<Vec3f> vertices) 
	{
		m_minCorner = Vec3f(1, 1, 1) * std::numeric_limits<float>::max();
		m_maxCorner = -m_minCorner;
		for (int i = 0; i < vertices.size(); i++)
		{
			compareAndUpdate(vertices[i]);
		}
	};
	inline void compareAndUpdate(Vec3f pos)
	{
		for (int k = 0 ; k < 3; k++)
		{
			if (m_minCorner[k] > pos[k]) m_minCorner[k] = pos[k];
			if (m_maxCorner[k] < pos[k]) m_maxCorner[k] = pos[k];
		}
	}
	bool hit(Ray ray, float& tmin, float& tmax) const;
	bool hit(Ray ray) const;
	inline bool contains(const Vec3f& position) const
	{
		if (m_minCorner[0] <= position[0] && position[0] <= m_maxCorner[0] 
		 && m_minCorner[1] <= position[1] && position[1] <= m_maxCorner[1] 
		 && m_minCorner[2] <= position[2] && position[2] <= m_maxCorner[2])
		{
			return true;
		}
		return false;
	}
	inline const Vec3f min() const { return m_minCorner; };
	inline const Vec3f max() const { return m_maxCorner; };
	inline Vec3f& min() { return m_minCorner; };
	inline Vec3f& max() { return m_maxCorner; };
};

class Sphere
{
private:
	Vec3f m_center;
	float m_radius;
public:
	Sphere() : m_center(Vec3f(0, 0, 0)), m_radius(0.f) {};
	Sphere(Vec3f center, float radius) : m_center(center), m_radius(radius) {};	
	inline bool contains(const Vec3f& position) const
	{
		if ((m_center - position).length() < m_radius) return true;
		else return false;
	}
	//accessors
	inline Vec3f center() { return m_center; };
	inline float radius() { return m_radius; };
	inline const Vec3f center() const { return m_center; };
	inline const float radius() const  { return m_radius; };
	//modifiers
	inline void setRadius(float newVal) { m_radius = newVal; }
};						  

