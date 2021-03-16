#pragma once
#include"Vec3.h"
#include"ray.h"

class AABB
{
private:
	Vec3f m_minCorner;
	Vec3f m_maxCorner;
public:
	AABB() : m_minCorner(Vec3f(1, 1, 1)*std::numeric_limits<float>::max()), m_maxCorner(-m_minCorner) {};
	AABB(Vec3f minC, Vec3f maxC) : m_minCorner(minC), m_maxCorner(maxC) {};
	inline void compareAndUpdate(Vec3f pos)
	{
		if (m_minCorner[0] > pos[0]) m_minCorner[0] = pos[0];
		if (m_maxCorner[0] < pos[0]) m_maxCorner[0] = pos[0];
		if (m_minCorner[1] > pos[1]) m_minCorner[1] = pos[1];
		if (m_maxCorner[1] < pos[1]) m_maxCorner[1] = pos[1];
		if (m_minCorner[2] > pos[2]) m_minCorner[2] = pos[2];
		if (m_maxCorner[2] < pos[2]) m_maxCorner[2] = pos[2];
	}
	bool hit(Ray ray, float& tmin, float& tmax) const;
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
};

class Sphere
{
private:
	Vec3f m_center;
	float m_radius;
public:
	Sphere() : m_center(Vec3f(0, 0, 0)), m_radius(0.f) {};
	Sphere(Vec3f center, float radius) : m_center(center), m_radius(radius) {};
	bool hit(Ray ray, float& tmin, float& tmax) const;
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

