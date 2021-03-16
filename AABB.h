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

