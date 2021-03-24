#include "boundingVolume.h"

bool AABB::hit(Ray ray, float& tmin, float& tmax) const
{
	tmin = std::numeric_limits<float>().min();
	tmax = std::numeric_limits<float>().max();
	//look for both intersections time in each direction
	for (int i = 0; i < 3; i++)
	{
		float invD = 1.0f / (float)ray.direction[i];
		float t0 = (m_minCorner[i] - ray.origin[i]) * invD;
		float t1 = (m_maxCorner[i] - ray.origin[i]) * invD;
		//swap order if ray goes in negative direction
		if (invD < 0) { std::swap(t0, t1); } 
		tmin = t0 > tmin ? t0 : tmin;
		tmax = t1 < tmax ? t1 : tmax;
		if (tmax <= tmin) return false;
	}
	return true;
}

bool AABB::hit(Ray ray) const
{
	float tmin = std::numeric_limits<float>().min();
	float tmax = std::numeric_limits<float>().max();
	//look for both intersections time in each direction
	for (int i = 0; i < 3; i++)
	{
		float invD = 1.0f / (float)ray.direction[i];
		float t0 = (m_minCorner[i] - ray.origin[i]) * invD;
		float t1 = (m_maxCorner[i] - ray.origin[i]) * invD;
		//swap order if ray goes in negative direction
		if (invD < 0) { std::swap(t0, t1); }
		tmin = t0 > tmin ? t0 : tmin;
		tmax = t1 < tmax ? t1 : tmax;
		if (tmax <= tmin) return false;
	}
	return true;
}