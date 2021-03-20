#pragma once
#include"surfel.h"
struct Ray
{
	Vec3f origin;
	Vec3f direction;
	Ray(Vec3f _from, Vec3f _to) : origin(_from), direction(normalize(_to - _from)) {};
	Ray() : origin(Vec3f(0, 0, 0)), direction(Vec3f(0, 0, 1)) {};

	bool testTriangleIntersection(Vec3<Vec3f> trianglePos, Vec3f& barCoord, float& parT, float threshold = 0.0001f)
	{
		Vec3f e0 = trianglePos[1] - trianglePos[0];
		Vec3f e1 = trianglePos[2] - trianglePos[0];
		Vec3f q = cross(direction, e1);
		float a = dot(e0, q);
		if (fabs(a) < threshold)
		{
			return false;
		}
		Vec3f s = origin - trianglePos[0];
		Vec3f r = cross(s, e0);
		float b0 = dot(s, q) / a;
		float b1 = dot(r, direction) / a;
		float b2 = 1 - b0 - b1;
		if (b0 < 0 || b1 < 0 || b0 > 1 || b1>1)
		{
			return false;
		}
		float t = dot(e1, r) / a;
		if (b0 >= 0.f && b1 >= 0.f && b2 >= 0.f && b0 + b1 + b2 <= 1.f)
		{
			barCoord = Vec3f(b0, b1, b2);
			parT = t;
			return true;
		}
		return false;
	}

	bool testPlaneIntersection(Vec3f planePos, Vec3f planeNormal, Vec3f& intersectionPos, float& parT, float threshold = 0.0001f)
	{
		normalize(planeNormal);
		float denom = dot(direction, planeNormal);
		if(abs(denom) > 1e-6)
		{
			parT = dot(planePos - origin, planeNormal)/denom;
			if (parT >= 0)
			{
				intersectionPos = origin + parT * direction;
				return true;
			}
		}
		return false;
	}

	bool testDiscIntersection(Vec3f discPos, Vec3f discNormal, float radius, Vec3f& intersectionPos, float& parT, float threshold = 0.0001f)
	{
		if (testPlaneIntersection(discPos, discNormal, intersectionPos, parT, threshold))
		{			
			if ((intersectionPos - discPos).length() <= radius)
			{								
				return true;
			}
			else
			{				
				return false;
			}
		}
		else return false;
	}

	bool testSurfelIntersection(Surfel surfel, Vec3f& intersectionPos, float& parT, float threshold = 0.0001f)
	{
		return testDiscIntersection(surfel.position, surfel.normal, surfel.radius, intersectionPos, parT, threshold);
	}
};