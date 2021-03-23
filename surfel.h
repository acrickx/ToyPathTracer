#pragma once
#include "Vec3.h"

struct Surfel
{
	Vec3f position;
	Vec3f normal;	
	Vec3f color;
	float radius;
	inline Surfel(Vec3f initPos, Vec3f initNorm, Vec3f initColor, float initRad) : position(initPos), normal(normalize(initNorm)), radius(initRad), color(initColor) {}
	inline Surfel() { radius = 0; }
};
