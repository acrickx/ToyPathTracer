#pragma once
#include "Vec3.h"

struct Surfel
{
	Vec3f position;
	Vec3f normal;
	Vec3f tangent;
	Vec3f color;
	float radius;
	inline Surfel(Vec3f initPos, Vec3f initNorm, Vec3f initTan, float initRad) : position(initPos), normal(initNorm), tangent(initTan), radius(initRad) {}
};
