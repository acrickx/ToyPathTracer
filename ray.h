#pragma once
struct Ray
{
	Vec3f origin;
	Vec3f direction;
	Ray(Vec3f _from, Vec3f _to) : origin(_from), direction(normalize(_to - _from)) {};
	Ray() : origin(Vec3f(0, 0, 0)), direction(Vec3f(0, 0, 1)) {};
};