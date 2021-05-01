#pragma once
#include"Vec3.h"
#include"lightSource.h"
#include"camera.h"


enum MaterialType
{
	Diffuse,
	Specular,
	Emissive
};

class Material
{

	public:
	Vec3f albedo;
	Vec3f emissive;
	MaterialType matType;

	//Methods
	Material(Vec3f _albedo, MaterialType type, Vec3f _emissive = Vec3f(0,0,0)) : albedo(_albedo), matType(type), emissive(_emissive) {};

	Vec3f colorResponse(const Vec3f& normal, const Vec3f& direction) const 
	{
		return (albedo/M_PI) * std::max(dot(normal, direction), 0.f);
	}
};
