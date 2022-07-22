#pragma once
#include"Vec3.h"
#include"lightSource.h"
#include"camera.h"


class Material
{
public:
	enum MaterialType
	{
		GGX,
		EMISSIVE,
	};

	virtual Vec3f colorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& direction, const Vec3f& viewPoint) = 0;

	MaterialType type;
};

class MaterialGGX : public Material
{
public:
	Vec3f albedo;
	float roughness;
	float metallic;

	//methods
	MaterialGGX() 
	{ 
		albedo = Vec3f(0.5f, 0.5f, 0.5f); 
		roughness = 0.0f; 
		metallic = 0.0f; 
		type = MaterialType::GGX;
	}

	MaterialGGX(Vec3f _albedo, float _diffuse, float _roughness, float _specular) : albedo(_albedo), roughness(_roughness), metallic(_specular) { type = MaterialType::GGX; };
	MaterialGGX(Vec3f _albedo) : albedo(_albedo) { roughness = 1.0f; metallic = 0.0f; type = MaterialType::GGX; };

	Vec3f Material::colorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& direction, const Vec3f& viewPoint) override
	{
		return evalBSDFCosine(position, normal, direction, viewPoint);
	}

	Vec3f evalBSDFCosine(const Vec3f& position, const Vec3f& normal, const Vec3f& direction, const Vec3f& viewPoint) const
	{
		Vec3f diffuseResponse(albedo / (float)M_PI);
		Vec3f specularResponse = reflectance(direction, normalize(viewPoint - position), normal);
		return (diffuseResponse + specularResponse) * std::max(dot(normal, direction), 0.f);
	};

	Vec3f diffuseBSDFCosine(const Vec3f& normal, const Vec3f& direction) const
	{
		return (albedo / M_PI) * std::max(dot(normal, direction), 0.f);
	}

private:

	inline Vec3f reflectance(const Vec3f& wi, const Vec3f& wo, const Vec3f& n) const
	{
		float alpha = roughness * roughness;
		Vec3f wh = normalize(wi + wo);
		Vec3f F0(0.04f);
		F0 = mix(F0, albedo, metallic);
		float term1 = G_GGX(wi, wo, alpha, n);
		Vec3f term2 = F(std::max(dot(wi, wh), 0.f), F0);
		float term3 = D(alpha, wh, n);
		return term1 * term2 * term3 / (4 * dot(n, wi) * dot(n, wo));
	}

	inline float G(const Vec3f& w, const Vec3f& n, float alpha) const
	{
		return 2.0 * (dot(n, w)) / (dot(n, w) + sqrt(pow(alpha, 2) + (1 - pow(alpha, 2)) * pow(dot(n, w), 2)));
	}

	inline float G_GGX(const Vec3f& wi, const Vec3f& wo, float alpha, const Vec3f& n) const
	{
		return G(wi, n, alpha) * G(wo, n, alpha);
	}

	inline Vec3f F(float cosTheta, const Vec3f& f0) const
	{
		return f0 + (Vec3f(1.f, 1.f, 1.f) - f0) * pow(1.0 - cosTheta, 5.0);
	}

	inline float D(float alpha, const Vec3f& m, const Vec3f& n) const
	{
		return (pow(alpha, 2)) / (3.1415926 * pow((1 + pow(dot(n, m), 2) * (pow(alpha, 2) - 1)), 2));
	}
};

class MaterialEmissive : public Material
{
public :
	float emissive = 1.0f;
	Vec3f color{ 1.0f, 1.0f, 1.0f };

	MaterialEmissive(const Vec3f& _color, float _emissive) : emissive(_emissive), color(_color) { type = Material::EMISSIVE; };

	Vec3f Material::colorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& direction, const Vec3f& viewPoint) override
	{
		return Vec3f(emissive * color);
	}
};
