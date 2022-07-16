#pragma once
#include"Vec3.h"
#include"lightSource.h"
#include"camera.h"

class Material
{
public:
	Vec3f albedo;
	float diffuse;
	float roughness;
	float metallic;

	//methods
	Material() { albedo = Vec3f(0.5f, 0.5f, 0.5f); diffuse = 1.f; roughness = 0.5f; metallic = 0.5f; }

	Material(Vec3f _albedo, float _diffuse, float _roughness, float _specular) : albedo(_albedo), diffuse(_diffuse), roughness(_roughness), metallic(_specular) {};

	Vec3f colorResponse(const Vec3f& position, const Vec3f& normal, const lightPtr& light, const Camera& eye) const {
		float diffuseResponse(diffuse / (float)M_PI);
		const Vec3f& wi = normalize(light->getPosition() - position);
		Vec3f w0 = normalize(eye.getPosition() - position);
		const Vec3f& specularResponse = reflectance(wi, w0, normal);
		Vec3f colorResponse = light->colorResponse() * (Vec3f(diffuseResponse)) * std::max(dot(normal, wi), 0.f);
		return colorResponse;
	};

	Vec3f colorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& direction, const Vec3f& eyePos) const {
		float diffuseResponse(diffuse / (float)M_PI);
		const Vec3f& wi = direction;
		Vec3f w0 = normalize(eyePos - position);
		const Vec3f& specularResponse = reflectance(wi, w0, normal);
		Vec3f colorResponse = (Vec3f(diffuseResponse) + specularResponse) * std::max(dot(normal, wi), 0.f);
		return colorResponse;
	};

	Vec3f evaluateDiffuseColorResponse(const Vec3f& normal, const Vec3f& direction) const {
		return (albedo/M_PI) * std::max(dot(normal, direction), 0.f);
	}

	Ray pureDiffuseReflectedRay(const Vec3f& direction, Vec3f& position, const Vec3f& normal) const
	{
		Vec3f nl = dot(normal,direction) < 0 ? normal : normal * -1;
		double r1 = 2 * M_PI * (float)rand()/RAND_MAX, r2 = (float)rand() / RAND_MAX, r2s = sqrt(r2);
		Vec3f w = nl;
		Vec3f u = normalize(cross((fabs(w[0]) > .1 ? Vec3f(0, 1,0) : Vec3f(1,0,0)), w));
		Vec3f v = cross(w,u);
		Vec3f d = normalize(u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2));
		Ray reflected;
		reflected.m_origin = position+0.01f*normal; reflected.m_direction = d;
		return reflected;
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

	float G(const Vec3f& w, const Vec3f& n, float alpha) const
	{
		return 2.0 * (dot(n, w)) / (dot(n, w) + sqrt(pow(alpha, 2) + (1 - pow(alpha, 2)) * pow(dot(n, w), 2)));
	}

	float G_GGX(const Vec3f& wi, const Vec3f& wo, float alpha, const Vec3f& n) const
	{
		return G(wi, n, alpha) * G(wo, n, alpha);
	}

	Vec3f F(float cosTheta, const Vec3f& f0) const
	{
		return f0 + (Vec3f(1.f, 1.f, 1.f) - f0) * pow(1.0 - cosTheta, 5.0);
	}
	float D(float alpha, const Vec3f& m, const Vec3f& n) const
	{
		return (pow(alpha, 2)) / (3.1415926 * pow((1 + pow(dot(n, m), 2) * (pow(alpha, 2) - 1)), 2));
	}
};
