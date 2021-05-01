#pragma once
#include"Vec3.h"
#include"image.h"
#include"scene.h"

class RayTracer
{
	public:		
		static void render(const Scene& scene, Image& image, size_t rayPerPixel);
		//debug point cloud		
		static bool rayTrace(const Ray& ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex);		
		static bool rayTraceBVH(const Ray& ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex);		
		static Vec3f directLightingShade(const Vec3f& position, const Vec3f& normal, const Material& mat, const Scene& scene);	
		static Vec3f evaluateRadiance(const Vec3f& position, const Vec3f& normal, const Material& mat, const Scene& scene, const Vec3f& lightPos, const Vec3f& lightColor);		
		static Vec3f pathTrace(const Ray& ray, size_t current, const Scene& scene, size_t maxBounces);
		static Vec3f pathTrace(const Ray& ray, int depth, const Scene& scene);
};

