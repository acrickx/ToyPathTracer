#pragma once
#include"Vec3.h"
#include"image.h"
#include"scene.h"
#include"GeometryHelper.h"

class RayTracer
{
	public:		
		static void render(const Scene& scene, Image& image, size_t rayPerPixel, size_t bounces);

		static bool rayTrace(const Ray& ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex);		

		static bool rayTraceBVH(const Ray& ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex);	

		static Vec3f evalDirect(const Vec3f& position, const Vec3f& normal, MaterialPtr mat, const Scene& scene);

		static Vec3f pathTrace(const Ray& ray, size_t current, const Scene& scene, size_t maxBounces);

		static Vec3f pathTrace(const Ray& ray, int depth, const Scene& scene);

		static Vec3f tracePath(const Vec3f& origin, const Vec3f& normal, MaterialPtr material, size_t nBounces, const Scene& scene);
};

