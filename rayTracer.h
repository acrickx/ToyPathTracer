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
		static Vec3f shade(const Vec3f& position, const Vec3f& normal, const Material& mat, const Scene& scene);			
};

