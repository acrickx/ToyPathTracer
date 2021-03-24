#pragma once
#include"Vec3.h"
#include"image.h"
#include"scene.h"

class RayTracer
{
	public:		
		static void render(Scene scene, Image& image, size_t rayPerPixel);
		//debug point cloud		
		static bool rayTrace(Ray ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex);		
		static bool rayTraceBVH(Ray ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex);				
		static Vec3f shade(Vec3f position, Vec3f normal, Material mat, const Scene& scene);			
};

