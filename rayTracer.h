#pragma once
#include"Vec3.h"
#include"image.h"
#include"scene.h"

class RayTracer
{
	public:		
		Image render(Scene scene, Image& image, size_t rayPerPixel);
		//debug point cloud		
		static bool rayTrace(Ray ray, Scene scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex, size_t& triangleIndex);
		static bool rayTrace(Ray ray, const std::vector<Vec3<Vec3f>>& positions, const std::vector<Vec3<Vec3f>>& normals, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& triangleIndex);
		static bool rayTrace(Ray ray, Scene scene);
		static Vec3f shade(Vec3f position, Vec3f normal, Material mat, const Scene& scene);			
};

