#pragma once
#include"Vec3.h"
#include"image.h"
#include"scene.h"

class RayTracer
{
	public:		
		Image render(Scene scene, Image& image, size_t rayPerPixel);
		//debug point cloud
		Image render(const std::vector<Vec3<Vec3f>>& positions, const std::vector<Vec3<Vec3f>>& normals, const Camera& renderCam, const std::vector<lightPtr>& lights, const Material& material, Image& renderImage);		
	private:				
		bool rayTrace(Ray ray, Scene scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex, size_t& triangleIndex);
		bool rayTrace(Ray ray, const std::vector<Vec3<Vec3f>>& positions, const std::vector<Vec3<Vec3f>>& normals, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& triangleIndex);
		bool rayTrace(Ray ray, Scene scene);
		Vec3f shade(Mesh mesh, Vec3f position, Vec3f normal, const Scene& scene);
};

