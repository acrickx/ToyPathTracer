#pragma once

#include"Vec3.h"
#include"ray.h"
#include"image.h"
#include"microbuffer.h"
#include"BVHnode.h"
#include"scene.h"
#include"pointCloud.h"
#include<vector>

class  PointBasedRenderer
{
	public:				
		Image render(const Scene& scene, const PointCloud& pointCloud, Image& renderImage, size_t microbuffersize);
		bool rayTrace(Ray ray, Scene scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex, size_t& triangleIndex);
	private:

};