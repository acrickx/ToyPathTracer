#pragma once

#include"Vec3.h"
#include"ray.h"
#include"rayTracer.h"
#include"microbuffer.h"
#include"BVHnode.h"
#include"pointCloud.h"
#include<vector>

class  PointBasedRenderer
{
	public:				
		static Image render(const Scene& scene, const PointCloud& pointCloud, Image& renderImage, size_t microbuffersize, size_t rayPerPixel);
		static Image renderPointCloud(const PointCloud& pointCloud, const Scene& scene, Image& renderImage);		
};



