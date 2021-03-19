#include "pointBasedRenderer.h"


//called to render image from scene
Image PointBasedRenderer::render(const Scene& scene, const PointCloud& pointCloud, Image& renderImage, size_t microBufferSize)
{
	std::vector<Mesh> sceneMeshes = scene.meshes();
	Camera renderCam = scene.camera();
	BVHnode::BVHptr root = pointCloud.BVHroot();
	//fill background of the image with arbitrary color
	renderImage.fillBackground(Vec3f(1, 0, 1), Vec3f(0.5f, 0.5f, 1));
	int width = renderImage.getWidth(); int height = renderImage.getHeight();
	for (int j = 0; j < height; j++)
	{
		//display progress with dots
		size_t progress = 0;
		if (progress % 10 == 0) std::cout << ".";
		progress++;
		#pragma omp parallel for
		for (int i = 0; i < width; i++)
		{
			//create rays for intersection test
			Vec3f totalColorResponse;
			bool intersectionFound = false;	
			Ray ray(renderCam.getPosition(), normalize(renderCam.getImageCoordinate(float(i) / width, 1.f - (float)j / height)));
			Vec3f intersectionPos, intersectionNormal; size_t meshIndex; size_t triangleIndex;
			intersectionFound = rayTrace(ray, scene, intersectionPos, intersectionNormal, meshIndex, triangleIndex);			
			if (intersectionFound)
			{				
				//microrendering
				MicroBuffer mBuffer(microBufferSize, intersectionPos, intersectionNormal);				
				mBuffer.fillMicroBuffer(root);
				mBuffer.postTraversalRayCasting();				
			}			
		}
	}
	return renderImage;
}

//actually raytrace the scene with a given ray
bool PointBasedRenderer::rayTrace(Ray ray, Scene scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex, size_t& triangleIndex)
{
	std::vector<Mesh> sceneMeshes = scene.meshes();
	float zmax = std::numeric_limits<float>::max();
	bool intersectFound = false;
	for (int l = 0; l < sceneMeshes.size(); l++)
	{
		Mesh sceneMesh = scene.meshes()[l];
		#pragma omp parallel for
		for (int k = 0; k < sceneMesh.indices().size(); k++)
		{
			//get triangle info
			Vec3i triangleIndices = sceneMesh.indices()[k]; Vec3<Vec3f> trianglePositions = sceneMesh.triangle(triangleIndices);
			//test intersection
			Vec3f barCoord; float parT;
			if (ray.testTriangleIntersection(trianglePositions, barCoord, parT))
			{
				//zmax for z buffer test				
				if (parT > 0 && zmax > parT)
				{
					intersectFound = true;
					zmax = parT;
					//return mesh index
					meshIndex = l;
					triangleIndex = k;
					//return intersection position and normal by interpoling using barycentric coordinates
					intersectionPos = sceneMesh.interpPos(barCoord, triangleIndices);
					intersectionNormal = sceneMesh.interpNorm(barCoord, triangleIndices);
				}
			}
		}
	}
	return intersectFound;
}
