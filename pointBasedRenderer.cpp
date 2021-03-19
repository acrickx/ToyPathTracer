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
			intersectionFound = RayTracer::rayTrace(ray, scene, intersectionPos, intersectionNormal, meshIndex, triangleIndex);			
			if (intersectionFound)
			{				
				//microrendering
				MicroBuffer mBuffer(microBufferSize, intersectionPos, intersectionNormal);				
				mBuffer.fillMicroBuffer(root);
				mBuffer.postTraversalRayCasting();
				renderImage(i,j) = mBuffer.convolveBRDF(scene.meshes()[meshIndex].material(), scene);
			}			
		}
	}
	return renderImage;
}

//called to render image from positions, colors and normals directly (debug point cloud)
Image PointBasedRenderer::renderPointCloud(PointCloud pointCloud, const Scene& scene, Image& renderImage)
{
	std::vector<Surfel> surfels = pointCloud.surfels();
	std::vector<lightPtr> lights = scene.lightSources();
	std::cout << surfels.size() << std::endl;
	Material surfelMat;
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
			Ray ray(scene.camera().getPosition(), scene.camera().getImageCoordinate(float(i) / width, 1.f - (float)j / height));
			float zbuffer = 1000000;
			for (int k = 0; k < surfels.size(); k++)
			{
				Surfel surfel = surfels[k];
				Vec3f intersectionPos; float parT;
				bool intersectionFound = false;
				intersectionFound = ray.testDiscIntersection(surfel.position, surfel.normal, surfel.radius, intersectionPos, parT);
				if (intersectionFound && parT <zbuffer)
				{
					zbuffer = parT;
					renderImage(i,j) = surfelMat.albedo + surfelMat.evaluateColorResponse(surfel.position, surfel.normal, lights[0], scene.camera());
				}
			}
		}
	}
	return renderImage;
}
