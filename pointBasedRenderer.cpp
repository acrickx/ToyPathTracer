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
	Camera renderCam = scene.camera();
	std::cout << surfels.size() << std::endl;
	Material surfelMat;
	int coloredPixel=0;
	int blankPixel = 0;
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
			Ray ray(renderCam.getPosition(), normalize(renderCam.getImageCoordinate(float(i) / width, 1.f - (float)j / height)));
			float zbuffer = std::numeric_limits<float>().max();
			bool intersect = false;
			for (int k = 0; k < surfels.size(); k++)
			{
				Surfel surfel = surfels[k];				
				Vec3f intersectionPos; float parT;
				bool intersectionFound = false;
				Vec3f pt1 = surfel.position - surfel.radius * surfel.tangent;
				Vec3f pt2 = surfel.position + surfel.radius * surfel.tangent;
				Vec3f pt3 = surfel.position + surfel.radius * normalize(cross(surfel.normal, surfel.tangent));
				Vec3<Vec3f> triangle(pt1, pt2, pt3);
				Vec3f barcoord;
				//intersectionFound = ray.testTriangleIntersection(triangle, barcoord, parT);
				intersectionFound = ray.testDiscIntersection(surfel.position, surfel.normal, surfel.radius, intersectionPos, parT);
				if (intersectionFound && parT<zbuffer)
				{
					if (!intersect) intersect = true;					
					zbuffer = parT;
					surfelMat.albedo = surfel.color;
					renderImage(i, j) = surfelMat.albedo + surfelMat.evaluateColorResponse(surfel.position, surfel.normal, lights[0], renderCam);
				}				
			}
			if (!intersect)
			{
				blankPixel++;
			}
			else coloredPixel++;
		}
	}
	std::cout << " coloredPixels : " << coloredPixel << " - Blank Pixels : " << blankPixel << std::endl;
	return renderImage;
}
