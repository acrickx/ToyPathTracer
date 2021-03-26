#include "pointBasedRenderer.h"


//called to render image from scene
Image PointBasedRenderer::render(const Scene& scene, const PointCloud& pointCloud, Image& renderImage, size_t microBufferSize, size_t rayPerPixel)
{
	const std::vector<Mesh>& sceneMeshes = scene.meshes();
	const Camera& renderCam = scene.camera();
	const BSHnode::BSHptr& root = pointCloud.BVHroot();
	//fill background of the image with arbitrary color
	renderImage.fillBackground(Vec3f(0.5, 0.5, 0.5), Vec3f(0.1f, 0.1f, 0.1f));
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
			bool intersection=false;
			for (int k = 0; k < rayPerPixel; k++)
			{
				Ray scatteredRay;
				if (rayPerPixel == 1) scatteredRay = Ray(renderCam.getPosition(), normalize(renderCam.getImageCoordinate((float(i) / width), (1.f - (float)j / height))));
				else scatteredRay = Ray(renderCam.getPosition(), normalize(renderCam.getImageCoordinate(float(i) / width + ((std::rand() % 100) / (float)100) * (1 / (float)width), 1.f - (((float)j / height) + ((std::rand() % 100) / (float)100) * (1 / (float)height)))));
				Vec3f intersectionPos, intersectionNormal; size_t meshIndex;
				bool intersectionFound = RayTracer::rayTraceBVH(scatteredRay, scene, intersectionPos, intersectionNormal, meshIndex);
				if (intersectionFound)
				{
					//microrendering
					MicroBuffer mBuffer(microBufferSize, intersectionPos + 0.01f * intersectionNormal, intersectionNormal);
					mBuffer.fillMicroBuffer(root);
					mBuffer.postTraversalRayCasting();
					const Material& mat = scene.meshes()[meshIndex].material();
					totalColorResponse += mBuffer.convolveBRDF(mat, scene)/(float)rayPerPixel;
					intersection = true;
				}
			}
			if (intersection) renderImage(i, j) = totalColorResponse;
		}
	}
	return renderImage;
}

//called to render image from positions, colors and normals directly (debug point cloud)
Image PointBasedRenderer::renderPointCloud(const PointCloud& pointCloud, const Scene& scene, Image& renderImage)
{
	std::vector<Surfel> surfels = pointCloud.surfels();
	std::vector<lightPtr> lights = scene.lightSources();
	Camera renderCam = scene.camera();	
	Material surfelMat;
	int coloredPixel=0;
	int blankPixel = 0;
	//fill background of the image with arbitrary color
	renderImage.fillBackground(Vec3f(0.5, 0.5, 0.5), Vec3f(0.1f, 0.1f, 0.1f));
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
			Ray ray(renderCam.getPosition(), renderCam.getImageCoordinate(i/(float)width, 1.f - j/(float)height));
			float zbuffer = std::numeric_limits<float>().max();
			bool intersect = false;
			#pragma omp parallel for
			for (int k = 0; k < surfels.size(); k++)
			{				
				Surfel surfel = surfels[k];				
				Vec3f intersectionPos; float parT;
				bool intersectionFound = false;				
				intersectionFound = ray.testDiscIntersection(surfel.position, surfel.normal, surfel.radius, intersectionPos, parT);
				if (intersectionFound && parT<zbuffer)
				{
					if (!intersect) intersect = true;					
					zbuffer = parT;
					surfelMat.albedo = surfel.color;
					renderImage(i, j) = surfelMat.albedo;
					for (int l = 0; l < lights.size(); l++)
					{
						renderImage(i, j) += surfelMat.evaluateColorResponse(surfel.position, surfel.normal, lights[l], renderCam);
					}
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
