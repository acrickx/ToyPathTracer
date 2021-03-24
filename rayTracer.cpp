#include "rayTracer.h"


//called to render image from scene
void RayTracer::render(Scene scene, Image& renderImage, size_t rayPerPixel = 8)
{
	std::vector<Mesh> sceneMeshes = scene.meshes();
	Camera renderCam = scene.camera();
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
			bool intersectionFound = false;
			bool intersection = false;
			for (int k = 0; k < rayPerPixel; k++)
			{
				Ray scatteredRay(renderCam.getPosition(), normalize(renderCam.getImageCoordinate(float(i) / width + ((std::rand() % 100) / (float)100) * (1 / (float)width), 1.f - (((float)j / height) + ((std::rand() % 100) / (float)100) * (1 / (float)height)))));
				Vec3f intersectionPos, intersectionNormal; size_t meshIndex;
				intersectionFound = rayTraceBVH(scatteredRay, scene, intersectionPos, intersectionNormal, meshIndex);
				if (intersectionFound)
				{
					if (!intersection) intersection = true;
					totalColorResponse += shade(intersectionPos, intersectionNormal, sceneMeshes[meshIndex].material(), scene) / (float)rayPerPixel;
				}
			}
			if (intersection) renderImage(i, j) = totalColorResponse;
		}
	}
}

//actually raytrace the scene with a given ray (loop over all triangles)
bool RayTracer::rayTrace(Ray ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex)
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
				if (parT >0 && zmax > parT)
				{
					intersectFound = true;
					zmax = parT;
					//return mesh index
					meshIndex = l;					
					//return intersection position and normal by interpoling using barycentric coordinates
					intersectionPos = sceneMesh.interpPos(barCoord, triangleIndices);
					intersectionNormal = sceneMesh.interpNorm(barCoord,triangleIndices);
				}
			}
		}
	}
	return intersectFound;
}

//BVH raytracer
bool RayTracer::rayTraceBVH(Ray ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex)
{
	bool intersectFound = false;
	BVHroot root = scene.getBVHroot();
	hitInfo hitRecord;	
	if (root.hit(ray, hitRecord, scene.meshes()))
	{
		intersectFound = true;
		meshIndex = hitRecord.meshIndex;				
		Vec3i triangleIndices = hitRecord.triangleIndices;
		Mesh sceneMesh = scene.meshes()[meshIndex];
		//return intersection position and normal by interpoling using barycentric coordinates
		intersectionPos = (hitRecord.barCoord[2] * sceneMesh.vertices()[triangleIndices[0]] + hitRecord.barCoord[0] * sceneMesh.vertices()[triangleIndices[1]] + hitRecord.barCoord[1] * sceneMesh.vertices()[triangleIndices[2]]);
		intersectionNormal = normalize(hitRecord.barCoord[2] * sceneMesh.normals()[triangleIndices[0]] + hitRecord.barCoord[0] * sceneMesh.normals()[triangleIndices[1]] + hitRecord.barCoord[1] * sceneMesh.normals()[triangleIndices[2]]);
	}
	return intersectFound;
}

Vec3f RayTracer::shade(Vec3f position, Vec3f normal, Material mat, const Scene& scene)
{
	std::vector<lightPtr> lights = scene.lightSources();
	Vec3f totalColor = mat.albedo;
	for (int i = 0; i < lights.size(); i++)
	{
		Ray shadowRay = Ray(position + normal * 0.001f, lights[i]->getPosition());
		Vec3f shadowInterPos, shadowInterNormal; size_t shadowMeshIndex;
		bool shadowIntersect = rayTrace(shadowRay, scene, shadowInterPos, shadowInterNormal, shadowMeshIndex);
		if (!shadowIntersect || (position - shadowInterPos).length() <= 0.0001f)
		{
			totalColor += mat.evaluateColorResponse(position, normal, lights[i], scene.camera());			
		}
	}
	return totalColor;
}
