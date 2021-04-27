#include "rayTracer.h"

//called to render image from scene
void RayTracer::render(const Scene& scene, Image& renderImage, size_t rayPerPixel = 8)
{
	const std::vector<Mesh>& sceneMeshes = scene.meshes();	
	const Camera& renderCam = scene.camera();
	const Vec3f& camPos = renderCam.getPosition();
	//fill background of the image with arbitrary color
	renderImage.fillBackground(Vec3f(0.5, 0.5, 0.5), Vec3f(0.1f, 0.1f, 0.1f));
	int width = renderImage.getWidth(); int height = renderImage.getHeight();
	#pragma omp parallel for schedule(dynamic, 1)  
	for (int j = 0; j < height; j++)
	{
		//display progress in percentage
		fprintf(stderr, "\rRendering (%i samples): %.2f%% ",      // Prints
			rayPerPixel, (double)j / height * 100);

		for (int i = 0; i < width; i++)
		{
			//create rays for intersection test
			Vec3f totalColorResponse(0,0,0);			
			for (int k = 0; k < rayPerPixel; k++)
			{
				float rd_width = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				float rd_height = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				Ray scatteredRay = renderCam.rayAt((i + rd_width) / (float)width, 1 - (j + rd_height) / (float)height);
				//path tracing
				totalColorResponse += pathTrace(scatteredRay, 0, scene, 0)/rayPerPixel;
			}
			totalColorResponse = Vec3f(sqrt(totalColorResponse[0]), sqrt(totalColorResponse[1]), sqrt(totalColorResponse[2]));
			renderImage(i, j) = totalColorResponse;			
		}
	}
}

Vec3f randomDirectionUnitSphere()
{
	Vec3f point;
	do
	{
		point[0] = 2.0f * (double)rand() / RAND_MAX - 1.f;
		point[1] = 2.0f * (double)rand() / RAND_MAX - 1.f;
		point[2] = 2.0f * (double)rand() / RAND_MAX - 1.f;
	} while (point.squaredLength() > 1.0f);
	return point;
}

Vec3f RayTracer::pathTrace(Ray ray, size_t current, const Scene& scene, size_t maxBounces = 10)
{
	if (current > maxBounces) return Vec3f(0,0,0);		
	Vec3f pathResponse(0,0,0);
	Vec3f intersectionPos, intersectionNormal; size_t meshIndex;
	if (RayTracer::rayTraceBVH(ray, scene, intersectionPos, intersectionNormal, meshIndex))
	{
		//direct lighting contribution
		Material hitMat = scene.meshes()[meshIndex].material();
		Vec3f directLight = RayTracer::directLightingShade(intersectionPos, intersectionNormal, hitMat, scene);
		current++;
		//Uniform random sample of the directions
		Vec3f target = intersectionPos + intersectionNormal + randomDirectionUnitSphere();
		Ray segment = Ray(intersectionPos, target);
		//Get recursively contribution of next bounce (indirect lighting contribution)
		Vec3f indirectLightColor = pathTrace(segment, current, scene, maxBounces);
		Vec3f indirectLight = indirectLightColor*hitMat.evaluateDiffuseColorResponse(intersectionNormal, normalize(segment.direction));		
		pathResponse = directLight + indirectLight;		
	}	
	return pathResponse;
}

//actually raytrace the scene with a given ray (loop over all triangles)
bool RayTracer::rayTrace(const Ray& ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex)
{
	const std::vector<Mesh>& sceneMeshes = scene.meshes();
	float zmax = std::numeric_limits<float>::max();
	bool intersectFound = false;
	for (int l = 0; l < sceneMeshes.size(); l++)
	{
		const Mesh& sceneMesh = sceneMeshes[l];
		for (int k = 0; k < sceneMesh.indices().size(); k++)
		{
			//get triangle info
			const Vec3i& triangleIndices = sceneMesh.indices()[k]; const Vec3<Vec3f>& trianglePositions = sceneMesh.triangle(triangleIndices);
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
bool RayTracer::rayTraceBVH(const Ray& ray, const Scene& scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex)
{ 
	bool intersectFound = false;
	const BVHroot& root = scene.getBVHroot();
	hitInfo hitRecord;	
	if (root.hit(ray, hitRecord, scene.meshes()))
	{
		intersectFound = true;
		meshIndex = hitRecord.meshIndex;				
		Vec3i triangleIndices = hitRecord.triangleIndices;		
		const std::vector<Vec3f>& vertices = scene.meshes()[meshIndex].vertices();
		const std::vector<Vec3f>& normals = scene.meshes()[meshIndex].normals();
		//return intersection position and normal by interpoling using barycentric coordinates
		intersectionPos = (hitRecord.barCoord[2] * vertices[triangleIndices[0]] + hitRecord.barCoord[0] * vertices[triangleIndices[1]] + hitRecord.barCoord[1] *vertices[triangleIndices[2]]);
		intersectionNormal = normalize(hitRecord.barCoord[2] * normals[triangleIndices[0]] + hitRecord.barCoord[0] * normals[triangleIndices[1]] + hitRecord.barCoord[1] * normals[triangleIndices[2]]);
	}
	return intersectFound;
}

//evaluate radiance at a given position
Vec3f RayTracer::evaluateRadiance(const Vec3f& position, const Vec3f& normal, const Material& mat, const Scene& scene, const Vec3f& lightPos, const Vec3f& lightColor)
{	
	Ray shadowRay = Ray(position, lightPos);
	Vec3f shadowInterPos, shadowInterNormal; size_t shadowMeshIndex;	
	if (RayTracer::rayTraceBVH(shadowRay, scene, shadowInterPos, shadowInterNormal, shadowMeshIndex))
	{		
		return Vec3f(0,0,0);
	}
	return lightColor*mat.evaluateDiffuseColorResponse(normalize(normal), normalize(lightPos - position));
}

Vec3f RayTracer::directLightingShade(const Vec3f& position, const Vec3f& normal, const Material& mat, const Scene& scene)
{
	const std::vector<lightPtr>& lights = scene.lightSources();	
	return evaluateRadiance(position, normal, mat, scene, lights[0]->getPosition(), lights[0]->colorResponse());
}
