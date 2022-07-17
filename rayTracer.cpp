#include "rayTracer.h"

//called to render image from scene
void RayTracer::render(const Scene& scene, Image& renderImage, size_t rayPerPixel = 8, size_t bounces = 0)
{
	const std::vector<Mesh>& sceneMeshes = scene.meshes();	
	const Camera& renderCam = scene.camera();
	const Vec3f& camPos = renderCam.getPosition();
	//fill background of the image with arbitrary color
	renderImage.fillBackground(Vec3f(0.5, 0.5, 0.5), Vec3f(0.1f, 0.1f, 0.1f));
	int width = renderImage.getWidth(); 
	int height = renderImage.getHeight();
	#pragma omp parallel for schedule(dynamic, 1)  
	for (int y = 0; y < height; y++)
	{
		//display progress in percentage
		fprintf(stderr, "\rRendering (%i samples): %.2f%% ",rayPerPixel, (double)y / height * 100);
		for (int x = 0; x < width; x++)
		{
			//create rays for intersection test
			Vec3f totalColorResponse(0,0,0);			
			for (int k = 0; k < 1; k++)
			{
				//float rd_width = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				//float rd_height = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				Ray scatteredRay = renderCam.rayAt(float(x) / width, 1.f - float(y) / height);
				//Ray tracing 
				Vec3f hitPosition, hitNormal;
				size_t meshIndex;
				if (rayTraceBVH(scatteredRay, scene, hitPosition, hitNormal, meshIndex))
				{
					// Direct Illumination
					Material hitMat = scene.meshes()[meshIndex].material();
					totalColorResponse += directLightingShade(hitPosition, hitNormal, hitMat, scene);
					// Indirect Illumination
					Vec3f randomDirection = randomDirectionUnitSphere();
					Ray reflectionRay = Ray(hitPosition + 0.01f * hitNormal, normalize(randomDirection));
					Vec3f secondaryHitPosition, secondaryHitNormal;
					size_t secondaryHitMesh;
					if (rayTraceBVH(reflectionRay, scene, secondaryHitPosition, secondaryHitNormal, secondaryHitMesh))
					{
						Material secondaryHitMat = scene.meshes()[secondaryHitMesh].material();
						Vec3f lightAtHit = directLightingShade(secondaryHitPosition, secondaryHitNormal, secondaryHitMat, scene);
						totalColorResponse += lightAtHit / M_PI * dot(normalize(secondaryHitPosition - hitPosition), hitNormal);
						//totalColorResponse += secondaryHitMat.albedo;
					}
				}
			}
			renderImage(x, y) = totalColorResponse / float(1);
		}
	}
}

Vec3f RayTracer::randomDirectionUnitSphere()
{
	/*Vec3f point;*/
	//do
	//{
	//	point[0] = 2.0f * (double)rand() / RAND_MAX - 1.f;
	//	point[1] = 2.0f * (double)rand() / RAND_MAX - 1.f;
	//	point[2] = 2.0f * (double)rand() / RAND_MAX - 1.f;
	//} while (point.squaredLength() > 1.0f);
	//return point;
	float rd = M_PI * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	float rd2 = M_PI * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	float x = cos(rd);
	float y = sin(rd2);
	float z = sqrt(1 - x * x - y * y);
	return Vec3f(x, y, z);
}

Vec3f RayTracer::pathTrace(const Ray& ray, size_t current, const Scene& scene, size_t maxBounces = 10)
{
	if (current > maxBounces) return Vec3f(0,0,0);		
	Vec3f pathResponse(0,0,0);
	Vec3f intersectionPos, intersectionNormal; size_t meshIndex;
	if (rayTraceBVH(ray, scene, intersectionPos, intersectionNormal, meshIndex))
	{
		//direct lighting contribution
		Material hitMat = scene.meshes()[meshIndex].material();
		Vec3f directLight = directLightingShade(intersectionPos, intersectionNormal, hitMat, scene);
		current++;
		//Uniform random sample of the directions
		Vec3f direction = normalize(intersectionNormal + randomDirectionUnitSphere());
		Ray segment = Ray(intersectionPos, direction);
		//Get recursively contribution of next bounce (indirect lighting contribution)
		Vec3f indirectLightColor = pathTrace(segment, current, scene, maxBounces);
		Vec3f indirectLight = indirectLightColor*hitMat.evaluateDiffuseColorResponse(intersectionNormal, normalize(segment.m_direction));		
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
		const Mesh& sceneMesh = scene.meshes()[meshIndex];
		const std::vector<Vec3f>& vertices = sceneMesh.vertices();
		const std::vector<Vec3f>& normals = sceneMesh.normals();
		//return intersection position and normal by interpoling using barycentric coordinates
		intersectionPos = sceneMesh.interpPos(hitRecord.barCoord, triangleIndices);
		intersectionNormal = sceneMesh.interpNorm(hitRecord.barCoord, triangleIndices);
	}
	return intersectFound;
}

Vec3f RayTracer::directLightingShade(const Vec3f& position, const Vec3f& normal, const Material& mat, const Scene& scene)
{
	const std::vector<lightPtr>& lights = scene.lightSources();	
	Vec3f lightContributionSum;
	for (int i = 0; i < lights.size(); ++i)
	{
		// Shadow Test
		Vec3f lightPos = lights[i]->getPosition();
		Ray shadowRay = Ray(position, normalize(lightPos - position));
		Vec3f shadowInterPos, shadowInterNormal; size_t shadowMeshIndex;
		if (RayTracer::rayTraceBVH(shadowRay, scene, shadowInterPos, shadowInterNormal, shadowMeshIndex))
		{
			continue;
		}
		lightContributionSum += lights[i]->colorResponse()*mat.evaluateDiffuseColorResponse(normalize(normal), normalize(lightPos - position));
	}
	return lightContributionSum;
}
