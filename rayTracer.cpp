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
			for (int k = 0; k < rayPerPixel; k++)
			{
				float rd_width = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				float rd_height = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				Ray scatteredRay = renderCam.rayAt((float(x) + rd_width)/ width, 1.f - (float(y) + rd_height)/height);
				//Ray tracing 
				Vec3f hitPosition, hitNormal;
				size_t meshIndex;
				if (rayTraceBVH(scatteredRay, scene, hitPosition, hitNormal, meshIndex))
				{
					// Direct Illumination
					Material hitMat = scene.meshes()[meshIndex].material();
					Vec3f direct = evalDirect(hitPosition, hitNormal, hitMat, scene);	
					// Indirect Illumination
					Vec3f indirect = tracePath(hitPosition, hitNormal, hitMat, 1, scene);
					totalColorResponse += (direct + indirect);
				}
			}
			renderImage(x, y) = totalColorResponse / float(rayPerPixel);
		}
	}
}

Vec3f RayTracer::tracePath(const Vec3f& origin, const Vec3f& normal, const Material& material, size_t nBounces, const Scene& scene)
{
	Vec3f indirect{};
	for (int i = 0; i < 5; ++i)
	{
		float rdX = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float rdY = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float pdf;
		Vec3f randomDirection = GeometryHelper::sampleCosineHemisphereConcentric(rdX, rdY, normal, pdf);
		Ray reflectionRay = Ray(origin + 0.01f * normal, normalize(randomDirection));
		Vec3f hitPosition, hitNormal; size_t hitMesh;
		if (rayTraceBVH(reflectionRay, scene, hitPosition, hitNormal, hitMesh))
		{
			Material hitMat = scene.meshes()[hitMesh].material();
			Vec3f hitDirect = evalDirect(hitPosition, hitNormal, hitMat, scene);
			indirect +=  hitDirect / pdf * material.evaluateDiffuseColorResponse(normal, normalize(hitPosition - origin));
		}
	}
	return indirect / 5.0f;
}

Vec3f RayTracer::randomDirectionUnitSphere()
{
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
		Vec3f directLight = evalDirect(intersectionPos, intersectionNormal, hitMat, scene);
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

Vec3f RayTracer::evalDirect(const Vec3f& position, const Vec3f& normal, const Material& mat, const Scene& scene)
{
	Vec3f direct{};
	const std::vector<lightPtr>& lights = scene.lightSources();	

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
		// direct += lights[i]->colorResponse() * mat.evaluateDiffuseColorResponse(normalize(normal), normalize(lightPos - position));
		direct += lights[i]->colorResponse() * M_1_PI * mat.albedo * dot(normal, normalize(lightPos - position));
	}
	return direct;
}
