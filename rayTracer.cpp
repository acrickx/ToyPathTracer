#include "rayTracer.h"

//called to render image from scene
void RayTracer::render(const Scene& scene, Image& renderImage, size_t rayPerPixel = 8, size_t bounces = 0)
{
	const std::vector<Mesh>& sceneMeshes = scene.meshes();	
	const Camera& renderCam = scene.camera();
	const Vec3f& camPos = renderCam.getPosition();

	// Fill background of the image with arbitrary color
	renderImage.fillBackground(Vec3f(0.5, 0.5, 0.5), Vec3f(0.1f, 0.1f, 0.1f));
	int width = renderImage.getWidth(); 
	int height = renderImage.getHeight();

	#pragma omp parallel for schedule(dynamic, 1)  
	for (int y = 0; y < height; y++)
	{
		// Display progress in percentage
		fprintf(stderr, "\rRendering (%i samples): %.2f%% ",rayPerPixel, (double)y / height * 100);

		for (int x = 0; x < width; x++)
		{
			Vec3f totalColorResponse(0,0,0);			
			for (int k = 0; k < rayPerPixel; k++)
			{
				// Pixel sampling
				float rd_width = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				float rd_height = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				Ray scatteredRay = renderCam.rayAt((float(x) + rd_width)/ width, 1.f - (float(y) + rd_height)/height);

				// Ray tracing 
				Vec3f hitPosition, hitNormal;
				size_t meshIndex;
				if (rayTraceBVH(scatteredRay, scene, hitPosition, hitNormal, meshIndex))
				{

					// Direct Illumination
					MaterialPtr hitMat = scene.meshes()[meshIndex].material();
					if (hitMat->type == Material::EMISSIVE)
					{
						totalColorResponse += hitMat->colorResponse(hitPosition, hitNormal, Vec3f(0.f), Vec3f(0.f));
						continue;
					}

					Vec3f direct = evalDirect(hitPosition, hitNormal, hitMat, scene);	

					// Indirect Illumination
					// Vec3f indirect = tracePath(hitPosition, hitNormal, hitMat, 1, scene);

					// Final gathering
					totalColorResponse += direct;
				}
			}
			renderImage(x, y) = totalColorResponse / float(rayPerPixel);
		}
	}
}

Vec3f RayTracer::tracePath(const Vec3f& origin, const Vec3f& normal, MaterialPtr material, size_t nBounces, const Scene& scene)
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
			MaterialPtr hitMat = scene.meshes()[hitMesh].material();
			Vec3f hitDirect = evalDirect(hitPosition, hitNormal, hitMat, scene);
			indirect += hitDirect / pdf * material->colorResponse(origin, normal, normalize(hitPosition - origin), origin);
		}
	}
	return indirect / 5.0f;
}

// Raytrace the scene with a given ray (loop over all triangles)
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
			// Get triangle info
			const Vec3i& triangleIndices = sceneMesh.indices()[k]; const Vec3<Vec3f>& trianglePositions = sceneMesh.triangle(triangleIndices);

			// Test intersection
			Vec3f barCoord; float parT;
			if (ray.testTriangleIntersection(trianglePositions, barCoord, parT))
			{
				// z buffer test				
				if (parT >0 && zmax > parT)
				{
					intersectFound = true;
					zmax = parT;
					meshIndex = l;		

					// Return intersection position and normal by interpoling using barycentric coordinates
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
	// Init
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

		// Return intersection position and normal by interpoling using barycentric coordinates
		intersectionPos = sceneMesh.interpPos(hitRecord.barCoord, triangleIndices);
		intersectionNormal = sceneMesh.interpNorm(hitRecord.barCoord, triangleIndices);
	}

	return intersectFound;
}

Vec3f sampleMeshUniformly(const Mesh& mesh, Vec3f& normal)
{
	if (mesh.indices().size() == 0) return Vec3f{};

	size_t rdIndex = size_t(GeometryHelper::random(0, mesh.indices().size() - 1));
	auto& rdTri = mesh.triangle(mesh.indices()[rdIndex]);
	normal = GeometryHelper::computeTriangleNormal(rdTri);
	return GeometryHelper::sampleTriangleUniformly(rdTri);
}


Vec3f RayTracer::evalDirect(const Vec3f& position, const Vec3f& normal, MaterialPtr mat, const Scene& scene)
{
	// Init
	const std::vector<lightPtr>& lights = scene.lightSources();	
	const size_t kAnalyticalSamples = 0;
	const size_t kEmissiveSamples = 1;
	Vec3f analytical{}, emissive{};

	// Analytical lights
	for (int i = 0; i < lights.size(); ++i)
	{
		// Shadow Test
		Vec3f lightPos = lights[i]->getPosition();
		Vec3f direction = normalize(lightPos - position);
		Ray shadowRay = Ray(position, direction);
		Vec3f shadowInterPos, shadowInterNormal; size_t shadowMeshIndex;

		// If occluded
		if (RayTracer::rayTraceBVH(shadowRay, scene, shadowInterPos, shadowInterNormal, shadowMeshIndex))
		{
			continue;
		}

		// Else shade based on light properties
		analytical += lights[i]->colorResponse() * M_1_PI * mat->colorResponse(position, normal, direction, scene.camera().getPosition());
	}

	// Emissive Triangles
	for (int i = 0; i < scene.emissiveMeshes().size(); ++i)
	{
		for (int k = 0; k < kEmissiveSamples; ++k)
		{
			size_t index = scene.emissiveMeshes()[i];
			Mesh emissiveMesh = scene.meshes()[index];

			// Safety net
			if (emissiveMesh.material()->type != Material::EMISSIVE)
			{
				std::cerr << "Should not be here" << std::endl;
				continue;
			}

			// Sample Mesh and test visibility
			Vec3f sampledNorm;
			Vec3f sampledPos = sampleMeshUniformly(emissiveMesh, sampledNorm);

			Vec3f direction = normalize(sampledPos - position);
			Ray shadowRay = Ray(position - 0.0001f* direction, normalize(sampledPos - position));
			Vec3f shadowInterPos, shadowInterNormal; size_t shadowMeshIndex;

			// If occluded
			//if (RayTracer::rayTraceBVH(shadowRay, scene, shadowInterPos, shadowInterNormal, shadowMeshIndex))
			//{
			//	if(shadowMeshIndex != index)
			//	continue;
			//}

			// Shade using mesh light
			emissive += emissiveMesh.material()->colorResponse(sampledPos, normal, Vec3f(0.0f), Vec3f(0.0f)) * mat->colorResponse(position, normal, direction, scene.camera().getPosition());
		}
	}

	return  emissive;
}
