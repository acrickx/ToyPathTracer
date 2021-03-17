#include "rayTracer.h"


//called to render image from scene
Image RayTracer::render(Scene scene, Image& renderImage, size_t rayPerPixel = 8)
{
	std::vector<Mesh> sceneMeshes = scene.meshes();
	Camera renderCam = scene.camera();
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
			bool intersection = false;
			for (int k = 0; k < rayPerPixel; k++)
			{
				Ray scatteredRay(renderCam.getPosition(), normalize(renderCam.getImageCoordinate(float(i) / width + ((std::rand() % 100) / (float)100) * (1 / (float)width), 1.f - (((float)j / height) + ((std::rand() % 100) / (float)100) * (1 / (float)height)))));
				Vec3f intersectionPos, intersectionNormal; size_t meshIndex; size_t triangleIndex;
				intersectionFound = rayTrace(scatteredRay, scene, intersectionPos, intersectionNormal, meshIndex, triangleIndex);
				if (intersectionFound)
				{					
					if (!intersection) intersection = true;
					totalColorResponse += shade(sceneMeshes[meshIndex], intersectionPos, intersectionNormal, scene) / (float)rayPerPixel;										
				}
			}
			if(intersection) renderImage(i, j) = totalColorResponse;
		}
	}
	return renderImage;
}

//actually raytrace the scene with a given ray
bool RayTracer::rayTrace(Ray ray, Scene scene, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& meshIndex, size_t& triangleIndex)
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
					triangleIndex = k;
					//return intersection position and normal by interpoling using barycentric coordinates
					intersectionPos = sceneMesh.interpPos(barCoord, triangleIndices);
					intersectionNormal = sceneMesh.interpNorm(barCoord,triangleIndices);
				}
			}
		}
	}
	return intersectFound;
}

//called to render image from positions and normals directly (debug point cloud)
Image RayTracer::render(const std::vector<Vec3<Vec3f>>& positions, const std::vector<Vec3<Vec3f>>& normals, const Camera& renderCam, const std::vector<lightPtr>& lights, const Material& material, Image& renderImage)
{	
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
			bool intersection = false;
			Ray ray(renderCam.getPosition(), renderCam.getImageCoordinate(float(i) / width, 1.f - (float)j / height));
			Vec3f intersectionPos, intersectionNormal; size_t triangleIndex;
			intersectionFound = rayTrace(ray, positions, normals, intersectionPos, intersectionNormal, triangleIndex);
			if (intersectionFound)
			{
				renderImage(i,j) = material.evaluateColorResponse(intersectionPos, intersectionNormal, lights[0], renderCam);
			}
		}
	}
	return renderImage;
}

//raytrace from vector of triangles positions directly
bool RayTracer::rayTrace(Ray ray, const std::vector<Vec3<Vec3f>>& positions, const std::vector<Vec3<Vec3f>>& normals, Vec3f& intersectionPos, Vec3f& intersectionNormal, size_t& triangleIndex)
{
	float zmax = std::numeric_limits<float>::max();
	bool intersectFound = false;

	for (int k = 0; k < positions.size(); k++)
	{
		//get triangle info
		Vec3<Vec3f> trianglePositions = positions[k];
		Vec3<Vec3f> triangleNormals = normals[k];
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
				triangleIndex = k;
				//return intersection position and normal by interpoling using barycentric coordinates
				intersectionPos = barCoord[2] * trianglePositions[0] + barCoord[0]*trianglePositions[1] + barCoord[1]*trianglePositions[2];
				intersectionNormal = barCoord[2] * triangleNormals[0] + barCoord[0] * triangleNormals[1] + barCoord[1] * triangleNormals[2];
			}
		}
	}

	return intersectFound;
}


bool RayTracer::rayTrace(Ray ray, Scene scene)
{
	std::vector<Mesh> sceneMeshes = scene.meshes();
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
			Vec3f barCoord; float parT = 0;
			if (ray.testTriangleIntersection(trianglePositions, barCoord, parT))
			{
				intersectFound = true;
			}
		}
	}
	return intersectFound;
}

Vec3f RayTracer::shade(Mesh mesh, Vec3f position, Vec3f normal, const Scene& scene)
{
	std::vector<lightPtr> lights = scene.lightSources();
	Vec3f totalColor = mesh.material().albedo;
	for (int i = 0; i < lights.size(); i++)
	{
		Ray shadowRay = Ray(position + normal * 0.001f, scene.lightSources()[i]->getPosition());
		Vec3f shadowInterPos, shadowInterNormal; size_t shadowMeshIndex; size_t shadowTriangleIndex;
		bool shadowIntersect = rayTrace(shadowRay, scene, shadowInterPos, shadowInterNormal, shadowMeshIndex, shadowTriangleIndex);
		if (!shadowIntersect || (position - shadowInterPos).length() <= 0.0001f)
		{
			totalColor += mesh.material().evaluateColorResponse(position, normal, lights[i], scene.camera());			
		}
	}
	return totalColor;
}
