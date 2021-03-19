#pragma once

#include<vector>
#include<cmath>
#include "Vec3.h"
#include "scene.h"
#include "surfel.h"
#include "BVHnode.h"

class PointCloud {
private:
	std::vector<Surfel> m_surfels;
	float m_samplingRate = 1.f;
	BVHnode::BVHptr m_BVHroot;
public:
	inline PointCloud(float samplingRate) : m_samplingRate(samplingRate) {};
	//using blue noise sampling
	inline void computePointCloud(const Scene& scene)
	{
		const std::vector<Mesh>& meshes = scene.meshes();
		for (int i = 0; i < meshes.size(); i++)
		{			
			for (int j = 0; j < meshes[i].indices().size(); j++)
			{
				const Vec3i& triangleIndices = meshes[i].indices()[j];
				const Vec3<Vec3f>& triangle = meshes[i].triangle(triangleIndices);
				float S = (cross(triangle[1] - triangle[0], triangle[2] - triangle[0])).length() / 2.f;
				int Nsamples = int(m_samplingRate * S);				
				for (int k = 0; k < Nsamples; k++)
				{
					float r1 = -0.5f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					float r2 = -0.5f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					float r3 = -0.5f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					Vec3f sampleBarCoord = Vec3f(0.5f) + Vec3f(r1, r2, r3);
					Vec3f samplePos = meshes[i].interpPos(sampleBarCoord, triangleIndices);					
					Vec3f sampleNorm = meshes[i].interpNorm(sampleBarCoord, triangleIndices);
					Vec3f sampleTan = normalize(cross(sampleNorm, triangle[1] - triangle[0]));
					Vec3f sampleColor = RayTracer::shade(samplePos, sampleNorm, meshes[i].material(), scene);
					float sampleRad = sqrt(1.f / m_samplingRate * M_PI);
					Surfel sampleSurfel = Surfel(samplePos, sampleNorm, sampleTan, sampleColor, sampleRad);
					m_surfels.push_back(sampleSurfel);
				}
			}
		}
	}


	inline void computeBVH() { m_BVHroot = BVHnode::BVHptr(new BVHnode(m_surfels)); };
	//generate tris from surfels to debug the generated PointCloud
	inline void triangleFromSurfels(std::vector<Vec3<Vec3f>>& positions, std::vector<Vec3<Vec3f>>& normals, std::vector<Vec3f> colors)
	{		
		for (int i = 0; i < m_surfels.size(); i++)
		{
			float r = m_surfels[i].radius;
			Vec3f p = m_surfels[i].position;
			Vec3f n = m_surfels[i].normal;
			Vec3f u = m_surfels[i].tangent;
			Vec3f v = normalize(cross(n, u));
			Vec3<Vec3f> triangle1 = Vec3<Vec3f>(p + r * u, p + r * v, p - r * v);
			Vec3<Vec3f> triangle2 = Vec3<Vec3f>(p - r * u, p + r * v, p - r * v);
			positions.push_back(triangle1);
			normals.push_back(n);
			positions.push_back(triangle2);
			normals.push_back(n);
			colors.push_back(m_surfels[i].color);
		}
	}
	//accessors
	inline std::vector<Surfel> surfels() { return m_surfels; }
	inline const std::vector<Surfel> surfels() const { return m_surfels; }
	inline float samplingRate() { return m_samplingRate; }
	inline const float samplingRate() const { return m_samplingRate; }
	inline const BVHnode::BVHptr BVHroot() const { return m_BVHroot; }
};
