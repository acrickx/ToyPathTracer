#pragma once

#include<vector>
#include<cmath>
#include "Vec3.h"
#include "scene.h"
#include "surfel.h"
#include "BSHnode.h"
#include <ctime>

class PointCloud {
private:
	std::vector<Surfel> m_surfels;
	float m_samplingRate = 1.f;
	BSHnode::BSHptr m_BSHroot;
public:
	inline PointCloud(float samplingRate) : m_samplingRate(samplingRate) {};
	inline PointCloud(std::vector<Surfel> surfels) : m_surfels(surfels) {};
	//using blue noise sampling
	inline void computePointCloud(const Scene& scene)
	{
		float sampleRad = 1 / (sqrt(m_samplingRate));		
		const std::vector<Mesh>& meshes = scene.meshes();
		for (int i = 0; i < meshes.size(); i++)
		{			
			for (int j = 0; j < meshes[i].indices().size(); j++)
			{				
				const Vec3i& triangleIndices = meshes[i].indices()[j];
				const Vec3<Vec3f>& triangle = meshes[i].triangle(triangleIndices);
				float S = (cross(triangle[1] - triangle[0], triangle[2] - triangle[0])).length() / 2.f;
				int Nsamples = int(m_samplingRate * S);		
				if (Nsamples > 0)
				{
					std::vector<Vec3f> sampleP, sampleN;
					linearSubdivision(meshes[i], triangleIndices, Nsamples, sampleP, sampleN);
					for (int k = 0; k < sampleP.size(); k++)
					{
						//compute surfel attributes for best candidate and add to the list	
						Vec3f samplePos = sampleP[k];
						Vec3f sampleNorm = sampleN[k];
						Vec3f sampleColor = RayTracer::directLightingShade(samplePos, sampleNorm, meshes[i].material(), scene);					
						Surfel sampleSurfel = Surfel(samplePos, sampleNorm, sampleColor, sampleRad);
						m_surfels.push_back(sampleSurfel);
					}					
				}
			}
		}
	}

	//Subdivision linéaire
	void linearSubdivision(const Mesh& mesh, const Vec3i& triangleIndices, int Nsamples, std::vector<Vec3f>& samplePositions, std::vector<Vec3f>& sampleNorm)
	{			
		float u = 0, v = 0;		
		int N = (sqrt(2*Nsamples));
		for (int i = 0; i < N; i++)
		{
			u = i /(float)N;
			for (int j = 0; j < N; j++)
			{
				v = j /(float)N;
				float w = 1 - u - v;
				if (w >=0 || abs(w) <= 1e-5)
				{
					Vec3f barCoord(u, v, 1.f - u - v);
					Vec3f sampleP = mesh.interpPos(barCoord, triangleIndices);
					Vec3f sampleN = mesh.interpNorm(barCoord, triangleIndices);
					samplePositions.push_back(sampleP);
					sampleNorm.push_back(sampleN);
				}
			}
		}			
	}

	//Mitchell's best candidate algorithm
	void BestCandidateSampling(const Mesh& mesh, const Vec3i& triangleIndices, int Nsamples, std::vector<Vec3f>& samplePositions, std::vector<Vec3f>& sampleNorm, float sampleRad)
	{				
		float minSquaredRad = sampleRad * sampleRad;
		//First random point near center
		float r1 = -0.1f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2f));
		float r2 = -0.1f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2f));
		float r3 = -0.1f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2f));
		Vec3f sampleBarCoord = Vec3f(r1, r2, r3);
		Vec3f samplePos = mesh.interpPos(sampleBarCoord, triangleIndices);		
		samplePositions.push_back(samplePos);
		sampleNorm.push_back(mesh.interpNorm(sampleBarCoord, triangleIndices));
		//best candidate algorithm to generate blue noise sampling
		for (int k = 0; k < Nsamples - 1; k++)
		{
			Vec3f maxPos, maxBarCoord;
			float maxDist = 0.f;
			//find the best point (maximizing distance with other close samples) among 10 candidates
			for (int l = 0; l < 20; l++)
			{							
				float rd1 = -0.5f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				float rd2 = -0.5f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				float rd3 = -0.5f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				Vec3f barCoord = Vec3f(0.5f) + Vec3f(rd1, rd2, rd3);
				Vec3f pos = mesh.interpPos(barCoord, triangleIndices);
				float squaredDist = 0; size_t nNeighbors = 0;
				for (int m = 0; m < samplePositions.size(); m++)
				{
					float sqD = (pos - samplePositions[m]).squaredLength();
					if (sqD < 100 * minSquaredRad)
					{
						squaredDist += sqD;
						nNeighbors++;
					}
				}
				if (squaredDist/(float)nNeighbors > maxDist)
				{
					maxDist = squaredDist;
					maxPos = pos;
					maxBarCoord = barCoord;
				}
			}			
			samplePositions.push_back(maxPos);
			sampleNorm.push_back(mesh.interpNorm(maxBarCoord, triangleIndices));
		}
	}

	//Pure random sampling
	void UniformSampling(const Mesh& mesh, const Vec3i& triangleIndices, int Nsamples, std::vector<Vec3f>& samplePositions, std::vector<Vec3f>& sampleNorm)
	{		
		for (int k = 0; k < Nsamples - 1; k++)
		{
			float r1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float r3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			Vec3f sampleBarCoord = Vec3f(r1, r2, r3);
			Vec3f samplePos = mesh.interpPos(sampleBarCoord, triangleIndices);
			samplePositions.push_back(samplePos);		
			sampleNorm.push_back(mesh.interpNorm(sampleBarCoord, triangleIndices));
		}
	}


	//Simple blue-noise sampling
	void BlueNoiseSampling(const Mesh& mesh, const Vec3i& triangleIndices, int Nsamples, std::vector<Vec3f>& samplePositions, std::vector<Vec3f>& sampleNorm, float sampleRad)
	{
		std::vector<Vec3f> samplePositionsTmp;
		std::vector<Vec3f> sampleNormTmp;
		for (int k = 0; k < 10*Nsamples - 1; k++)
		{
			float r1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float r3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			Vec3f sampleBarCoord = Vec3f(r1, r2, r3);
			Vec3f samplePos = mesh.interpPos(sampleBarCoord, triangleIndices);
			samplePositionsTmp.push_back(samplePos);
			sampleNormTmp.push_back(mesh.interpNorm(sampleBarCoord, triangleIndices));
		}
		int iter = 100000;
		float maxRadiusSquare = sampleRad*sampleRad;
		while (iter > 0 && samplePositionsTmp.size() > Nsamples)
		{			
			int rd = rand()%samplePositionsTmp.size();
			Vec3f rdSample = samplePositionsTmp[rd];
			//samplePositions.push_back(samplePositionsTmp[rd]);
			//sampleNorm.push_back(sampleNormTmp[rd]);
			for (int i = 0; i < samplePositionsTmp.size(); i++)
			{
				if (rd != i && (samplePositionsTmp[i] - rdSample).squaredLength() < maxRadiusSquare)
				{
					//samplePositions.push_back(samplePositionsTmp[i]);
					//sampleNorm.push_back(sampleNormTmp[i]);
					samplePositionsTmp.erase(samplePositionsTmp.begin() + i);
					sampleNormTmp.erase(sampleNormTmp.begin() + i);
				}
			}
			iter--;
		}
		samplePositions = samplePositionsTmp;
		sampleNorm = sampleNormTmp;
	}
	//compute BVH
	inline void computeBSH() { m_BSHroot = BSHnode::BSHptr(new BSHnode(m_surfels)); };
	//accessors
	inline std::vector<Surfel> surfels() { return m_surfels; }
	inline const std::vector<Surfel> surfels() const { return m_surfels; }
	inline float samplingRate() { return m_samplingRate; }
	inline const float samplingRate() const { return m_samplingRate; }
	inline const BSHnode::BSHptr BVHroot() const { return m_BSHroot; }
};
