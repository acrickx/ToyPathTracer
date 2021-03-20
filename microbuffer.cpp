#pragma once

#include"microbuffer.h"

//Coordinate system changes and pixel/direction mapping
	bool MicroBuffer::positionToPixel(Vec3f pos, int& i, int& j) 
	{
		i = (size_t)(dot(pos - m_bottomLeftCorner, m_horizontal)*m_width/2.f);
		j = (size_t)(dot(pos - m_bottomLeftCorner, m_vertical)*m_height/2.f);
		if (i > m_width || i<0 || j>m_height || j < 0) { std::cerr << "error : given pos is outside the image bounds - coord found : (" << i << "," << j << ")\n"; return false; }
		return true;
	}

	Vec3f MicroBuffer::pixelToPostion(size_t i, size_t j)
	{
		if (i > m_width || i<0 || j>m_height || j < 0) { std::cerr << "error : given pos is outside the image bounds - coord : (" << i << "," << j << ")\n";}
		return m_bottomLeftCorner + m_horizontal * 2 * i / m_width + m_vertical * 2 * j / m_height;
	}

	Vec3f MicroBuffer::pixelToDirection(size_t i, size_t j)
	{
		float size = m_width; // = m_height;
		Vec3f absPixelPos = pixelToPostion(i, j);
		Vec3f relPixelPos = absPixelPos-m_gatheringPos;
		Vec3f localPixelPos = Vec3f(dot(relPixelPos, m_horizontal), dot(relPixelPos, m_vertical), absPixelPos[2]);
		Vec3f hemisphericalPoint = absPixelPos + sqrt(1 - relPixelPos[0] * relPixelPos[0] - relPixelPos[1] * relPixelPos[1]) * m_gatheringNormal;
		Vec3f direction = normalize(hemisphericalPoint - m_gatheringPos);
		return direction;
	}

	bool MicroBuffer::directionToPixel(Vec3f direction, int& i, int& j)
	{ 
		Vec3f normDirection = normalize(direction);
		Vec3f normalizedPixelPos = m_gatheringPos + normDirection - dot(normDirection, m_gatheringNormal)*m_gatheringNormal ;		
		return positionToPixel(normalizedPixelPos,i,j);
	}

	float MicroBuffer::solidAngle(size_t i, size_t j)
	{
		Vec3f absPixelPos = pixelToPostion(i, j);
		Vec3f relPixelPos = absPixelPos - m_gatheringPos;
		Vec3f localPixelPos = Vec3f(dot(relPixelPos, m_horizontal), dot(relPixelPos, m_vertical), absPixelPos[2]);
		float num = (localPixelPos[0] * localPixelPos[0] + localPixelPos[1] * localPixelPos[1]);
		float denum = (1-localPixelPos[0] * localPixelPos[0] + localPixelPos[1] * localPixelPos[1]);
		//std::cout << "num : " << num << std::endl;
		//std::cout << "denum : " << denum << std::endl;
		float solidAngle = sqrt(num / denum);
		return solidAngle;
	}

	//BVH traversal
	void MicroBuffer::fillMicroBuffer(BVHnode::BVHptr node)
	{		
		Vec3f direction = (node->position() - m_gatheringPos);
		float distance = direction.length();
		bool hasChildren = (node->surfels().size() > 1);
		float BVHsolidAngle = 0;
		if (hasChildren) BVHsolidAngle = M_PI * (node->radius() * node->radius()) / (distance * distance);
		else BVHsolidAngle = M_PI * (node->surfels()[0].radius * node->surfels()[0].radius) / (distance * distance);
		int indexI, indexJ;
		if (directionToPixel(direction, indexI, indexJ))
		{
			//std::cout << "BVH solid angle : " << BVHsolidAngle << std::endl;
			//std::cout << "Pixel solid angle : " << solidAngle(indexI, indexJ) << std::endl;
			if (solidAngle(indexI, indexJ) < BVHsolidAngle)
			{
				if (hasChildren)
				{
					fillMicroBuffer(node->left());
					fillMicroBuffer(node->right());
				}
				else //update post traversal list with too big leaf nodes for rasterization
				{
					m_postTraversalList.push_back(node);					
				}
			}
			else
			{
				if (depth(indexI, indexJ) > distance)
				{
					setDepthValue(indexI, indexJ, distance);
					setIndex(indexI, indexJ, node);
					setColorValue(indexI, indexJ, node->getColor() * BVHsolidAngle);
				}
			}
		}
	}

	//ray cast leave node for precise rasterization
	void MicroBuffer::postTraversalRayCasting()
	{
		//std::cout << "postTraversalList size : " << m_postTraversalList.size() << std::endl;
		for (int j = 0; j < m_height; j++)
		{
			for (int i = 0; i < m_width; i++)
			{
				Ray ray(m_gatheringPos, pixelToDirection(i, j));					
				for (int k = 0; k < m_postTraversalList.size(); k++)
				{
					BVHnode::BVHptr node = m_postTraversalList[k];
					Vec3f intersectionPos; float parT=0;
					Surfel surfel = node->surfels()[0];
					if (ray.testDiscIntersection(surfel.position, surfel.normal, surfel.radius, intersectionPos, parT))
					{
						if (parT < depth(i, j))
						{
							setDepthValue(i, j, parT);
							setIndex(i, j, m_postTraversalList[k]);
							setColorValue(i, j, surfel.color*M_PI*surfel.radius*surfel.radius/parT);
						}
					}
				}
			}
		}
	}

	//convolve microbuffer with BRDF
	Vec3f MicroBuffer::convolveBRDF(Material mat, const Scene& scene)
	{
		std::vector<lightPtr> lights = scene.lightSources();
		Vec3f totalColorResponse;
		for (int i = 0; i < m_width; i++)
		{
			for (int j = 0; j < m_height; j++)
			{
				for (int k = 0; k < lights.size(); k++)
				{
					totalColorResponse += color(i, j) * mat.evaluateColorResponse(m_gatheringPos, m_gatheringNormal, lights[k], scene.camera());
				}
			}
		}
		return totalColorResponse;
	}
