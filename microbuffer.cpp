#pragma once

#include"microbuffer.h"

//Coordinate system changes and pixel/direction mapping
	bool MicroBuffer::positionToPixel(Vec3f pos, int& i, int& j) 
	{
		std::cout << "m_horizontal : " << m_horizontal << std::endl;
		std::cout << "m_vertical : " << m_vertical << std::endl;
		i = (size_t)(dot(pos - m_bottomLeftCorner, m_horizontal)*m_width);
		j = (size_t)(dot(pos - m_bottomLeftCorner, m_vertical)*m_height);
		if (i > m_width || i<0 || j>m_height || j < 0) { std::cerr << "error : given pos is outside the image bounds - coord found : (" << i << "," << j << ")\n"; return false; }
		return true;
	}

	Vec3f MicroBuffer::pixelToPostion(size_t i, size_t j)
	{
		if (i > m_width || i<0 || j>m_height || j < 0) { std::cerr << "error : given pos is outside the image bounds - coord : (" << i << "," << j << ")\n";}
		return m_bottomLeftCorner + (float)i * m_horizontal + (float)j * m_vertical;
	}

	Vec3f MicroBuffer::pixelToDirection(size_t i, size_t j)
	{
		float size = m_width; // = m_height;
		float x = (0.5f - 2 * i / size) * (0.5f - 2 * i / size);
		float y = (0.5f - 2 * j / size) * (0.5f - 2 * j / size);
		float z = sqrt(1 - x*x - y*y);			
		return pixelToPostion(i, j) + z * size * m_gatheringNormal;
	}

	bool MicroBuffer::directionToPixel(Vec3f direction, int& i, int& j)
	{ 
		Vec3f normDirection = normalize(direction);
		Vec3f normalizedPixelPos = m_gatheringPos + normDirection - dot(-normDirection, -m_gatheringNormal)*m_gatheringNormal ;
		return positionToPixel(normalizedPixelPos,i,j);
	}

	float MicroBuffer::solidAngle(size_t i, size_t j)
	{
		float size = m_width; // = m_height;
		float x = (0.5f - 2 * i / size) * (0.5f - 2 * i / size);
		float y = (0.5f - 2 * j / size) * (0.5f - 2 * j / size);
		float factor = 1 / (x * x + y * y - 1);
		return sqrt((x * x + y * y) / pow(y * y + x * x - 1, 2));
	}

	//BVH traversal
	void MicroBuffer::fillMicroBuffer(BVHnode::BVHptr node)
	{		
		Vec3f direction = (node->position() - m_gatheringPos);
		float distance = direction.length();		
		float BVHsolidAngle = M_PI *(node->radius()*node->radius())/(distance * distance);
		int indexI, indexJ;
		if (directionToPixel(direction, indexI, indexJ))
		{
			if (solidAngle(indexI, indexJ) < BVHsolidAngle)
			{
				if (node->left() != nullptr)
				{
					fillMicroBuffer(node->left());
				}
				if (node->right() != nullptr)
				{
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
				}
			}
		}
	}

	void MicroBuffer::postTraversalRayCasting()
	{
		std::cout << "postTraversalList size : " << m_postTraversalList.size() << std::endl;
		for (int j = 0; j < m_height; j++)
		{
			for (int i = 0; i < m_width; i++)
			{
				Ray ray(m_gatheringPos, pixelToDirection(i, j));					
				for (int k = 0; k < m_postTraversalList.size(); k++)
				{
					BVHnode::BVHptr node = m_postTraversalList[k];
					Vec3f intersectionPos; float parT;
					if (ray.testDiscIntersection(node->position(), node->normal(), node->radius(), intersectionPos, parT))
					{
						if (parT < depth(i, j))
						{
							setIndex(i, j, m_postTraversalList[k]);
						}
					}
				}
			}
		}
	}