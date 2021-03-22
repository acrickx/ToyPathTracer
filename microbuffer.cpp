#pragma once

#include"microbuffer.h"

//Coordinate system changes and pixel/direction mapping
	bool MicroBuffer::positionToPixel(Vec3f pos, int& i, int& j) 
	{
		i = (size_t)(dot(pos - m_bottomLeftCorner, m_horizontal)*(m_width-1)/2.f);
		j = (size_t)(dot(pos - m_bottomLeftCorner, m_vertical)*(m_height-1)/2.f);
		if (i > m_width-1 || i<0|| j>m_height-1 || j < 0) { std::cerr << "error : given pos is outside the image bounds - coord found : (" << i << "," << j << ")\n"; return false; }
		return true;
	}

	Vec3f MicroBuffer::pixelToPostion(size_t i, size_t j)
	{
		if (i > m_width || i<0 || j>m_height || j < 0) { std::cerr << "error : given pos is outside the image bounds - coord : (" << i << "," << j << ")\n";}
		return m_bottomLeftCorner + m_horizontal * 2 * i / (m_width-1) + m_vertical * 2 * j / (m_height-1);
	}

	Vec3f MicroBuffer::pixelToDirection(size_t i, size_t j)
	{
		float size = m_width; // = m_height;
		Vec3f absPixelPos = pixelToPostion(i, j);
		Vec3f relPixelPos = absPixelPos-m_gatheringPos;
		float squaredLength = relPixelPos[0] * relPixelPos[0] + relPixelPos[1] * relPixelPos[1];
		Vec3f localPixelPos = Vec3f(dot(relPixelPos, m_horizontal), dot(relPixelPos, m_vertical), absPixelPos[2]);
		Vec3f hemisphericalPoint = absPixelPos + sqrt(1 - squaredLength) * m_gatheringNormal;
		Vec3f direction = normalize(hemisphericalPoint - m_gatheringPos);
		return direction;
		if (squaredLength >= 1)
		{
			return normalize(dot(relPixelPos,m_horizontal)*m_horizontal + dot(relPixelPos,m_vertical)*m_vertical + 0.1f*m_gatheringNormal);
		}
		else
		{
			Vec3f localPixelPos = Vec3f(dot(relPixelPos, m_horizontal), dot(relPixelPos, m_vertical), absPixelPos[2]);
			Vec3f hemisphericalPoint = absPixelPos + sqrt(1 - squaredLength) * m_gatheringNormal;			
			Vec3f direction = normalize(hemisphericalPoint - m_gatheringPos);
			return direction;
		}
	}

	bool MicroBuffer::directionToPixel(Vec3f direction, int& i, int& j)
	{ 
		Vec3f normDirection = normalize(direction);
		Vec3f normalizedPixelPos = m_gatheringPos + normDirection - dot(normDirection, m_gatheringNormal)*m_gatheringNormal ;		
		return positionToPixel(normalizedPixelPos,i,j);
	}

	float MicroBuffer::solidAngle(size_t i, size_t j)
	{
		//if (i < 1) i++;
		//if (i > m_width-1) i--;
		//if (j < 1) j++;
		//if (j > m_height-1) j--;
		//Vec3f dx = (pixelToDirection(i + 1, j) - pixelToDirection(i - 1, j)) / 2.f;
		//Vec3f dy = (pixelToDirection(i, j+1) - pixelToDirection(i, j-1)) / 2.f;
		//float solidAngle = dx.length() * dy.length();
		//return solidAngle;
		return(dot(pixelToDirection(i, j), m_gatheringNormal));
	}

	//BVH traversal
	void MicroBuffer::fillMicroBuffer(BVHnode::BVHptr node)
	{			
		//compute distance and solid angle
		Vec3f direction = (node->position() - m_gatheringPos);
		float distance = direction.length();
		bool hasChildren = (node->surfels().size() > 1);
		float BVHsolidAngle = 0;
		//if (hasChildren) BVHsolidAngle = M_PI * sin(node->normalAngle())*sin(node->normalAngle());
		if (hasChildren) BVHsolidAngle = M_PI * (node->radius() * node->radius()) / (distance * distance);
		else if(node->surfels().size() ==1 ) BVHsolidAngle = M_PI * (node->surfels()[0].radius * node->surfels()[0].radius) / (distance * distance);
		int indexI, indexJ;
		directionToPixel(direction, indexI, indexJ);
		//std::cout << "Pixel solid angle : " << solidAngle(indexI, indexJ) << std::endl;
		if (BVHsolidAngle < solidAngle(indexI,indexJ)) //rasterize node directly
		{			
			if (depth(indexI, indexJ) > distance)
			{
				std::cout << "rasterized : " << indexI << " - " << indexJ << std::endl;
				setDepthValue(indexI, indexJ, distance);
				setIndex(indexI, indexJ, node);
				setColorValue(indexI, indexJ, node->getColor());
			}			
		}
		else
		{
			if (hasChildren)
			{
				m_depth++;
				fillMicroBuffer(node->left());
				fillMicroBuffer(node->right());
			}
			else //update post traversal list with too big leaf nodes for rasterization
			{
				m_postTraversalList.push_back(node);
			}
		}				
	}

	//ray cast leave node for precise rasterization
	void MicroBuffer::postTraversalRayCasting()
	{		
		std::cout << m_postTraversalList.size() << std::endl;
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
							std::cout << "raytraced : " << i << " - " << j << std::endl;
							setDepthValue(i, j, parT);
							setIndex(i, j, m_postTraversalList[k]);
							setColorValue(i, j, surfel.color);
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
				float solidAgl = solidAngle(i, j);			
				Vec3f direction = pixelToDirection(i, j);
				for (int k = 0; k < lights.size(); k++)
				{
					totalColorResponse += color(i, j) * mat.evaluateColorResponse(m_gatheringPos, m_gatheringNormal, direction)*dot(m_gatheringNormal,direction)/10.f;
				}
			}
		}
		return totalColorResponse;
	}
