#pragma once

#include"microbuffer.h"

//Coordinate system changes and pixel/direction mapping
	bool MicroBuffer::positionToPixel(const Vec3f& pos, int& i, int& j) 
	{
		i = (int)(floor(dot(pos - m_bottomLeftCorner, m_horizontal) * (m_width-1) / 2.f));
		j = (int)(floor(dot(pos - m_bottomLeftCorner, m_vertical) * (m_height-1) / 2.f));
		if (i > m_width-1 || i<0|| j>m_height-1 || j < 0) { 
			//std::cerr << "error : given pos is outside the image bounds - coord found : (" << i << "," << j << ")\n"; 
			return false; 
		}
		return true;
	}

	Vec3f MicroBuffer::pixelToPostion(int i, int j)
	{
		//if (i > m_width || i<0 || j>m_height || j < 0) { std::cerr << "error : given pos is outside the image bounds - coord : (" << i << "," << j << ")\n";}
		return m_bottomLeftCorner + m_horizontal * 2 * i / (m_width-1) + m_vertical * 2 * j / (m_height-1);
	}

	Vec3f MicroBuffer::pixelToDirection(int i, int j)
	{
		Vec3f direction;
		float size = m_width; // = m_height;
		const Vec3f& absPixelPos = pixelToPostion(i, j);
		Vec3f relPixelPos = absPixelPos-m_gatheringPos;			
		float squaredLength = relPixelPos[0] * relPixelPos[0] + relPixelPos[1] * relPixelPos[1];
		if (squaredLength < 1.f)
		{
			Vec3f hemisphericalPoint = absPixelPos + sqrt(1.f - squaredLength) * m_gatheringNormal;			
			direction = normalize(hemisphericalPoint - m_gatheringPos);
		}
		return direction;		
	}

	bool MicroBuffer::directionToPixel(const Vec3f& direction, int& i, int& j)
	{ 
		Vec3f normDirection = normalize(direction);
		Vec3f normalizedPixelPos = m_gatheringPos + normDirection - dot(normDirection, m_gatheringNormal)*m_gatheringNormal ;		
		return positionToPixel(normalizedPixelPos,i,j);
	}

	float MicroBuffer::solidAngle(int i, int j)
	{
		if (i < 1) i++;
		if (i > m_width-1) i--;
		if (j < 1) j++;
		if (j > m_height-1) j--;
		Vec3f dx = (pixelToDirection(i + 1, j) - pixelToDirection(i - 1, j)) / 2.f;
		Vec3f dy = (pixelToDirection(i, j+1) - pixelToDirection(i, j-1)) / 2.f;
		float solidAngle = dx.length() * dy.length();
		return solidAngle;
	}

	//BVH traversal
	void MicroBuffer::fillMicroBuffer(BSHnode::BSHptr node)
	{			
		//compute distance and solid angle
		Vec3f direction = (node->position() - m_gatheringPos);
		float distance = direction.length();			
		float BVHsolidAngle = (node->radius() * node->radius())/(distance*distance);
		int indexI, indexJ;
		directionToPixel(direction, indexI, indexJ);
		if (BVHsolidAngle < solidAngle(indexI, indexJ)) //rasterize node directly
		{
			if (depth(indexI, indexJ) > distance)
			{
				setDepthValue(indexI, indexJ, distance);
				setIndex(indexI, indexJ, node);
				setColorValue(indexI, indexJ, node->getColor());				
			}
		}
		else
		{
			if (node->hasChildren())
			{				
				fillMicroBuffer(node->left());
				fillMicroBuffer(node->right());
			}
			else //update post traversal list with too big leaf nodes for rasterization
			{
				m_postTraversalList.push_back(node);
			}
		}
	}


	//BVH traversal //DEBUG MODE
	void MicroBuffer::fillMicroBuffer(BSHnode::BSHptr node, std::vector<Surfel>& surfels)
	{
		//compute distance and solid angle
		Vec3f direction = (node->position() - m_gatheringPos);
		float distance = direction.length();
		float BVHsolidAngle = (node->radius() * node->radius()) / (distance * distance);
		int indexI, indexJ;
		directionToPixel(direction, indexI, indexJ);		
		if (BVHsolidAngle < solidAngle(indexI, indexJ)) //rasterize node directly
		{
			if (depth(indexI, indexJ) > distance)
			{
				setDepthValue(indexI, indexJ, distance);
				setIndex(indexI, indexJ, node);
				setColorValue(indexI, indexJ, node->getColor());
				surfels[indexJ * m_width + indexI] = Surfel(node->position(), node->normal(), node->getColor(), node->radius());
			}
		}
		else
		{
			if (node->hasChildren())
			{
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
		for (int j = 0; j < m_height; j++)
		{
			for (int i = 0; i < m_width; i++)
			{
				Ray ray(m_gatheringPos, pixelToDirection(i, j));
				#pragma omp parallel for
				for (int k = 0; k < m_postTraversalList.size(); k++)
				{
					const BSHnode::BSHptr& node = m_postTraversalList[k];
					Vec3f intersectionPos; float parT = 0;
					Surfel surfel = node->surfels()[0];
					if (ray.testDiscIntersection(surfel.position, surfel.normal, surfel.radius, intersectionPos, parT))
					{
						if (parT < depth(i, j))
						{
							setDepthValue(i, j, parT);
							setIndex(i, j, m_postTraversalList[k]);
							setColorValue(i, j, surfel.color);							
						}
					}
				}
			}
		}
	}

	//ray cast leave node for precise rasterization // DEBUG MODE
	void MicroBuffer::postTraversalRayCasting(std::vector<Surfel>& surfels)
	{				
		for (int j = 0; j < m_height; j++)
		{
			for (int i = 0; i < m_width; i++)
			{
				Ray ray(m_gatheringPos, pixelToDirection(i, j));
				#pragma omp parallel for
				for (int k = 0; k < m_postTraversalList.size(); k++)
				{
					BSHnode::BSHptr node = m_postTraversalList[k];
					Vec3f intersectionPos; float parT = 0;
					Surfel surfel = node->surfels()[0];
					if (ray.testDiscIntersection(surfel.position, surfel.normal, surfel.radius, intersectionPos, parT))
					{
						if (parT < depth(i, j))
						{	
							setDepthValue(i, j, parT);
							setIndex(i, j, m_postTraversalList[k]);
							setColorValue(i, j, surfel.color);							
							surfels[j * m_width + i] = surfel;
						}
					}
				}
			}
		}
	}

	//convolve microbuffer with BRDF
	Vec3f MicroBuffer::convolveBRDF(const Material& mat, const Scene& scene)
	{			
		const Vec3f& eye = scene.camera().getPosition();
		Vec3f totalColorResponse = mat.albedo;
		for (int i = 0; i < m_width; i++)
		{			
			for (int j = 0; j < m_height; j++)
			{
				float solidAgl = solidAngle(i, j);			
				const Vec3f& direction = pixelToDirection(i, j);
				if (direction != Vec3f(0, 0, 0))
				{
					Vec3f pixelColor = color(i, j) * solidAgl * mat.colorResponse(m_gatheringPos,m_gatheringNormal,direction,eye);
					totalColorResponse += pixelColor;
				}
			}
		}
		return totalColorResponse;
	}
