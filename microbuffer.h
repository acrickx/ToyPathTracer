#pragma once

#include"Vec3.h"
#include "Ray.h"
#include "pointCloud.h"
#include<vector>

class  MicroBuffer
{
public:

	MicroBuffer(size_t size, Vec3f gatheringPos, Vec3f gatheringNormal)
	{
		m_gatheringPos = gatheringPos;
		m_gatheringNormal = normalize(gatheringNormal);
		if (m_gatheringNormal == Vec3f(0, 1, 0)) { m_horizontal = normalize(cross(Vec3f(0, 0, -1),m_gatheringNormal)); }
		else m_horizontal = normalize(cross(Vec3f(0, 1, 0), m_gatheringNormal));
		m_vertical = normalize(cross(m_gatheringNormal, m_horizontal));
		m_bottomLeftCorner = m_gatheringPos - (m_horizontal + m_vertical);
		m_width = m_height = size;
		//initialize vectors
		m_colors = std::vector<Vec3f>(size * size);
		m_zBuffer = std::vector<float>(size * size, std::numeric_limits<float>().max());
		m_indexBuffer = std::vector<BVHnode::BVHptr>(size * size);
	};
	//accessors
	const size_t size() const { return m_width; }
	inline const Vec3f color(size_t i, size_t j) const { return m_colors[j * m_width + i]; }
	inline const float depth(size_t i, size_t j) const { return m_zBuffer[j * m_width + i]; }	
	inline const BVHnode::BVHptr index(size_t i, size_t j) const { return m_indexBuffer[j * m_width + i]; }
	inline void setColorValue(size_t i, size_t j, Vec3f colorValue) { m_colors[j * m_width + i] = colorValue; };
	inline void setDepthValue(size_t i, size_t j, float depthValue) { m_zBuffer[j * m_width + i] = depthValue; };
	inline void setIndex(size_t i, size_t j, BVHnode::BVHptr index) { m_indexBuffer[j * m_width + i] = index; };
	//pixel/direction mapping
	bool positionToPixel(Vec3f pos, int& i, int& j);
	Vec3f pixelToPostion(size_t i, size_t j);
	Vec3f pixelToDirection(size_t i, size_t j);
	bool directionToPixel(Vec3f direction, int& i, int& j);
	float solidAngle(size_t i, size_t j);
	//rendering
	void fillMicroBuffer(BVHnode::BVHptr node);
	void postTraversalRayCasting();
	Vec3f convolveBRDF(Material mat, const Scene& scene);


private:
	size_t m_width = 0;
	size_t m_height = 0;
	int m_depth = 0;
	Vec3f m_gatheringPos;
	Vec3f m_gatheringNormal;
	Vec3f m_horizontal;
	Vec3f m_vertical;
	Vec3f m_bottomLeftCorner;
	std::vector<Vec3f> m_colors;
	std::vector<float> m_zBuffer;
	std::vector<BVHnode::BVHptr> m_indexBuffer;
	std::vector<BVHnode::BVHptr> m_postTraversalList;
};

