#pragma once
#include "mesh.h"
#include "Vec3.h"
#include "GeometryHelper.h"

class EmissiveMesh : public Mesh
{
public:
	EmissiveMesh(float emissive, Vec3f color)
	{
		m_mat = MaterialEmissive(color, emissive);
	}

	static Vec3f getRandomPointOnSurface(const Mesh& mesh)
	{
		if (mesh.indices().size() == 0) return Vec3f{};
		size_t rdIndex = size_t((mesh.indices().size()-1) * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)));
		auto& rdTri = mesh.triangle(mesh.indices()[rdIndex]);
		return GeometryHelper::sampleTriangleUniformly(rdTri);
	}
};

