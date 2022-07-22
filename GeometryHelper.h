#pragma once

#include "Vec3.h"
#include "mesh.h"

class GeometryHelper
{
public:
	static Vec3f sampleDiskUniform(float randomX, float randomY);

    inline static Vec3f computeTriangleNormal(Vec3<Vec3f> triangle)
    {
        return normalize(cross(triangle[1] - triangle[0], triangle[2] - triangle[0]));
    }

	static Vec3f sampleCosineHemisphereConcentric(float rdX, float rdY, const Vec3f& normal, float& pdf);

    /// <summary>
    /// Sample input triangle uniformly and returns a random point on this triangle. ref : https://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle-in-3d
    /// </summary>
    /// <param name="triangle : input triangle"></param>
    /// <returns></returns>
    inline static Vec3f sampleTriangleUniformly(const Vec3<Vec3f>& triangle)
    {
        float r1 = random(0, 1);
        float r2 = random(0, 1);
        float r1Sqrt = sqrt(r1);
        return (1 - r1Sqrt) * triangle[0] + (r1Sqrt * (1 - r2)) * triangle[1] + (r2 * r1Sqrt) * triangle[2];
    }

    /// <summary>
    /// Returns a random float in specified range
    /// </summary>
    /// <param name="min"></param>
    /// <param name="max"></param>
    /// <returns></returns>
    inline static float random(float min, float max)
    {
        return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
    }

private:
    static Vec3f orientAlongNormal(const Vec3f& input, const Vec3f& normal);

    inline static Vec3f perp_stark(const Vec3f & u)
    {
        Vec3f a = Vec3f(abs(u[0]) , abs(u[1]), abs(u[2]));
        uint32_t uyx = (a[0] - a[1]) < 0 ? 1 : 0;
        uint32_t uzx = (a[0] - a[2]) < 0 ? 1 : 0;
        uint32_t uzy = (a[1] - a[2]) < 0 ? 1 : 0;
        uint32_t xm = uyx & uzx;
        uint32_t ym = (1 ^ xm) & uzy;
        uint32_t zm = 1 ^ (xm | ym); // 1 ^ (xm & ym)
        Vec3f v = cross(u, Vec3f(xm, ym, zm));
        return v;
    }
};

