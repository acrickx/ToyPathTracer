#pragma once

#include "Vec3.h"

class GeometryHelper
{
public:
	static Vec3f sampleDiskUniform(float randomX, float randomY);

	static Vec3f sampleCosineHemisphereConcentric(float rdX, float rdY, const Vec3f& normal, float& pdf);

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

