#include "GeometryHelper.h"

/** Uniform sampling of the unit disk using Shirley's concentric mapping.
    \param[in] rdX, rdY =  uniform random numbers in [0,1)^2.
    \return Sampled point on the unit disk (format : (x,y,0)).
*/
Vec3f GeometryHelper::sampleDiskUniform(float rdX, float rdY)
{
    Vec3f output{};
    rdX = 2.f * rdX - 1.f;
    rdY = 2.f * rdY - 1.f;
    if (rdX == 0.f && rdY == 0.f) return output;
    float phi, r;
    if (abs(rdX) > abs(rdY))
    {
        r = rdX;
        phi = (rdY / rdX) * M_PI_4;
    }
    else
    {
        r = rdY;
        phi = M_PI_2 - (rdX / rdY) * M_PI_4;
    }
    return r * Vec3f(cos(phi), sin(phi), 0.f);

}

Vec3f GeometryHelper::orientAlongNormal(const Vec3f& input, const Vec3f& normal)
{
    Vec3f ortho = perp_stark(input);
    Vec3f tangent = cross(ortho, normal);
    return input[0] * ortho + input[1] * tangent + input[2] * normal;
}

Vec3f GeometryHelper::sampleCosineHemisphereConcentric(float rdX, float rdY, const Vec3f& normal, float& pdf)
{
    Vec3f randomDiskPoint = sampleDiskUniform(rdX, rdY);
    float z = sqrt(fmaxf(0.f, 1.f - dot(randomDiskPoint, randomDiskPoint)));
    pdf = z * M_1_PI;
    Vec3f direction = (randomDiskPoint[0], randomDiskPoint[1], z);
    Vec3f localDirection = orientAlongNormal(direction, normal);
    return localDirection;
}