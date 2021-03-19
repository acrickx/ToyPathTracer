#pragma once
#include <vector>
#include <algorithm>

#include "Vec3.h"
#include "mesh.h"
#include "boundingVolume.h"
#include "surfel.h"
#include "scene.h"

class BVHnode {

public:
    using BVHptr = std::shared_ptr<BVHnode>;
    typedef std::shared_ptr<BVHnode> BVHptr;    
    BVHnode(const std::vector<Surfel>& surfels);    
    BVHnode(const std::vector<Surfel>& surfels, Sphere sphere);
    Sphere computeBoundingSphere(std::vector<Surfel> surfels);  
    inline const Vec3f position() const { return m_sphere.center(); }
    inline const Vec3f normal() const { return m_normal; }
    inline const float radius() const { return m_sphere.radius(); }
    inline const BVHptr left() const { return m_left; }
    inline const BVHptr right() const { return m_right; }
    inline const std::vector<Surfel> const surfels() { return m_surfels; }
    Vec3f computeColor(const Scene& scene, Material mat);

private:
    BVHptr m_left = nullptr;
    BVHptr m_right = nullptr;
    Sphere m_sphere{};
    std::vector<Surfel> m_surfels;
    //normal representation
    float m_normalConeAngle;
    Vec3f m_normal;
};