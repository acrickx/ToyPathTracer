#pragma once
#include <vector>
#include <algorithm>

#include "Vec3.h"
#include "mesh.h"
#include "boundingVolume.h"
#include "surfel.h"

class BVHnode {

public:
    using BVHptr = std::shared_ptr<BVHnode>;
    typedef std::shared_ptr<BVHnode> BVHptr;    
    BVHnode(const std::vector<Surfel>& surfels);    
    BVHnode(const std::vector<Surfel>& surfels, Sphere sphere);
    Sphere computeBoundingSphere(std::vector<Surfel> surfels);    

private:
    BVHptr m_left = nullptr;
    BVHptr m_right = nullptr;
    Sphere m_sphere{};
    std::vector<Surfel> m_surfels;
    //normal representation
    float m_normalConeAngle;
    Vec3f m_normal;

};