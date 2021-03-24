#pragma once
#include <vector>
#include <algorithm>

#include "Vec3.h"
#include "mesh.h"
#include "boundingVolume.h"
#include "surfel.h"
#include "rayTracer.h"

class BSHnode {

public:
    using BSHptr = std::shared_ptr<BSHnode>;
    typedef std::shared_ptr<BSHnode> BSHptr;    
    BSHnode(const std::vector<Surfel>& surfels);    
    BSHnode(const std::vector<Surfel>& surfels, const Sphere& sphere);
    Sphere computeBoundingSphere(const std::vector<Surfel>& surfels);  
    inline const Vec3f position() const { return m_sphere.center(); }
    inline const Vec3f normal() const { return m_normal; }
    inline const float normalAngle() const { return m_normalConeAngle; }
    inline const float radius() const { return m_radius; }
    inline const BSHptr left() const { return m_left; }
    inline const BSHptr right() const { return m_right; }
    inline const bool  hasChildren() const { return m_hasChildren; }
    inline const std::vector<Surfel> const surfels() { return m_surfels; }
    Vec3f getColor();

private:
    BSHptr m_left = nullptr;
    BSHptr m_right = nullptr;
    bool m_hasChildren = true;
    float m_radius = 0;
    Sphere m_sphere{};
    std::vector<Surfel> m_surfels;
    //normal representation
    float m_normalConeAngle;
    Vec3f m_normal;
};