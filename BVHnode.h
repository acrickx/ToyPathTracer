#pragma once
#include <vector>
#include <algorithm>

#include "Vec3.h"
#include "mesh.h"
#include "boundingVolume.h"

struct hitInfo {
	float parT;
	Vec3f barCoord;
	size_t meshIndex;
	Vec3i triangleIndices;
	hitInfo() { parT = std::numeric_limits<float>::max(); meshIndex = -1; };
	hitInfo(float _parT, Vec3f _barCoord, size_t _meshIndex, Vec3i _trianglesIndices) : parT(_parT), barCoord(_barCoord), meshIndex(_meshIndex), triangleIndices(_trianglesIndices) {};
};


class BVHnode {

public:
    using BVHptr = std::shared_ptr<BVHnode>;
    typedef std::shared_ptr<BVHnode> BVHptr;

    inline BVHnode() {}    
    BVHnode(const std::vector<Vec3i>& connectivity, const AABB& aabb, const Mesh& mod, int meshIndex);
    bool hit(const Ray& ray, hitInfo& hitRecord, const std::vector<Mesh>& meshes);

private:
    BVHptr m_left = nullptr;
    BVHptr m_right = nullptr;
    AABB m_aabb{};
    std::vector<Vec3i> m_connectivity{};    
    int m_meshIndex = -1;
};

class BVHroot {

public:
    inline BVHroot() {}

    inline BVHroot(const std::vector<Mesh>& meshes)        
    {
        Vec3f minScene, maxScene;
        for (int i = 0; i < meshes.size(); i++)
        {
            AABB BB = meshes[i].boundingBox();
            for (int k = 0; k < 3; k++)
            {
                if (BB.min()[k] < minScene[k]) minScene[k] = BB.min()[k];
                if (BB.max()[k] > maxScene[k]) maxScene[k] = BB.max()[k];
            }
            m_nodes.push_back(BVHnode::BVHptr(new BVHnode(meshes[i].indices(), BB, meshes[i], i)));
        }
        m_aabb = AABB(minScene, maxScene);
    };

    bool hit(const Ray& ray, hitInfo& hitRecord, const std::vector<Mesh>& meshes)
    {
        hitInfo closestHit{}; bool intersect = false;
        for (int i = 0; i < m_nodes.size(); i++)
        {
            hitInfo meshHitInfo{};                        
            if (m_nodes[i]->hit(ray, meshHitInfo, meshes))
            {            
                intersect = true;
                if (meshHitInfo.parT < closestHit.parT)
                {                                                                
                    closestHit = meshHitInfo;
                }
            }
        }        
        hitRecord = closestHit;
        return intersect;
    }

private:
    std::vector<BVHnode::BVHptr> m_nodes;
    AABB m_aabb;
};




