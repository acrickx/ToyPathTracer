#include "BVHnode.h"

 BVHnode::BVHnode(const std::vector<Vec3i>& connectivity, const AABB& aabb, const Mesh& mod, int meshIndex)
{
    //std::cout << "size : " << connectivity.size() << std::endl;
    //stop condition
    if (connectivity.size() <= 1)
    {
        m_connectivity = connectivity;
        m_aabb = aabb;
        m_meshIndex = meshIndex;
    }
    else
    {
        m_connectivity = connectivity;
        m_aabb = aabb;
        m_meshIndex = meshIndex;
        //determine in which dimension to split 
        Vec3f diff = aabb.max() - aabb.min();
        int dimension = -1;
        if (diff[0] >= std::max(diff[1], diff[2])) dimension = 0;
        else if (diff[1] >= std::max(diff[0], diff[2])) dimension = 1;
        else if (diff[2] >= std::max(diff[1], diff[0])) dimension = 2;
        std::vector<float> barycenters;
        //compute triangle barycenter
        for (int i = 0; i < connectivity.size(); i++)
        {
            const Vec3f& pt0 = mod.vertices()[connectivity[i][0]];
            const Vec3f& pt1 = mod.vertices()[connectivity[i][1]];
            const Vec3f& pt2 = mod.vertices()[connectivity[i][2]];
            Vec3f bar = (pt0 + pt1 + pt2) / 3.f;
            barycenters.push_back(bar[dimension]);
        }
        std::sort(barycenters.begin(), barycenters.end());
        float median = barycenters[barycenters.size() / 2];
        //sort triangles in two subsets
        std::vector<Vec3i> connectivityLeft, connectivityRight;
        const Vec3f& init = mod.vertices()[connectivity[0][0]];
        Vec3f minRight = init, minLeft = init, maxRight = init, maxLeft = init;
        bool addLeft = false;
        for (int i = 0; i < connectivity.size(); i++)
        {
            const Vec3f& pt0 = mod.vertices()[connectivity[i][0]];
            const Vec3f& pt1 = mod.vertices()[connectivity[i][1]];
            const Vec3f& pt2 = mod.vertices()[connectivity[i][2]];
            Vec3f bar = (pt0 + pt1 + pt2) / 3.f;
            if (bar[dimension] == median)
            {
                if (addLeft) {
                    connectivityLeft.push_back(connectivity[i]);
                    for (int j = 0; j < 3; j++)
                    {
                        minLeft[j] = std::min(minLeft[j], std::min(pt2[j], std::min(pt0[j], pt1[j])));
                        maxLeft[j] = std::max(maxLeft[j], std::max(pt2[j], std::max(pt0[j], pt1[j])));
                    }
                }
                else {
                    connectivityRight.push_back(connectivity[i]);
                    for (int j = 0; j < 3; j++)
                    {
                        minRight[j] = std::min(minRight[j], std::min(pt2[j], std::min(pt0[j], pt1[j])));
                        maxRight[j] = std::max(maxRight[j], std::max(pt2[j], std::max(pt0[j], pt1[j])));
                    }
                }
                addLeft = !addLeft;
            }
            else if (bar[dimension] < median)
            {
                connectivityLeft.push_back(connectivity[i]);
                for (int j = 0; j < 3; j++)
                {
                    minLeft[j] = std::min(minLeft[j], std::min(pt2[j], std::min(pt0[j], pt1[j])));
                    maxLeft[j] = std::max(maxLeft[j], std::max(pt2[j], std::max(pt0[j], pt1[j])));
                }
            }
            else
            {
                connectivityRight.push_back(connectivity[i]);
                for (int j = 0; j < 3; j++)
                {
                    minRight[j] = std::min(minRight[j], std::min(pt2[j], std::min(pt0[j], pt1[j])));
                    maxRight[j] = std::max(maxRight[j], std::max(pt2[j], std::max(pt0[j], pt1[j])));
                }
            }
        }
        //compute AABB
        AABB aabbLeft(minLeft, maxLeft), aabbRight(minRight, maxRight);        
        m_left = BVHptr(new BVHnode(connectivityLeft, aabbLeft, mod, meshIndex)); 
        m_right = BVHptr(new BVHnode(connectivityRight, aabbRight, mod, meshIndex));
    }    
}

 bool BVHnode::hit(const Ray& ray, hitInfo& hitRecord, const std::vector<Mesh>& meshes)
 {
     float tmin, tmax;
     //the root has no defined bounding box and its meshIndex is = to -1     
     if (m_aabb.hit(ray, tmin, tmax) || m_connectivity.size()==0)
     {   
         //Stop condition : node is leaf
         if (m_connectivity.size() <= 1)
         {                          
             bool test = false;
             for (int i = 0; i < m_connectivity.size(); i++)
             {
                 float t; Vec3f barCoord;
                 bool triangleIntersect = ray.testTriangleIntersection(meshes[m_meshIndex].triangle(m_connectivity[i]), barCoord, t, 0.0001f);
                 if (triangleIntersect && t < hitRecord.parT)
                 {                     
                     hitRecord.barCoord = barCoord;
                     hitRecord.parT = t;
                     hitRecord.meshIndex = m_meshIndex;
                     hitRecord.triangleIndices = m_connectivity[0];
                     test = true;
                 }
             }
             return test;
         }
         else
         {             
             hitInfo hitInfo1(hitRecord), hitInfo2(hitRecord);
             bool hit1 = m_left->hit(ray, hitInfo1, meshes);
             bool hit2 = m_right->hit(ray, hitInfo2, meshes);
             if (hit1 && hit2)
             {
                 if (hitInfo1.parT < hitInfo2.parT)
                 {
                     hitRecord = hitInfo1;
                     return hit1;
                 }
                 else
                 {
                     hitRecord = hitInfo2;
                     return hit2;
                 }
             }
             else if (hit1) { hitRecord = hitInfo1; return hit1; }
             else if (hit2) { hitRecord = hitInfo2; return hit2; }
             else 
             { 
                 //std::cout << "false" << std::endl;
                 return false; 
             }
         }
     }
     else return false;
 }
