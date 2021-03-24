#include "BSHnode.h"

BSHnode::BSHnode(const std::vector<Surfel>& surfels)
{            
    Sphere sphere = computeBoundingSphere(surfels);
    new (this) BSHnode(surfels, sphere);
}

 BSHnode::BSHnode(const std::vector<Surfel>& surfels, Sphere sphere)
{
    //stop condition
    if (surfels.size() <= 1)
    {
        m_surfels = surfels;
        m_sphere = sphere;
        m_radius = surfels[0].radius;
        m_hasChildren = false;
    }
    else
    {
        m_surfels = surfels;
        m_sphere = sphere;
        m_radius = sphere.radius();
        m_hasChildren = true;
        //determine in which dimension to split 
        AABB aabb{};
        for (int i = 0; i < surfels.size(); i++)
        {
            aabb.compareAndUpdate(surfels[i].position);
        }
        Vec3f diff = aabb.max() - aabb.min();
        int dimension = -1;
        if (diff[0] >= std::max(diff[1], diff[2])) dimension = 0;
        else if (diff[1] >= std::max(diff[0], diff[2])) dimension = 1;
        else if (diff[2] >= std::max(diff[1], diff[0])) dimension = 2;
        std::vector<float> centers;
        //initialize vector of surfel positions for sorting
        for (int i = 0; i < surfels.size(); i++)
        {
            centers.push_back(surfels[i].position[dimension]);
        }
        std::sort(centers.begin(), centers.end());
        float median = centers[centers.size() / 2];
        //sort surfels in two subsets + compute average normal
        std::vector<Surfel> surfelsLeft, surfelsRight;
        Vec3f totalNormal;
        bool addLeft = false;
        for (int i = 0; i < surfels.size(); i++)
        {
            Vec3f center = surfels[i].position;  
            //add normal to total for average
            totalNormal += surfels[i].normal;
            //altern left and right if positions are equal
            if (center[dimension] == median)
            {
                if (addLeft) surfelsLeft.push_back(surfels[i]);
                else surfelsRight.push_back(surfels[i]);                
                addLeft = !addLeft;
            }
            //first half
            else if (center[dimension] < median)
            {
                surfelsLeft.push_back(surfels[i]);
            }
            //second half
            else
            {
                surfelsRight.push_back(surfels[i]);
            }
        }
        //assign normal value
        m_normal = normalize(totalNormal / surfels.size());
        //compute BoundingSphere
        Sphere leftSphere = computeBoundingSphere(surfelsLeft);
        Sphere rightSphere = computeBoundingSphere(surfelsRight);
        m_left = BSHptr(new BSHnode(surfelsLeft, leftSphere)); 
        m_right = BSHptr(new BSHnode(surfelsRight, rightSphere));
        //std::cout << "n_surfels : " << surfels.size() <<  " |  sphere - center : " << sphere.center() << " radius : " << sphere.radius() << " | angle : " << m_normalConeAngle << std::endl;
    }    
}

 Sphere BSHnode::computeBoundingSphere(std::vector<Surfel> surfels)
 {
     const Vec3f& init = surfels[0].position;
     //First step : going through the points to find extreme points along each direction, + compute normal angle
     Vec3f minX = init, minY = init, minZ = init, maxX = init, maxY = init, maxZ = init;
     for (int i = 0; i < surfels.size(); i++)
     {
         //test all normals to find max cone angle
         if (surfels[i].normal != m_normal)
         {
             float angle = acos((dot(surfels[i].normal, m_normal)));
             if (abs(angle) > abs(m_normalConeAngle)) m_normalConeAngle = angle;             
         }
         //test center to find extreme points in each direction & filling the two subsets
         Vec3f center = surfels[i].position;
         if (center[0] < minX[0]) minX = center;
         else if (center[0] > maxX[0]) maxX = center;
         if (center[1] < minY[1]) minY = center;
         else if (center[1] < maxY[1]) maxY = center;
         if (center[2] < minZ[2]) minZ = center;
         else if (center[2] < maxZ[2]) maxZ = center;
     }
     //Second step : create inital sphere for max direction and update it
     float xSpan = (maxX - minX).squaredLength();
     float ySpan = (maxY - minY).squaredLength();
     float zSpan = (maxZ - minZ).squaredLength();
     float maxSpan = xSpan; Vec3f minPt = minX, maxPt = maxX;
     if (ySpan > maxSpan)
     {
         maxSpan = ySpan;
         minPt = minY; maxPt = maxY;
     }
     if (zSpan > maxSpan)
     {
         maxSpan = zSpan;
         minPt = minZ; maxPt = maxZ;
     }
     Vec3f ctr = (minPt + maxPt) / 2.f;
     Sphere sphere(ctr, (maxPt - ctr).length());
     for (int i = 0; i < surfels.size(); i++)
     {
         Vec3f pos = surfels[i].position;
         float dist = (pos - sphere.center()).length();
         //if point is outside sphere
         if (dist > sphere.radius())
         {
             //update radius            
             float test = sphere.radius();
             sphere.setRadius((dist + sphere.radius()) / 2.f);
             //update center
             Vec3f centerShift = dist - sphere.radius();
             sphere.center() = (sphere.radius() * sphere.center() + centerShift * pos) / dist;
         }
     }
     return sphere;
 }

 Vec3f BSHnode::getColor()
 {
     Vec3f color;
     for (int i = 0; i < m_surfels.size(); i++)
     {
         color += m_surfels[i].color;
     }
     return color / m_surfels.size();
 }


