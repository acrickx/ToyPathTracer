#include <iostream>
#include <chrono>
#include "image.h"
#include "scene.h"
#include "lightSource.h"
#include "rayTracer.h"
#include "pointCloud.h"
#include"pointBasedRenderer.h"
 
using namespace std;

//debug
void debugMicrobuffer(size_t i, size_t j, Camera cam, const Scene& scene, BVHnode::BVHptr root)
{
    size_t size = 24;
    Ray ray(cam.getPosition(), cam.getImageCoordinate(i, 1 - j));
    Vec3f intersectionPos, intersectionNorm; size_t meshIndex, triangleIndex;
    RayTracer::rayTrace(ray, scene, intersectionPos, intersectionNorm, meshIndex, triangleIndex);
    MicroBuffer debugMb(size, intersectionPos, intersectionNorm);    
    debugMb.postTraversalRayCasting();
    Image debugImage(size, size);
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            debugImage(i, j) = debugMb.color(i, j);
        }
    }
    debugImage.savePPM("DebugMicroBuffer.ppm");
}

void debugMicrobuffer(Vec3f position, Vec3f normal, const Scene& scene, BVHnode::BVHptr root, std::string filename, std::vector<Surfel>& surfels)
{
    size_t size = 24;
    surfels.clear();
    surfels.resize(size * size);
    MicroBuffer debugMb(size, position, normal);    
    surfels = std::vector<Surfel>(size * size);
    debugMb.fillMicroBuffer(root, surfels);
    debugMb.postTraversalRayCasting();
    Image debugImage(size*25, size*25);
    int w = debugImage.getWidth(), h = debugImage.getHeight();
    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
        {
            debugImage(i, h-1-j) = debugMb.color((size_t)(i / 25.f), (size_t)(j / 25.f));            
        }
    }    
    debugImage.savePPM(filename);
}

int main(int argc, char* argv[])
{
    float aspectRatio = 1.f;
    size_t width=400, height= width/aspectRatio;
    string filename="backgroundImage.ppm";
    //"Usage : ./MyRayTracer –width <width> -height <height> -output <filename.ppm>" << std::endl;
    if (argc >1)
    {
        for (int i = 1; i < argc; i++)
        {
            if (std::string(argv[i]) == "-width")
            {
                width = std::stoi(argv[i + 1]);
                std::cout << "w : " << width << std::endl;
            }
            else if (std::string(argv[i]) == "-height")
            {
                height = std::stoi(argv[i + 1]);
                std::cout << "h : " << height << std::endl;
            }
            else if (std::string(argv[i]) == "-output")
            {
                filename = argv[i + 1];
                std::cout << "filename : " << filename << std::endl;
            }
        }
    }
    Image image(width, height);
    //MESHES
    Mesh plane = Plane(Vec3f(0, -0.25f, 0), Vec3f(0, 1, 0), Vec3f(1, 0, 0), 0.5f, Material(Vec3f(0.7f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    Mesh topPlane = Plane(Vec3f(0, 0.25f, 0), Vec3f(0, -1, 0), Vec3f(1, 0, 0), 0.5f, Material(Vec3f(0.0f, 0.0f, 0.7f), 1.0f, 1.0f, 0.f));
    Mesh backPlane = Plane(Vec3f(0, 0, -0.25f), Vec3f(0, 0, 1), Vec3f(1, 0, 0), 0.5f , Material(Vec3f(0.7f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    Mesh leftPlane = Plane(Vec3f(-0.25f, 0, 0.f), Vec3f(1, 0, 0), Vec3f(0, 0, -1), 0.5f, Material(Vec3f(0.7f, 0.0f, 0.0f), 1.0f, 1.0f, 0.f));
    Mesh rightPlane = Plane(Vec3f(0.25f, 0, 0.f), Vec3f(-1, 0, 0), Vec3f(0, 0, 1), 0.5f, Material(Vec3f(0.f, 0.7f, 0.f), 1.0f, 1.0f, 0.f));
    Mesh model(Material(Vec3f(0.4f, 0.4f, 0.7f), 1.0f, 1.0f, 0.f));    
    model.loadOFF("example_lowres.off");        
    model.scale(0.5f);
    //Mesh cubePlane = Plane(Vec3f(0, -0.3f, 0), Vec3f(0, 1, 0), Vec3f(1, 0, 0), 0.2f, Material(Vec3f(0.1f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    //Mesh cubeTopPlane = Plane(Vec3f(0, -0.1f, 0), Vec3f(0, -1, 0), Vec3f(1, 0, 0), 0.2f, Material(Vec3f(0.1f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    //Mesh cubeBackPlane = Plane(Vec3f(0, -0.2f, -0.1f), Vec3f(0, 0, 1), Vec3f(1, 0, 0), 0.2f, Material(Vec3f(0.1f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    //Mesh cubeFrontPlane = Plane(Vec3f(0, -0.2f, 0.1f), Vec3f(0, 0, 1), Vec3f(1, 0, 0), 0.2f, Material(Vec3f(0.1f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    //Mesh cubeLeftPlane = Plane(Vec3f(-0.1f, -0.2f, 0.f), Vec3f(1, 0, 0), Vec3f(0, 0, -1), 0.2f, Material(Vec3f(0.1f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    //Mesh cubeRightPlane = Plane(Vec3f(0.1f, -0.2f, 0.f), Vec3f(-1, 0, 0), Vec3f(0, 0, 1), 0.2f, Material(Vec3f(0.1f, 0.7f, 0.7f), 1.0f, 1.0f, 0.f));
    
    //CAMERA
    Camera cam(Vec3f(0, 0.0f, 0.75f), Vec3f(0, 0, 0.f), Vec3f(0, 1, 0), 45.f);
    //LIGHTS 
    lightPtr point = lightPtr(new PointLight(Vec3f(1, 1, 1), Vec3f(0.2f,0.f,1.f), 0.35f));
    lightPtr point2 = lightPtr(new PointLight(Vec3f(1, 1, 1), Vec3f(-0.2f,0.f,1.f), 0.35f));
    std::vector<lightPtr> lights{point, point2};
    //CREATE SCENES 
    Scene scene(cam, std::vector<Mesh> {backPlane, leftPlane, rightPlane, topPlane, plane, model}, lights);
    //Test Microbuffer
    //MicroBuffer microbuffer(24, Vec3f(0, 0, 0), Vec3f(0, 1, 0));
    //float sumAngle=0;
    //for (int i = 0; i < 24; i++)
    //{
    //    for (int j = 0; j < 24; j++)
    //    {
    //        std::cout << "direction (" << i << "," << j << ") : "<<microbuffer.pixelToDirection(i, j) << std::endl;
    //        float angle = microbuffer.solidAngle(i, j);
    //        sumAngle += angle;
    //        std::cout << " angle : " << angle << std::endl;
    //    }
    //}
    //std::cout << "sum angle : " << sumAngle << std::endl;
    //std::cout << "sphere half surface : " << 4*M_PI/2.f << std::endl;
    //RENDERING
    PointCloud pointCloud(15000.f);
    std::cout << "computing point cloud ... \n";
    pointCloud.computePointCloud(scene);
    std::cout << "done. \n";
    std::cout << "computing BVH for point cloud ... \n";
    pointCloud.computeBVH();
    std::cout << "done. \n";    
    std::cout << "Point Cloud composed of : " << pointCloud.surfels().size() << " surfels." << std::endl;
    //Rendering
    //Point based Rendering
    std::vector<Surfel> surfels;
    debugMicrobuffer(Vec3f(0.23f, 0.f, 0), Vec3f(-1, 0, 0), scene, pointCloud.BVHroot(), "mb_right.ppm", surfels);    
    //debugMicrobuffer(Vec3f(-0.23f, 0.f, 0), Vec3f(1, 0, 0), scene, pointCloud.BVHroot(), "mb_left.ppm", surfels);
    //debugMicrobuffer(Vec3f(0.0f, 0.23f, 0), Vec3f(0, -1, 0), scene, pointCloud.BVHroot(), "mb_top.ppm", surfels);
    //debugMicrobuffer(Vec3f(0.0f, -0.23f, 0), Vec3f(0, 1, 0), scene, pointCloud.BVHroot(), "mb_bot.ppm", surfels);
    //debugMicrobuffer(Vec3f(0.0f, 0, -0.23f), Vec3f(0, 0, 1), scene, pointCloud.BVHroot(), "mb_back.ppm", surfels);
    //PointBasedRenderer::render(scene, pointCloud, image, 8);
    PointCloud debugPointCloud(surfels);
    PointBasedRenderer::renderPointCloud(debugPointCloud,scene, image);
    //for (int i = 0; i < surfels.size(); i++)
    //{
    //    std::cout << "radius : " << surfels[i].radius << std::endl;
    //}
    //RayTracer::render(scene, image, 1);
    image.savePPM("PointBasedGI.ppm");
    return 0;
}

