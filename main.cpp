#include <iostream>
#include <chrono>
#include "image.h"
#include "scene.h"
#include "lightSource.h"
#include "rayTracer.h"
#include "pointCloud.h"
#include"pointBasedRenderer.h"
 
using namespace std;

int main(int argc, char* argv[])
{
    //std::cout << "Preparing display of Point Cloud ... \n";
    //std::vector<Vec3<Vec3f>> pos, norm;
    //pointCloud.triangleFromSurfels(pos, norm);
    ////render image
    //RayTracer rt;
    //std::cout << "Start rendering ... \n";
    //rt.render(pos, norm, scene.camera(), lights, ground.material(), sceneImage);
    //std::cout << "Done. \n";
    //std::cout << "Saving image ... \n";
    //sceneImage.savePPM("PointCloudDebug.ppm");
    //std::cout << "Done. \n";
    size_t width=300, height=200;
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
    Image sceneImage(width, height);
    //create scene components
    //MESHES
    //load model
    Mesh model(Material(Vec3f(0.5f, 0.5f, 0.5f), 0.7f, 0.5f, 0.3f));
    model.loadOFF("example_lowres.off");    
    //ground
    Mesh ground(Material(Vec3f(0.5f, 0.5f, 0.5f), 1.f, 1.f, 0.0f));
    float yvalue = model.boundingBox().min()[1];
    ground.vertices() = std::vector<Vec3f>{ Vec3f(-2,yvalue,-2), Vec3f(-2,yvalue,2), Vec3f(2,yvalue,2), Vec3f(2,yvalue,-2) };
    ground.indices() = std::vector<Vec3i>{ Vec3i(0,1,2), Vec3i(0,2,3) };
    ground.normals() = std::vector<Vec3f>{ Vec3f(0,1,0), Vec3f(0,1,0), Vec3f(0,1,0), Vec3f(0,1,0) }; 
    //CAMERA
    Camera cam(Vec3f(0, 5, 30.f), Vec3f(0, 0, 0), Vec3f(0, 1, 0), (float)width / (float)height, 90.f);
    //LIGHTS 
    lightPtr point = lightPtr(new PointLight(Vec3f(1, 1, 1), Vec3f(1,1,1), 0.6f));
    lightPtr point2 = lightPtr(new PointLight(Vec3f(1, 1, 1), Vec3f(5,1,1), 0.4f));
    std::vector<lightPtr> lights{point};
    //CREATE SCENES  
    Scene scene(cam, std::vector<Mesh> {ground, model}, lights);
    //////////////////////////////////////////////////////////////////
    PointCloud pointCloud(1500.f);
    std::cout << "computing point cloud ... \n";
    pointCloud.computePointCloud(scene);
    std::cout << "done. \n";
    std::cout << "computing BVH for point cloud ... \n";
    pointCloud.computeBVH();
    std::cout << "done. \n";    
    std::cout << "Point Cloud composed of : " << pointCloud.surfels().size() << " surfels." << std::endl;
    //testing microbuffer
    MicroBuffer testMB(8, Vec3f(0, 0, 0), Vec3f(0, 1, 0));
    int i, j;
    testMB.positionToPixel(Vec3f(0.5f, 0, 0.5f), i, j);
    std::cout << " i :" << i << ", j : " << j << std::endl;
    testMB.positionToPixel(Vec3f(-0.5f, 0, -0.5f), i, j);
    std::cout << " i :" << i << ", j : " << j << std::endl;
    testMB.directionToPixel(Vec3f(10, 1, 1),i,j);
    std::cout << " i :" << i << ", j : " << j << std::endl;
    //Point based Rendering
    PointBasedRenderer renderer;
    renderer.render(scene, pointCloud, sceneImage, 8);
    return 0;
}

