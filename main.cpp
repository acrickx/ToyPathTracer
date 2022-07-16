#include <iostream>
#include <chrono>
#include "image.h"
#include "scene.h"
#include "lightSource.h"
#include "rayTracer.h"
#include "pointCloud.h"
#include "pointBasedRenderer.h"
 
using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


//MAIN LOOP
int main(int argc, char* argv[])
{
    float aspectRatio = 1.f;
    size_t microBufferSize = 8;
    size_t rayPerPixel = 1;
    size_t width = 720, height = 720;
    string filename="output.png";
    //"USAGE : ./MyRayTracer –width value -height value -output value -microbuffer value -rayperpixel value //
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
            else if (std::string(argv[i]) == "-microbuffer")
            {
                microBufferSize = std::stoi(argv[i + 1]);
                std::cout << "buffer size : " << microBufferSize << std::endl;
            }
            else if (std::string(argv[i]) == "-rayperpixel")
            {
                rayPerPixel = std::stoi(argv[i + 1]);
                std::cout << "ray per pixel : " << rayPerPixel << std::endl;
            }
        }
    }
    Image image(width, height);
    //MESHES
    Mesh plane = Plane(Vec3f(0, -0.5f, 0), Vec3f(0, 1, 0), Vec3f(1, 0, 0), 1.f, Material(Vec3f(1.f, 1.f, 1.f), Diffuse));
    Mesh topPlane = Plane(Vec3f(0, 0.5f, 0), Vec3f(0, -1, 0), Vec3f(1, 0, 0), 1.f, Material(Vec3f(1.f, 1.f, 1.f), Emissive, Vec3f(1.f,1.f,1.f)));
    Mesh backPlane = Plane(Vec3f(0, 0, -0.5f), Vec3f(0, 0, 1), Vec3f(1, 0, 0), 1.f , Material(Vec3f(1.f, 1.f, 1.f), Diffuse));
    Mesh leftPlane = Plane(Vec3f(-0.5f, 0, 0.f), Vec3f(1, 0, 0), Vec3f(0, 0, -1), 1.f, Material(Vec3f(1.f, 0.0f, 0.0f), Diffuse));
    Mesh rightPlane = Plane(Vec3f(0.5f, 0, 0.f), Vec3f(-1, 0, 0), Vec3f(0, 0, 1), 1.f, Material(Vec3f(0.f, 1.f, 0.f), Diffuse));
    Mesh model(Material(Vec3f(1.f, 1.f, 1.f), Diffuse));
    model.loadOFF("example_lowres.off");
    //CAMERA
    Camera cam(Vec3f(0.f, 0.f, 1.0f), Vec3f(0, 0, 0.f), Vec3f(0, 1, 0), 90.f, 1.0f);
    //LIGHTS 
    lightPtr point = lightPtr(new PointLight(Vec3f(1, 1, 1), Vec3f(0.2f,0.0f,1.f), 2.0f));
    lightPtr point2 = lightPtr(new PointLight(Vec3f(1, 1, 1), Vec3f(-0.2f,0.0f,1.f), 4.0f));
    lightPtr area = lightPtr(new AreaLight(Vec3f(1,1,1), Vec3f(0, 0.4f, 0), 30.f, 0.4f, Vec3f(0,0,0)));
    std::vector<lightPtr> lights{area};
    //CREATE SCENE
    Scene scene(cam, std::vector<Mesh> {backPlane, leftPlane, rightPlane, topPlane, plane,model}, lights);    
    std::cout << "Computing BVH for raytracing ... \n";
    auto t1 = high_resolution_clock::now();
    scene.computeBVH();
    auto t2 = high_resolution_clock::now();
    std::cout << "Done.  \n";
    auto chrono = duration_cast<milliseconds>(t2 - t1);
    std::cout << "BVH computation : " << chrono.count() * 0.001f << "s." << std::endl;
    //RENDERING
    t1 = high_resolution_clock::now();
    std::cout << "Starting Rendering ...";
    RayTracer::render(scene, image, rayPerPixel);    
    std::cout << "Done. \n";    
    t2 = high_resolution_clock::now();
    chrono = duration_cast<milliseconds>(t2 - t1);
    std::cout << "\nRendering : " << chrono.count() * 0.001f  << "s." << std::endl;    
    image.savePNG("test.png");
    return 0;
}

