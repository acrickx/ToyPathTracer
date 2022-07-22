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
    // PARAMETERS
    float aspectRatio = 1.f;
    size_t microBufferSize = 8;
    size_t rayPerPixel = 8;
    size_t width = 700, height = 700;
    string filename="output.png";
    Image image(width, height);

    // CONSOLE USAGE : ./MyRayTracer –width value -height value -output value -microbuffer value -rayperpixel value
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

    // MATERIALS
    MaterialPtr white = MaterialPtr(new MaterialGGX(Vec3f(1.f, 1.f, 1.f)));
    MaterialPtr red = MaterialPtr(new MaterialGGX(Vec3f(0.8f, 0.f, 0.f)));
    MaterialPtr blue = MaterialPtr(new MaterialGGX(Vec3f(0.f, 0.f, 8.f)));
    MaterialPtr light = MaterialPtr(new MaterialEmissive(Vec3f(1.0f, 1.0f, 1.0f), 2.0f));

    // MESHES
    Mesh plane = Plane(Vec3f(0, -0.5f, 0), Vec3f(0, 1, 0), Vec3f(1, 0, 0), 1.f, white);
    Mesh topPlane = Plane(Vec3f(0, 0.5f, 0), Vec3f(0, -1, 0), Vec3f(1, 0, 0), 1.01f, light);
    Mesh backPlane = Plane(Vec3f(0, 0, -0.5f), Vec3f(0, 0, 1), Vec3f(1, 0, 0), 1.f , white);
    Mesh leftPlane = Plane(Vec3f(-0.5f, 0, 0.f), Vec3f(1, 0, 0), Vec3f(0, 0, -1), 1.f, red);
    Mesh rightPlane = Plane(Vec3f(0.5f, 0, 0.f), Vec3f(-1, 0, 0), Vec3f(0, 0, 1), 1.f, red);
    Mesh lightPlane = Plane(Vec3f(0, 0.45f, 0), Vec3f(0, -1, 0), Vec3f(1, 0, 0), 0.2f, light);

    // Loading model
    MaterialPtr purple = MaterialPtr(new MaterialGGX(Vec3f(1.0f, 0.5f, 1.f)));
    Mesh model(purple);
    model.loadOBJ("cow.obj");
    model.scale(0.6f);
    model.translate(Vec3f(0.f, -0.35f, 0.f));
    model.computeNormals();
    std::vector<Mesh> meshes{ backPlane, leftPlane, rightPlane, topPlane, plane, model};

    // CAMERA
    Camera cam(Vec3f(0.f, 0.f, 1.2f), Vec3f(0, 0, 0.f), Vec3f(0, 1, 0), 60.f, 1.0f);

    // LIGHTS 
    lightPtr point = lightPtr(new PointLight(Vec3f(1, 1, 1), Vec3f(0.2f,0.0f,1.f), 2.0f));
    std::vector<lightPtr> lights{};

    // CREATE SCENE
    Scene scene(cam, meshes, lights);
    std::cout << "Computing BVH for raytracing ... \n";
    auto t1 = high_resolution_clock::now();
    scene.computeBVH();
    auto t2 = high_resolution_clock::now();
    std::cout << "Done.  \n";
    auto chrono = duration_cast<milliseconds>(t2 - t1);
    std::cout << "BVH computation : " << chrono.count() * 0.001f << "s." << std::endl;

    // RENDERING
    t1 = high_resolution_clock::now();
    std::cout << "Starting Rendering ...";
    RayTracer::render(scene, image, rayPerPixel, 1);    
    std::cout << "Done. \n";    
    t2 = high_resolution_clock::now();
    chrono = duration_cast<milliseconds>(t2 - t1);
    std::cout << "\nRendering : " << chrono.count() * 0.001f  << "s." << std::endl;    
    image.savePNG("test.png");
    return 0;
}

