#include "image.h"

//CONSTRUCTORS

Image::Image(size_t _width, size_t _height)
{
	width = _width;
	height = _height;
	colors = std::vector<Vec3f>(width * height);
}

//SETTERS

void Image::setColors(std::vector<Vec3f> newColors)
{
    colors = newColors;
}

void Image::setColorValue(size_t i, size_t j, Vec3f newColorValue)
{
    colors[i * width + j] = newColorValue;
}

//IMAGE FUNCTIONS

Image::Image(Vec3f colorValue, size_t _width=64, size_t _height=64)
{
	width = _width;
	height = _height;
	colors = std::vector<Vec3f>(width * height, colorValue);
}

void Image::savePPM(std::string filename)
{
    std::ofstream ofs(filename);

    ofs << "P3\n" << width << " " << height << "\n255\n";
    for (int j = 0; j < height; j++)
    {   
        #pragma omp parallel for
        for (int i = 0; i < width; i++)
        {
            Vec3f rgb = 255.99f * colors[j*width+i];
            ofs << int(rgb[0]) << " " << int(rgb[1]) << " " << int(rgb[2]) << "\n";
        }
    }
    std::cout << "-Done saving image" << std::endl;        
    ofs.close();
}

//Save in PNG format (james griffin implementation)
inline double clamp(double x) { return x < 0 ? 0 : x>1 ? 1 : x; }
inline int toInt(double x) { return int(clamp(x) * 255 + .5); }

//using external library
void Image::savePNG(const char* file_path)
{
    std::vector<unsigned char> pixel_buffer;
    int pixel_count = width * height;

    for (int i = 0; i < pixel_count; i++) {
        pixel_buffer.push_back(toInt(colors[i][0]));
        pixel_buffer.push_back(toInt(colors[i][1]));
        pixel_buffer.push_back(toInt(colors[i][2]));
        pixel_buffer.push_back(255);
    }

    //Encode the image
    unsigned error = lodepng::encode(file_path, pixel_buffer, width, height);
    //if there's an error, display it
    if (error) std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;

    pixel_buffer.clear();
}

void Image::fillBackground(Vec3f from, Vec3f to)
{
    for (int j = 0; j < height; j++)
    {
        #pragma omp parallel for
        for (int i = 0; i < width; i++)
        {
            colors[j * width + i] = (float)((height - j)/(float)height) * from + (float)(j/(float)height) * to;
        }
    }
}