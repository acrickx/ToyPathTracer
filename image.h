#pragma once

#include<vector>
#include"Vec3.h"
#include"lodepng.h"
#include<fstream>
#include<string>
#include <omp.h>

class Image
{
	private:
		size_t width =0;
		size_t height = 0;
		std::vector<Vec3f> colors;

	public:
		Image(size_t _width, size_t _height);
		Image(Vec3f colorValue, size_t _width, size_t _height);
		size_t getWidth() const { return width; }
		inline size_t getHeight() const { return height; }
		std::vector<Vec3f> getColorVector() { return colors; }
		void savePPM(std::string filename);
		void savePNG(const char* file_path);
		void fillBackground(Vec3f from, Vec3f to);
		void setColors(std::vector<Vec3f> colors);
		void setColorValue(size_t i, size_t j, Vec3f colorValue);
		inline const Vec3f& operator() (size_t i, size_t j) const { return colors[j * width + i]; }
		inline Vec3f& operator() (size_t i, size_t j) { return colors[j * width + i]; }
};

