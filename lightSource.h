#pragma once
#include"Vec3.h"

class LightSource
{
	protected:
		Vec3f m_position;
		Vec3f m_color;
		float m_intensity;
		LightSource(Vec3f _color, Vec3f _position, float _intensity) : m_color(_color), m_position(_position), m_intensity(_intensity) {};
		LightSource() : m_color(Vec3f(1, 1, 1)), m_position(Vec3f(0, 0, 0)), m_intensity(1) {};
	public:
		inline virtual Vec3f colorResponse() const = 0;
		inline virtual const Vec3f getPosition() const = 0;
};

using lightPtr = std::shared_ptr<LightSource>;
typedef std::shared_ptr<LightSource> lightPtr;

class PointLight : public LightSource
{
	public:
		PointLight(Vec3f _color, Vec3f _position, float _intensity) : LightSource(_color, _position, _intensity) {}
		PointLight() : LightSource() {}
		inline Vec3f colorResponse() const { return m_color * m_intensity; };
		inline const Vec3f getPosition() const { return m_position; }
};

class AreaLight : public LightSource
{
	protected:
		float m_sideLength;
		Vec3f m_lookAt;
	public:
		AreaLight(Vec3f _color, Vec3f _position, float _intensity, float _sideLength, Vec3f _lookAt) : LightSource(_color, _position, _intensity), m_sideLength(_sideLength), m_lookAt(_lookAt) {}
		AreaLight(float _sideLength, Vec3f _lookAt) : LightSource(), m_sideLength(_sideLength), m_lookAt(_lookAt) {}
		AreaLight() : LightSource(), m_sideLength(0.2f), m_lookAt(Vec3f(0,0,0)) {}
		inline Vec3f colorResponse() const { return m_color * m_intensity; }
		inline const Vec3f getPosition() const { return m_position; }
		inline const Vec3f getRandomPoint() const { return m_position + m_sideLength * Vec3f(rand() % 100 / (float)100 - 1, rand() % 100 / (float)100 - 1, 0); }
};


