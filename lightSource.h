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
		Vec3f m_bottomLeftCorner;
		Vec3f m_horizontal;
		Vec3f m_vertical;
	public:
		AreaLight(Vec3f color, Vec3f position, float intensity, float sideLength, Vec3f lookAt)
		{
			m_color = color;
			m_intensity = intensity;
			m_position = position;
			m_sideLength = sideLength;
			m_lookAt = lookAt;
			Vec3f normal = normalize(m_lookAt - m_position);
			std::cout << "n :" << normal << std::endl;
			if (normal == Vec3f(0, 1, 0) || normal == -Vec3f(0,1,0))
			{
				m_horizontal = cross(normal, Vec3f(1, 0, 0));
			}
			else m_horizontal = cross(normal, Vec3f(0, 1, 0));
			m_vertical = cross(normal, m_horizontal);
			m_bottomLeftCorner = m_position - m_sideLength*0.5f*(m_horizontal + m_vertical);
		}
		AreaLight(Vec3f position,float sideLength)
		{
			m_color = Vec3f(1,1,1);
			m_intensity = 1.f;
			m_lookAt = Vec3f();
			m_position = position;
			m_sideLength = sideLength;
			Vec3f normal = normalize(m_lookAt - m_position);
			if (normal == Vec3f(0, 1, 0) || normal == -Vec3f(0, 1, 0))
			{
				m_horizontal = cross(normal, Vec3f(1, 0, 0));
			}
			else m_horizontal = cross(normal, Vec3f(0, 1, 0));
			m_vertical = cross(normal, m_horizontal);
			m_bottomLeftCorner = m_position - m_sideLength * 0.5f * (m_horizontal + m_vertical);
		}
		inline const Vec3f getPosition() const
		{
			float rd1 = (double)rand() / RAND_MAX;
			float rd2 = (double)rand() / RAND_MAX;
			return m_bottomLeftCorner + (rd1 * m_horizontal + rd2 * m_vertical)*m_sideLength;
		}
		inline Vec3f colorResponse() const { return m_color * m_intensity; };
};


