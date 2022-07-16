#pragma once

#include"Vec3.h"
#include"ray.h"
class Camera
{
	public:
		inline Camera(const Vec3f& position, const Vec3f& lookAt, const Vec3f& up, float verticalFOV = 60.f, float aspectRatio = 1.f)
		{
			m_position = position;
			m_verticalFOV = verticalFOV;
			m_aspectRatio = aspectRatio;
			float angle = verticalFOV * M_PI / 180.f;
			float halfHeight = tan(angle / 2.f);			
			float halfWidth = m_aspectRatio * halfHeight;
			m_forward = normalize(position - lookAt);
			Vec3f u = normalize(cross(up, m_forward));
			Vec3f v = cross(m_forward, u);
			m_bottomLeftCorner = m_position - m_forward - halfWidth * u - halfHeight * v;
			m_horizontal = 2.f * halfWidth * u;			
			m_vertical = 2.f * halfHeight * v;	

		}
		inline const Vec3f getImageCoordinate(float i, float j) const { return m_bottomLeftCorner - m_position + (float)i * m_horizontal + (float)j * m_vertical;}
		inline const Vec3f getPosition() const { return m_position; }
		inline const Vec3f getBLcorner() const { return m_bottomLeftCorner; }		
		inline const Ray rayAt(float u, float v) const { return Ray(m_position, normalize(m_bottomLeftCorner + u * m_horizontal + v * m_vertical - m_position));  }

	private:
		Vec3f m_position;
		Vec3f m_forward;
		Vec3f m_horizontal;	
		Vec3f m_vertical;
		float m_aspectRatio;
		float m_verticalFOV;	
		Vec3f m_bottomLeftCorner;		
};

