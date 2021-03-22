#pragma once

#include"Vec3.h"
class Camera
{
	public:
		inline Camera(Vec3f position, Vec3f lookAt, Vec3f up, float verticalFOV)
		{
			m_position = position;
			m_aspectRatio = 1.f;
			float viewportHeight = 2.0f;
			float viewportWidth = m_aspectRatio * viewportHeight;

			m_verticalFOV = verticalFOV;
			float halfHeight = 2* tan(verticalFOV*M_PI*0.5f / 180.f);			
			float halfWidth = m_aspectRatio * halfHeight;
			m_forward = normalize(m_position - lookAt);
			Vec3f u = normalize(cross(up, m_forward));
			Vec3f v = cross(m_forward, u);
			m_bottomLeftCorner = m_position - m_forward - halfWidth * u - halfHeight * v;
			m_horizontal = 2.f*halfWidth*u;			
			m_vertical = 2.f*halfHeight*v;	

		}
		inline const Vec3f getImageCoordinate(float i, float j) const { return m_bottomLeftCorner - m_position + (float)i * m_horizontal + (float)j * m_vertical;}
		inline const Vec3f getPosition() const { return m_position; }
		inline const Vec3f getBLcorner() const { return m_bottomLeftCorner; }		
	private:
		//need to be specified
		Vec3f m_position;
		Vec3f m_forward;
		Vec3f m_horizontal;	
		Vec3f m_vertical;
		float m_aspectRatio;
		float m_verticalFOV;		
		Vec3f m_bottomLeftCorner;		
};

