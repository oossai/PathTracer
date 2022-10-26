#pragma once

#include <Pch.h>
#include <Image.h>
#include <Ray.h>

namespace PathTracer
{
	struct CamOptions
	{
		Eigen::Vector3f LookFrom;
		Eigen::Vector3f LookAt;
		Eigen::Vector3f Up;
		Eigen::Vector2i Resolution;
		Float FOVDegrees;
	};
	
	class Camera
	{
	public:
		Eigen::Vector2i GetImageResolution() const
		{
			return mResolution;
		}

		void SetFieldOfView(Float FOVDegrees)
		{
			mFieldOfView = std::tanf((FOVDegrees * kPiDiv180 / 2));
		}

		Camera() = default;
		Camera(const CamOptions& Options);

		Camera(const Camera&) = delete;
		Camera& operator=(const Camera&) = delete;

		Camera(Camera&&) = default;
		Camera& operator=(Camera&&) = default;

		~Camera() = default;

		void WriteImageToPPM(std::string_view filename);
		
		void SetPixelColour(int Row, int Col, const Eigen::Vector3f& RGB);
		
		Ray GenerateRay(int Row, int Col, const Eigen::Vector2f& SamplePoint);
	private:
		Eigen::Matrix4f mCameraToWorld;
		Image mImage;
		Eigen::Vector3f mOrigin;
		Eigen::Vector2i mResolution;
		Eigen::Vector2f mInvResolution;
		Float mImageAspectRatio;
		Float mFieldOfView;
	};

} // namespace PathTracer
