#include <Camera.h>

using namespace Eigen;

namespace PathTracer
{
	Camera::Camera(const CamOptions& Options)
	{
		// compute basis
		Vector3f W = (Options.LookFrom - Options.LookAt).normalized();
		Vector3f U = (Options.Up.cross(W)).normalized();
		Vector3f V = W.cross(U).normalized();

		// transformation
		mCameraToWorld(0, 0) = U.x();
		mCameraToWorld(0, 1) = V.x();
		mCameraToWorld(0, 2) = W.x();
		mCameraToWorld(0, 3) = Options.LookFrom.x();

		mCameraToWorld(1, 0) = U.y();
		mCameraToWorld(1, 1) = V.y();
		mCameraToWorld(1, 2) = W.y();
		mCameraToWorld(1, 3) = Options.LookFrom.y();

		mCameraToWorld(2, 0) = U.z();
		mCameraToWorld(2, 1) = V.z();
		mCameraToWorld(2, 2) = W.z();
		mCameraToWorld(2, 3) = Options.LookFrom.z();

		mCameraToWorld(3, 0) = 0;
		mCameraToWorld(3, 1) = 0;
		mCameraToWorld(3, 2) = 0;
		mCameraToWorld(3, 3) = 0;

		// image
		mImage = Image(Options.Resolution.x(), Options.Resolution.y());

		mImageAspectRatio = Options.Resolution.x() / Float(Options.Resolution.y());

		SetFieldOfView(Options.FOVDegrees);

		mOrigin = Options.LookFrom;

		mResolution = Options.Resolution;

		mInvResolution.x() = 1.f / mResolution.x();
		mInvResolution.y() = 1.f / mResolution.y();
	}

	Ray Camera::GenerateRay(int Row, int Col, const Eigen::Vector2f& SamplePoint)
	{
		const Float PixelXNdc = (Col + SamplePoint.x()) * mInvResolution.x();
		const Float PixelYNdc = (Row + SamplePoint.y()) * mInvResolution.y();

		const Float PixelXScreen = ( 2 * PixelXNdc - 1) * mFieldOfView * mImageAspectRatio;
		const Float PixelYScreen = (-2 * PixelYNdc + 1) * mFieldOfView;
		
		const Vector4f PixelWorld = mCameraToWorld * Vector4f(PixelXScreen, PixelYScreen, -1, 1);

		Vector3f Direction = Vector3f(PixelWorld.x(), PixelWorld.y(), PixelWorld.z());

		Direction = (Direction - mOrigin).normalized();
		
		return Ray{mOrigin, Direction};
	}

	void Camera::SetPixelColour(int Row, int Col, const Vector3f& RGB)
	{
		int PixelIndex = Row * mResolution.x() + Col;
		
		mImage.mImageData[PixelIndex].RGB[0] = RGB[0];
		mImage.mImageData[PixelIndex].RGB[1] = RGB[1];
		mImage.mImageData[PixelIndex].RGB[2] = RGB[2];
	}

	void Camera::WriteImageToPPM(std::string_view filename)
	{
		mImage.WriteImageToPPM(filename);
	}

} // namespace PathTracer
