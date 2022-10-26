#include <Scene.h>
#include <Camera.h>
#include <Sampler.h>

using namespace PathTracer;
using namespace Eigen;

void Render(Camera& Camera, Scene& Scene)
{
	const Vector2i CamResolution = Camera.GetImageResolution();

    const Float InvCamY = 1.f / (CamResolution.y() - 1);

	const unsigned nSamples = 16;

	HammersleySampler Sampler(nSamples);

	std::cout << "\nStarting Rendering\n";

	for (int Row = 0; Row < CamResolution.y(); Row++)
	{
		for (int Col = 0; Col < CamResolution.x(); Col++)
		{
			Vector3f PixelColor(0, 0, 0);

			for (unsigned N = 0; N < nSamples; N++)
			{
				Vector2f CameraSample = Sampler.SampleUnitSquare();

				const Ray aRay = Camera.GenerateRay(Row, Col, CameraSample);

				Intersection HitResult;

				if (Scene.Intersect(aRay, HitResult))
				{
					PixelColor += HitResult.Normal;
				}
			}

			PixelColor /= static_cast<Float>(nSamples);
			
			Camera.SetPixelColour(Row, Col, PixelColor);
		}

		std::cout << "\r( Rendering " << unsigned(Row * InvCamY * 100.0F) << " % Completed )";
	}

	std::cout << "\n";
	
	Camera.WriteImageToPPM("Image");
}

int main(int argc, char** argv)
{
	CamOptions Options;
	Options.LookFrom = Vector3f(3, 2, 5);
	Options.LookAt = Vector3f(0, 2, -1);
	Options.Up = Vector3f(0, 1, 0);
	Options.Resolution = Vector2i(500, 500);
    Options.FOVDegrees = 45;

	Camera NewCamera(Options);

    Scene Cube(R"(..\..\Models\Cube.obj)");

	Render(NewCamera, Cube);

	return 0;
}
