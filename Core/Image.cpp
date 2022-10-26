#include <Image.h>

namespace PathTracer
{
	Image::Image(unsigned Width, unsigned Height)
	: mImageWidth{Width}, mImageHeight{Height}
	{
		mImageData.resize(mImageWidth * mImageHeight);
	}

	void Image::WriteImageToPPM(std::string_view filename)
	{
		using namespace std::string_literals;

		if (mImageWidth == 0 || mImageHeight == 0)
		{
			throw std::logic_error("Cannot save image with 0 dimension\n");
		}

		std::ofstream Output { filename.data() + ".ppm"s };

		if (!Output.is_open())
		{
			throw std::runtime_error("Failed to open file output\n");
		}

		Output << "P3\n" << mImageWidth << " " << mImageHeight << "\n255\n";

		float GammaCorrection = 1.f / 2.2f;

		std::cout << "Saving File...\n";

		for (size_t i = 0; i < mImageData.size(); i++)
		{
			int Red   = static_cast<int>(std::pow(std::clamp(mImageData[i].RGB[0], 0.f, 1.f), GammaCorrection) * 255);
			int Green = static_cast<int>(std::pow(std::clamp(mImageData[i].RGB[1], 0.f, 1.f), GammaCorrection) * 255);
			int Blue  = static_cast<int>(std::pow(std::clamp(mImageData[i].RGB[2], 0.f, 1.f), GammaCorrection) * 255);

			Output << Red << " " << Green << " "<< Blue << "\n";
		}

		Output.close();

		std::cout << "Saving Completed!\n";
	}

} // namespace PathTracer