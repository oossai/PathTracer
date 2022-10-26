#pragma once

#include <Pch.h>

namespace PathTracer
{
	struct Pixel
	{
		Float RGB[3];
	};
	
	struct Image
	{
	public:
		Image() = default;
		
		Image(unsigned width, unsigned height);

		Image(const Image&) = delete;
		Image& operator=(const Image&) = delete;

		Image(Image&&) = default;
		Image& operator=(Image&&) = default;

		~Image() = default;

		void WriteImageToPPM(std::string_view filename);

	public:
		std::vector<Pixel> mImageData;
		unsigned mImageWidth, mImageHeight;
	};

} // namespace PathTracer

