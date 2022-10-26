#pragma once

#include <Sampler.h>

using namespace Eigen;

namespace PathTracer
{
    ISampler::ISampler(unsigned NumSamples, unsigned NumSets) 
    : mRng{std::random_device()()}, mDistrib{0.f, 1.f},
        nCountSquare{0}, nSamples{NumSamples}, nSets{NumSets}
    {
        GetRandomFloat01 = std::bind(mDistrib, mRng);

		mSamples.reserve(nSets * nSamples);

		mShuffledIndices.resize(nSamples * nSets);
		
		for (size_t i = 0; i < mShuffledIndices.size();)
		{
			for (unsigned j = 0; j < nSamples; j++)
			{
				mShuffledIndices[i + j] = j;
			}
			
			i += nSamples;
		}
		
		for (size_t i = 0; i < nSamples * nSets; i += nSamples)
		{
			std::shuffle(mShuffledIndices.begin() + i, mShuffledIndices.begin() + i + nSamples, mRng);
		}
    }
    
    Eigen::Vector3f ISampler::InternalSampleHemisphere(Float CosTheta, Float Phi)
    {
        Float SinTheta = std::sqrtf(1 - CosTheta * CosTheta);

        return Eigen::Vector3f{SinTheta * std::cosf(Phi), CosTheta, SinTheta * std::sinf(Phi)};
    }

    // Random
    Vector2f RandomSampler::SampleUnitSquare()
    {
        return Vector2f{GetRandomFloat01(), GetRandomFloat01()};
    }

    Vector2f RandomSampler::SampleUnitDisk()
    {
        Float Radius = std::sqrtf(GetRandomFloat01());
        Float Theta = k2Pi * GetRandomFloat01();

        return Vector2f{Radius * std::cosf(Theta), Radius * std::sinf(Theta)};
    }

    Vector3f RandomSampler::SampleHemisphere(SamplingStrategy Strategy, Float DensityPower)
    {
        switch (Strategy)
        {
        case SamplingStrategy::Uniform:
            {
                Float CosTheta = GetRandomFloat01();
                Float Phi = k2Pi * GetRandomFloat01();

                return InternalSampleHemisphere(CosTheta, Phi);
            }
            break;

        case SamplingStrategy::CosineWeighted:
            {
                Float CosTheta = std::powf(GetRandomFloat01(), 1 / (DensityPower + 1));
                Float Phi = k2Pi * GetRandomFloat01();

                return InternalSampleHemisphere(CosTheta, Phi);
            }
            break;

        default:
            throw std::invalid_argument("Unsupported strategy");

            break;
        }
    }

    // MultiJitteredSampler
    CMJSampler::CMJSampler(unsigned NumSamples, unsigned NumSets) 
    : ISampler(NumSamples, NumSets)
    {
        GenerateSamples();
    }

    void CMJSampler::GenerateSamples()
	{
		unsigned n = static_cast<unsigned>(std::sqrtf(static_cast<Float>(nSamples)));

        Float InvN = 1.f / n;

		for (unsigned p = 0; p < nSets; p++)
		{
            unsigned SampleSet = p * nSamples;

            for (unsigned Row = 0; Row < n; Row++)
            {
                for (unsigned Col = 0; Col < n; Col++)
                {
                    mSamples[(Row * n + Col) + SampleSet].x() = (Col + (Row + GetRandomFloat01()) * InvN) * InvN;
                    mSamples[(Row * n + Col) + SampleSet].y() = (Row + (Col + GetRandomFloat01()) * InvN) * InvN;
                }
            }
        }

		for (unsigned p = 0; p < nSets; p++)
		{
            unsigned SampleSet = p * nSamples;

            for (unsigned Row = 0; Row < n; Row++)
            {
                unsigned k = unsigned(Row + GetRandomFloat01() * (n - Row));

                for (unsigned Col = 0; Col < n; Col++)
                {
                    

                    std::swap(mSamples[(Row * n + Col) + SampleSet].x(), mSamples[(k * n + Col) + SampleSet].x());
                }
            }
        }

		for (unsigned p = 0; p < nSets; p++)
		{
            unsigned SampleSet = p * nSamples;
            
            for (unsigned Col = 0; Col < n; Col++)
            {
                unsigned k = unsigned(Col + GetRandomFloat01() * (n - Col));

                for (unsigned Row = 0; Row < n; Row++)
                {
                   

                    std::swap(mSamples[(Row * n + Col) + SampleSet].y(), mSamples[(Row * n + k) + SampleSet].y());
                }
            }
        }

    }

    Vector2f CMJSampler::SampleUnitSquare()
    {
		if (nCountSquare % nSamples == 0)
		{
			mJumpSquare = (mRng() % nSets) * nSamples;
		}
		
		return mSamples[mJumpSquare + mShuffledIndices[mJumpSquare + nCountSquare++ % nSamples]];
    }

    Vector2f CMJSampler::SampleUnitDisk()
    {
        return Vector2f{};
    }

    Vector3f CMJSampler::SampleHemisphere(SamplingStrategy Strategy, Float DensityPower)
    {
        return Vector3f{};
    }

    // HammersleySampler
    HammersleySampler::HammersleySampler(unsigned NumSamples, unsigned NumSets) 
    : ISampler(NumSamples, NumSets)
    {
        GenerateSamples();
    }

    void HammersleySampler::GenerateSamples()
	{
		Float OneDivSamples = 1.f / nSamples;

		for (unsigned s = 0; s < nSets; s++)
		{
			for (unsigned i = 0; i < nSamples; i++)
			{
				mSamples.push_back(Vector2f((i + 0.5f) * OneDivSamples, Reverse64bit(i) * k1Div2PowNeg64));
			}
		}
	}

    Vector2f HammersleySampler::SampleUnitSquare()
    {
		if (nCountSquare % nSamples == 0)
		{
			mJumpSquare = (mRng() % nSets) * nSamples;
		}
		
		return mSamples[mJumpSquare + mShuffledIndices[mJumpSquare + nCountSquare++ % nSamples]];
    }

    Vector2f HammersleySampler::SampleUnitDisk()
    {
        return Vector2f{};
    }

    Vector3f HammersleySampler::SampleHemisphere(SamplingStrategy Strategy, Float DensityPower)
    {
        return Vector3f{};
    }

	uint32_t Reverse32bit(uint32_t Number)
	{
        Number = (Number >> 1)  & 0x55555555 | (Number << 1)  & 0xaaaaaaaa;
        Number = (Number >> 2)  & 0x33333333 | (Number << 2)  & 0xcccccccc;
        Number = (Number >> 4)  & 0x0f0f0f0f | (Number << 4)  & 0xf0f0f0f0;
        Number = (Number >> 8)  & 0x00ff00ff | (Number << 8)  & 0xff00ff00;
        Number = (Number >> 16) & 0x0000ffff | (Number << 16) & 0xffff0000;
        
		return Number;
    }

	uint64_t Reverse64bit(uint64_t Number)
	{
		uint64_t Low  = Reverse32bit(static_cast<uint32_t>(Number));
		uint64_t High = Reverse32bit(static_cast<uint32_t>(Number >> 32));

		return (Low << 32) | High;
	}

} // namespace PathTracer