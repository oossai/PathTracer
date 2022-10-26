#pragma once

#include <Pch.h>
#include <Constants.h>

namespace PathTracer
{
	enum class SamplingStrategy
	{
		Uniform = 1,
		CosineWeighted = 2,
	};

    class ISampler
    {
    public:
        ISampler(unsigned NumSamples = 0, unsigned NumSets = 0);

		ISampler(const ISampler&) = delete;
		ISampler& operator=(const ISampler&) = delete;

		ISampler(ISampler&&) = default;
		ISampler& operator=(ISampler&&) = default;

        virtual ~ISampler() = default;

        virtual Eigen::Vector2f SampleUnitSquare() = 0;
        virtual Eigen::Vector2f SampleUnitDisk() = 0;
        virtual Eigen::Vector3f SampleHemisphere(SamplingStrategy Strategy, Float DensityPower = 0) = 0;
    protected:
        Eigen::Vector3f InternalSampleHemisphere(Float CosTheta, Float Phi);

        std::function<Float()> GetRandomFloat01;
        std::mt19937 mRng;
        std::uniform_real_distribution<Float> mDistrib;

        std::vector<Eigen::Vector2f> mSamples;
        std::vector<unsigned> mShuffledIndices;
		unsigned nCountSquare, mJumpSquare, nSamples, nSets;
    };
    
    class RandomSampler : public ISampler
    {
    public:
        RandomSampler() = default;
        
        Eigen::Vector2f SampleUnitSquare() override;
        Eigen::Vector2f SampleUnitDisk() override;
        Eigen::Vector3f SampleHemisphere(SamplingStrategy Strategy, Float DensityPower = 0) override;

        ~RandomSampler() = default;
    };
    
    class CMJSampler : public ISampler
    {
    public:
        CMJSampler(unsigned NumSamples, unsigned NumSets = 97);
        
        Eigen::Vector2f SampleUnitSquare() override;
        Eigen::Vector2f SampleUnitDisk() override;
        Eigen::Vector3f SampleHemisphere(SamplingStrategy Strategy, Float DensityPower = 0) override;

        ~CMJSampler() = default;
    private:
        void GenerateSamples();
    };

	class HammersleySampler : public ISampler
	{
	public:
		HammersleySampler(unsigned NumSamples, unsigned NumSets = 97);

        Eigen::Vector2f SampleUnitSquare() override;
        Eigen::Vector2f SampleUnitDisk() override;
        Eigen::Vector3f SampleHemisphere(SamplingStrategy Strategy, Float DensityPower = 0) override;

		~HammersleySampler() = default;

	private:
		void GenerateSamples();
	};

    uint32_t Reverse32bit(uint32_t Number);
	uint64_t Reverse64bit(uint64_t Number);
    
} // namespace PathTracer