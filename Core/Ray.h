#pragma once

#include <Pch.h>
#include <Constants.h>

namespace PathTracer
{
    struct Intersection
    {
        Eigen::Vector3f HitPoint;
        Eigen::Vector3f Normal;
    };
    
	struct Ray
	{
        Ray(const Eigen::Vector3f& Origin, const Eigen::Vector3f& Direction, Float tMax = kInfinity)
        : Origin{Origin}, Direction{Direction}, tMax{tMax}
        {
            InvDirection.x() = 1 / Direction.x();
            InvDirection.y() = 1 / Direction.y();
            InvDirection.z() = 1 / Direction.z();

            IsDirectionNeg[0] = Direction.x() < 0;
            IsDirectionNeg[1] = Direction.y() < 0;
            IsDirectionNeg[2] = Direction.z() < 0;
        }

        Eigen::Vector3f operator()(Float tPoint) const noexcept
        {
            return Origin + tPoint * Direction;
        }

        Ray() = default;

		Ray(const Ray&) = delete;
		Ray& operator=(const Ray&) = delete;

		Ray(Ray&&) = default;
		Ray& operator=(Ray&&) = default;
		
		~Ray() = default;

		Eigen::Vector3f Origin;
		Eigen::Vector3f Direction;
		Eigen::Vector3f InvDirection;
		unsigned IsDirectionNeg[3];
		mutable Float tMax;
	};

} // namespace PathTracer