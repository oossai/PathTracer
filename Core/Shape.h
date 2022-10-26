#pragma once

#include <Pch.h>
#include <Ray.h>

namespace PathTracer
{
    class TriangleMesh;
    struct Vertex;

    struct IShape
    {
    public:
        IShape() = default;
        virtual ~IShape() = default;
        virtual bool Intersect(const Ray& aRay, Intersection& HitResult) const = 0;
        // virtual void Transform(const Eigen::Matrix4f& Transform) = 0;
        virtual Float GetArea() const noexcept = 0;
    };

	// Triangle
	struct Triangle : public IShape
	{
	public:
		Triangle() = default;

        Triangle(const TriangleMesh* pMesh, const Vertex& V0, const Vertex& V1, const Vertex& V2)
		: pMesh{pMesh}, V0{V0}, V1{V1}, V2{V2} {}

		Triangle(const Triangle&) = delete;
		Triangle& operator=(const Triangle&) = delete;

		Triangle(Triangle&&) = default;
		Triangle& operator=(Triangle&&) = default;

		bool Intersect(const Ray& aRay, Intersection& HitResult) const override;

		Float GetArea() const noexcept override { return Area; };
	public:
        const Vertex& V0;
		const Vertex& V1;
		const Vertex& V2;
		const TriangleMesh* pMesh;
		Float Area;
	};
	
	// Sphere
	struct Sphere : public IShape
	{
		Sphere() = default;
		
		Sphere(const Eigen::Vector3f& Center, Float Radius)
		: mCenter{Center}, Radius{Radius}
		{
			RadiusSq = Radius * Radius;
			Area = 4 * kPi * RadiusSq;
		}

		Sphere(const Sphere&) = delete;
		Sphere& operator=(const Sphere&) = delete;

		Sphere(Sphere&&) = default;
		Sphere& operator=(Sphere&&) = default;
		
		bool Intersect(const Ray& aRay, Intersection& HitResult) const override;

		Float GetArea() const noexcept override { return Area; };
	
		Eigen::Vector3f mCenter;
		Float Radius, RadiusSq, Area;
	};

	// Box
	struct Aabb
	{
		Aabb() noexcept
		{
			Bounds[0] = Eigen::Vector3f(kInfinity, kInfinity, kInfinity);
			Bounds[1] = Eigen::Vector3f(-kInfinity, -kInfinity, -kInfinity);
		}
		
		Aabb(const Eigen::Vector3f& MinBound, const Eigen::Vector3f& MaxBound) noexcept
		{
			Bounds[0] = MinBound;
			Bounds[1] = MaxBound;
		}

		Eigen::Vector3f GetExtent() const noexcept
		{
			return Bounds[1] - Bounds[0];
		}

		Float GetArea() const noexcept
		{
			Eigen::Vector3f Extent = GetExtent();

			return Extent.x() * Extent.y() + Extent.x() * Extent.z() + Extent.y() * Extent.z();
		}

		void GrowBy(const Eigen::Vector3f& Position)
		{
			Bounds[0].x() = std::min(Bounds[0].x(), Position.x());
			Bounds[0].y() = std::min(Bounds[0].y(), Position.y());
			Bounds[0].z() = std::min(Bounds[0].z(), Position.z());

			Bounds[1].x() = std::max(Bounds[1].x(), Position.x());
			Bounds[1].y() = std::max(Bounds[1].y(), Position.y());
			Bounds[1].z() = std::max(Bounds[1].z(), Position.z());
		}

		Aabb(const Aabb&) = delete;
		Aabb& operator=(const Aabb&) = delete;

		Aabb(Aabb&&) = default;
		Aabb& operator=(Aabb&&) = default;

		~Aabb() = default;

		bool Intersect(const Ray& Ray) const;

		Eigen::Vector3f Bounds[2];
	};

} // namespace PathTracer
