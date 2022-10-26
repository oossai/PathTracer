#include <Shape.h>
#include <Scene.h>

using namespace Eigen;

#define BACKFACECULLING

namespace PathTracer
{
	// Triangle
	bool Triangle::Intersect(const Ray& aRay, Intersection& HitResult) const
	{
		const Vector3f V0ToV1 = V1.Position - V0.Position;
		const Vector3f V0ToV2 = V2.Position - V0.Position;

		const Vector3f V0ToRayOrigin = aRay.Origin - V0.Position;
		const Float MDeterminant = aRay.Direction.cross(V0ToV2).dot(V0ToV1);
		const Float InvMDeterminant = 1 / MDeterminant;

	#ifdef BACKFACECULLING

		const float tHit = V0ToV2.cross(V0ToRayOrigin).dot(V0ToV1) * InvMDeterminant;

		if (tHit < kEpsilon || aRay.tMax < tHit)
		{
			return false;
		}
		
		Float U = aRay.Direction.cross(V0ToV2).dot(V0ToRayOrigin);

		if (U < 0.0 || U > MDeterminant)
		{
			return false;
		}
		
		float V = aRay.Direction.cross(V0ToRayOrigin).dot(V0ToV1);

		if (V < 0.0 || V + U > MDeterminant)
		{
			return false;
		}
		
		U *= InvMDeterminant;
		V *= InvMDeterminant;

		float W = 1 - U - V;

		HitResult.HitPoint = aRay(tHit);
		HitResult.Normal = W * V0.Normal + U * V1.Normal + V * V2.Normal;
		aRay.tMax = tHit;
		
		return true;

	#endif

	}

	// Sphere
	bool Sphere::Intersect(const Ray& aRay, Intersection& HitResult) const
	{
		const Vector3f CenterToRay = aRay.Origin - mCenter;
		const Float Halfb = CenterToRay.dot(aRay.Direction);
		const Float C = CenterToRay.dot(CenterToRay) - RadiusSq;

		const Float Discriminant = Halfb * Halfb - C;

		if (Discriminant < 0)
		{
			return false;
		}

		Float DiscriminantSqrt = std::sqrt(Discriminant);
		Float tHit = -Halfb - DiscriminantSqrt;

		if (tHit > aRay.tMax)
		{
			return false;
		}
		
		if (tHit < kEpsilon)
		{
			tHit = -Halfb + DiscriminantSqrt;
			
			if (tHit < kEpsilon)
			{
				return false;
			}
		}

		HitResult.HitPoint = aRay(tHit);
		HitResult.Normal = (HitResult.HitPoint - mCenter).normalized();
		aRay.tMax = tHit;
		
		return true;				
	}

	// Box
	bool Aabb::Intersect(const Ray& aRay) const
	{
		Float TMin, TMax, TyMin, TyMax, TzMin, TzMax;

		TMin = (Bounds[aRay.IsDirectionNeg[0]].x() -  aRay.Origin.x()) * aRay.InvDirection.x();
		TMax = (Bounds[1 - aRay.IsDirectionNeg[0]].x() -  aRay.Origin.x()) * aRay.InvDirection.x();

		TyMin = (Bounds[aRay.IsDirectionNeg[1]].y() -  aRay.Origin.y()) * aRay.InvDirection.y();
		TyMax = (Bounds[1 - aRay.IsDirectionNeg[1]].y() -  aRay.Origin.y()) * aRay.InvDirection.y();

		if (TyMax < TMin || TMax < TyMin)
		{
			return false;
		}
		
		if (TyMin > TMin)
		{
			TMin = TyMin;
		}
		
		if (TyMax < TMax)
		{
			TMax = TyMax;
		}
		
		TzMin = (Bounds[aRay.IsDirectionNeg[2]].z() -  aRay.Origin.z()) * aRay.InvDirection.z();
		TzMax = (Bounds[1 - aRay.IsDirectionNeg[2]].z() -  aRay.Origin.z()) * aRay.InvDirection.z();

		if (TMin > TzMax || TzMin > TMax)
		{
			return false;
		}

		if (TzMin > TMin)
		{
			TMin = TzMin;
		}
		
		if (TzMax < TMax)
		{
			TMax = TzMax;
		}

		return ((TMin < aRay.tMax) && (TMax > kEpsilon));
	}

} // namespace PathTracer