#pragma once

#include <Pch.h>
#include <Shape.h>
#include <Constants.h>

namespace PathTracer
{
	struct BvhNode
	{
        Aabb BoundingBox;
        unsigned LeftChild = 0;
        unsigned NumPrimitives = 0;
        unsigned SplitAxis = 0;
	};

    class Bvh
    {
    public:
        Bvh(const std::vector<Triangle*>& pTriangles);

		Bvh(const Bvh&) = delete;
		Bvh& operator=(const Bvh&) = delete;

		Bvh(Bvh&&) = default;
		Bvh& operator=(Bvh&&) = default;

        void BuildStructure();

        void CalcTriangleCentroid();
        
        void UpdateNodeBounds(BvhNode& Node);

        int SplitNode(BvhNode& Node, int SplitAxis, Float SplitPos);
        
        void MidPointSplit(BvhNode& Node);

        bool Intersect(const Ray& aRay, Intersection& HitResult) const;

    private:
        const std::vector<Triangle*>* pTriangles;
        std::vector<unsigned> mTriangleIndices;
        std::vector<Eigen::Vector3f> mCentroids;
        std::vector<BvhNode> mNodes;
        int mNodesUsed = 1;
    };

} // namespace PathTracer