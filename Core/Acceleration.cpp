#include <Acceleration.h>
#include <Scene.h>

using namespace Eigen;

namespace PathTracer
{
    Bvh::Bvh(const std::vector<Triangle*>& Triangles)
    : pTriangles{&Triangles}
    {
        mNodes.resize(2 * Triangles.size() - 1);
        
        mTriangleIndices.resize(Triangles.size());
        
        std::iota(mTriangleIndices.begin(), mTriangleIndices.end(), 0);

        CalcTriangleCentroid();

        BuildStructure();
    }

    void Bvh::CalcTriangleCentroid()
    {
        const auto& TriangleList = *pTriangles;
        
        constexpr Float x = 1.f / 3;

        mCentroids.reserve(TriangleList.size());

        for (size_t Index = 0; Index < TriangleList.size(); Index++)
        {
            const auto& Triangle = *TriangleList[Index];

            mCentroids.push_back((Triangle.V0.Position + Triangle.V1.Position + Triangle.V2.Position) * 0.3333333433F);
        }
    }
    
    void Bvh::BuildStructure()
    {
        BvhNode& Node = mNodes[0]; // root node is 0;
        Node.LeftChild = 0;
        Node.NumPrimitives = static_cast<unsigned>(mTriangleIndices.size());

        CalcTriangleCentroid();
        
        UpdateNodeBounds(mNodes[0]);
        MidPointSplit(mNodes[0]);
    }

    void Bvh::UpdateNodeBounds(BvhNode& Node)
    {
        const auto& TriangleList = *pTriangles;

        for (unsigned Index = 0; Index < Node.NumPrimitives; Index++)
        {
            const auto& Triangle = *TriangleList[mTriangleIndices[Index + Node.LeftChild]];

            Node.BoundingBox.GrowBy(Triangle.V0.Position);
            Node.BoundingBox.GrowBy(Triangle.V1.Position);
            Node.BoundingBox.GrowBy(Triangle.V2.Position);
        }
    }

    int Bvh::SplitNode(BvhNode& Node, int SplitAxis, Float SplitPos)
    {
        const auto& Triangles = *pTriangles;

        int CurrentIndex = Node.LeftChild;
        int EndIndex = CurrentIndex + Node.NumPrimitives - 1;

        while (CurrentIndex <= EndIndex)
        {
            if (mCentroids[mTriangleIndices[CurrentIndex]][SplitAxis] < SplitPos)
            {
                CurrentIndex++;
            }
            else
            {
                std::swap(mTriangleIndices[CurrentIndex], mTriangleIndices[EndIndex--]);
            }
        }

        int LeftCount = CurrentIndex - Node.LeftChild;
        int RightCount = Node.NumPrimitives - LeftCount;

        if (LeftCount == 0 || RightCount == 0)
        {
            return -1;
        }

        int LeftChildIndex = mNodesUsed++;
        mNodesUsed++;

        mNodes[LeftChildIndex].NumPrimitives = LeftCount;
        mNodes[LeftChildIndex].LeftChild = Node.LeftChild;

        mNodes[LeftChildIndex + 1].NumPrimitives = RightCount;
        mNodes[LeftChildIndex + 1].LeftChild = CurrentIndex;

        Node.SplitAxis = SplitAxis;
        Node.LeftChild = LeftChildIndex;
        Node.NumPrimitives = 0;

        return LeftChildIndex;
    }

    void Bvh::MidPointSplit(BvhNode& Node)
    {
        if (Node.NumPrimitives <= 2)
        {
            return;
        }

        // Split along longest axis
        Eigen::Vector3f BoxExtent = Node.BoundingBox.GetExtent();

        int SplitAxis = 0;

        if (BoxExtent.y() > BoxExtent.x())
        {
            SplitAxis = 1;
        }

        if (BoxExtent.z() > BoxExtent[SplitAxis])
        {
            SplitAxis = 2;
        }

        Float SplitPos = Node.BoundingBox.Bounds[0][SplitAxis] + BoxExtent[SplitAxis] * 0.5f;

        int LeftChildIndex = SplitNode(Node, SplitAxis, SplitPos);

        if (LeftChildIndex == -1)
        {
            return;
        }

        UpdateNodeBounds(mNodes[LeftChildIndex]);
        UpdateNodeBounds(mNodes[LeftChildIndex + 1]);
        
        MidPointSplit(mNodes[LeftChildIndex]);
        MidPointSplit(mNodes[LeftChildIndex + 1]);
    }

    bool Bvh::Intersect(const Ray& aRay, Intersection& HitResult) const
    {
	    unsigned CurrentNode = 0;
	    unsigned ToVisitOffset = 0;
	    unsigned NodesToVisit[64];

		bool HitSomething = false;
		
		while (true)
		{
			const BvhNode& Node = mNodes[CurrentNode];
			
			if (Node.BoundingBox.Intersect(aRay))
			{
				if (Node.NumPrimitives > 0)
				{
                    auto& Triangles = *pTriangles;

                    for (unsigned Index = 0; Index < Node.NumPrimitives; Index++)
                    {
                        if (Triangles[mTriangleIndices[Index + Node.LeftChild]]->Intersect(aRay, HitResult))
                        {
                            HitSomething = true;
                        }
                    }

					if (ToVisitOffset == 0)
					{
						break;
					}

					CurrentNode = NodesToVisit[--ToVisitOffset];
				}
				else
				{
					if (aRay.IsDirectionNeg[Node.SplitAxis])
					{
						NodesToVisit[ToVisitOffset++] = Node.LeftChild;
						CurrentNode = Node.LeftChild + 1;
					}
					else
					{
						NodesToVisit[ToVisitOffset++] = Node.LeftChild + 1;
						CurrentNode = Node.LeftChild;
					}
				}
			}
			else
			{
				if (ToVisitOffset == 0)
				{
					break;
				}

				CurrentNode = NodesToVisit[--ToVisitOffset];
			}
		}

		return HitSomething;
    }

} // namespace PathTracer