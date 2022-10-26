#pragma once

#include <Pch.h>
#include <Ray.h>
#include <Shape.h>
#include <Acceleration.h>

namespace PathTracer
{
	struct Vertex
	{
		Eigen::Vector3f Position;
		Eigen::Vector3f Normal;
	};

    class TriangleMesh
	{
	friend class Scene;

	public:
		TriangleMesh() = default;

		TriangleMesh(const TriangleMesh&) = delete;
		TriangleMesh& operator=(const TriangleMesh&) = delete;

		TriangleMesh(TriangleMesh&&) = default;
		TriangleMesh& operator=(TriangleMesh&&) = default;

		~TriangleMesh() = default;

		bool Intersect(const Ray& aRay, Intersection& Result) const;

		void BuildTriangles();
	private:        
		std::vector<Vertex> mVertices;
        std::vector<unsigned> mIndices;
		std::vector<Triangle> mTriangles;
	};
    
	class Scene
	{
	public:
		bool Intersect(const Ray& aRay, Intersection& HitResult) const
		{
			return mBvh->Intersect(aRay, HitResult);
		}

		Scene() = default;

		Scene(std::string_view FileName);

		~Scene() = default;

		void ProcessNode(const aiScene* pScene, const aiNode* pNode);

		void BuildBvh();
	private:

        std::vector<TriangleMesh> mMeshes;
		std::vector<Triangle*> pTriangles;
		std::unique_ptr<Bvh> mBvh;
	};

} // namespace PathTracer