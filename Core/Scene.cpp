#pragma once

#include <Scene.h>

#define ASSIMP_PREPROCESS_FLAGS (aiProcess_Triangulate | aiProcess_JoinIdenticalVertices)

using namespace Eigen;

namespace PathTracer
{
	void TriangleMesh::BuildTriangles()
	{
		mTriangles.reserve(mIndices.size() / 3);

		for (size_t Index = 0; Index < mIndices.size(); Index += 3)
		{
			mTriangles.emplace_back(this, mVertices[mIndices[Index]], mVertices[mIndices[Index + 1]], mVertices[mIndices[Index + 2]]);
		}
	}

    bool TriangleMesh::Intersect(const Ray& aRay, Intersection& HitResult) const
    {
        bool Result = false;
        
        for (const auto& Triangle : mTriangles)
        {
            if (Triangle.Intersect(aRay, HitResult))
            {
                Result = true;
            }
        }

        return Result;
    }

	Scene::Scene(std::string_view FileName)
	{
		using namespace std::string_literals;

		Assimp::Importer Importer;

		const aiScene* pScene = Importer.ReadFile(FileName.data(), ASSIMP_PREPROCESS_FLAGS);

		if (pScene == nullptr)
		{
			throw std::runtime_error("Assimp Error \n"s + Importer.GetErrorString());
		}

		mMeshes.reserve(pScene->mNumMeshes);

		ProcessNode(pScene, pScene->mRootNode);

		BuildBvh();
	}

	void Scene::ProcessNode(const aiScene* pScene, const aiNode* pNode)
	{
		if (pNode)
		{
			for (size_t MeshIndex = 0; MeshIndex < pNode->mNumMeshes; MeshIndex++)
			{
				const aiMesh& mMesh = *pScene->mMeshes[pNode->mMeshes[MeshIndex]];

				TriangleMesh NewMesh;
				
				NewMesh.mVertices.reserve(mMesh.mNumVertices);

				for (size_t VertexIndex = 0; VertexIndex < mMesh.mNumVertices; VertexIndex++)
				{
					Vertex NewVertex;

					NewVertex.Position.x() = mMesh.mVertices[VertexIndex].x;
					NewVertex.Position.y() = mMesh.mVertices[VertexIndex].y;
					NewVertex.Position.z() = mMesh.mVertices[VertexIndex].z;

					NewVertex.Normal.x() = mMesh.mNormals[VertexIndex].x;
					NewVertex.Normal.y() = mMesh.mNormals[VertexIndex].y;
					NewVertex.Normal.z() = mMesh.mNormals[VertexIndex].z;

					NewMesh.mVertices.push_back(std::move(NewVertex));
				}

				NewMesh.mIndices.reserve(mMesh.mNumFaces * 3);

				for (size_t FaceIndex = 0; FaceIndex < mMesh.mNumFaces; FaceIndex++)
				{
					assert (mMesh.mFaces[FaceIndex].mNumIndices == 3);

					NewMesh.mIndices.push_back(mMesh.mFaces[FaceIndex].mIndices[0]);
					NewMesh.mIndices.push_back(mMesh.mFaces[FaceIndex].mIndices[1]);
					NewMesh.mIndices.push_back(mMesh.mFaces[FaceIndex].mIndices[2]);
				}

				NewMesh.BuildTriangles();

				mMeshes.push_back(std::move(NewMesh));
			}
		}

		for (size_t ChildIndex = 0; ChildIndex < pNode->mNumChildren; ChildIndex++)
		{
			ProcessNode(pScene, pNode->mChildren[ChildIndex]);
		}
	}

    void Scene::BuildBvh()
    {
        for (int i = 0; i < mMeshes.size(); i++)
        {
            int MeshTriangleCount = static_cast<int>(mMeshes[i].mTriangles.size());

            for (int j = 0; j < MeshTriangleCount; j++)
            {
                pTriangles.push_back(&mMeshes[i].mTriangles[j]);
            }
        }
        
        pTriangles.shrink_to_fit();

        mBvh = std::make_unique<Bvh>(pTriangles);
    }

} // namespace PathTracer