#include "Scene.h"
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

bool Scene::Intersect(const Ray& R, TriangleIntersection& IntersectionInfo) const {
	bool SuccessfulHit = false;

	for (const Mesh& M : Meshes) {
		SuccessfulHit |= M.Intersect(R, IntersectionInfo);
	}

	return SuccessfulHit;
}

void Scene::LoadFile(const char* Path) {
    std::string CXXPath = Path;
    std::string Folder = CXXPath.substr(0, CXXPath.find_last_of('/') + 1);

    Assimp::Importer Importer;

    // Turn off smooth normals for path tracing to prevent "broken" BRDFs and energy loss. 
    const aiScene* Scene = Importer.ReadFile(Path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals | aiProcess_GenUVCoords);

    for (uint32_t MeshIndex = 0; MeshIndex < Scene->mNumMeshes; MeshIndex++) {
        const aiMesh* SceneComponent = Scene->mMeshes[MeshIndex];

        std::vector     <Vertex> Vertices;
        std::vector <glm::ivec3> Indices;

        Vertices.reserve(SceneComponent->mNumVertices);
        Indices.reserve(SceneComponent->mNumFaces);

        glm::vec3 MeshColor;

        Vertices.reserve(Vertices.size() + SceneComponent->mNumVertices);
        for (uint32_t VertexIndex = 0; VertexIndex < SceneComponent->mNumVertices; VertexIndex++) {
            Vertex CurrentVertex;

            aiVector3D& Position = SceneComponent->mVertices[VertexIndex];
            aiVector3D& Normal   = SceneComponent->mNormals [VertexIndex];

            CurrentVertex.Position = glm::vec3(Position.x, Position.y, Position.z);
            CurrentVertex.Normal = glm::vec3(Normal.x, Normal.y, Normal.z);

            if (SceneComponent->mTextureCoords[0]) {
                aiVector3D& TextureCoordinates = SceneComponent->mTextureCoords[0][VertexIndex];
                CurrentVertex.TextureCoordinates = glm::vec2(TextureCoordinates.x, TextureCoordinates.y);
            }

            Vertices.push_back(CurrentVertex);

            if (SceneComponent->mColors[0])
                MeshColor = glm::vec3(SceneComponent->mColors[0][VertexIndex].r, SceneComponent->mColors[0][VertexIndex].g, SceneComponent->mColors[0][VertexIndex].b);
        }

        Indices.reserve(Indices.size() + SceneComponent->mNumFaces);
        for (uint32_t FaceIndex = 0; FaceIndex < SceneComponent->mNumFaces; FaceIndex++) {
            const aiFace& Face = SceneComponent->mFaces[FaceIndex];

            glm::ivec3 CurrentIndexData;
            for (uint32_t ElementIndex = 0; ElementIndex < Face.mNumIndices; ElementIndex++) {
                CurrentIndexData[ElementIndex] = Face.mIndices[ElementIndex];
            }

            Indices.push_back(CurrentIndexData);
        }

        Mesh CurrentMesh;

        std::vector<Triangle> Triangles;

        Triangles.reserve(Indices.size());

        for (const glm::ivec3& IndexTriple : Indices) {
            Triangle T;

            T.Vertices[0] = Vertices[IndexTriple[0]];
            T.Vertices[1] = Vertices[IndexTriple[1]];
            T.Vertices[2] = Vertices[IndexTriple[2]];

            Triangles.push_back(T);
        }

        CurrentMesh.SetTriangles(Triangles);

        Meshes.push_back(CurrentMesh);
    }

    Importer.FreeScene();
}

// Basic sky I always use. Copied from builderboy's code
glm::vec3 Scene::GetSkyIrradiance(const Ray& R) const {
    constexpr glm::vec3 SkyCoefficients = glm::vec3(0.0625, 0.125, 0.25);
    return SkyCoefficients / (R.Direction.y * R.Direction.y + SkyCoefficients);
}