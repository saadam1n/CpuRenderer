#include "Scene.h"
#include <string>
#include <iostream>
#include <numeric>
#include <limits>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>

glm::vec2 ConvertToGLM(const aiVector2D& Vec) {
    return glm::vec2(Vec.x, Vec.y);
}

glm::vec3 ConvertToGLM(const aiVector3D& Vec) {
    return glm::vec3(Vec.x, Vec.y, Vec.z);
}

glm::vec3 ConvertToGLM(const aiColor4D& Vec) { // Col
    return glm::vec3(Vec.r, Vec.g, Vec.b);
}

// Robust AABB intersection by madman
float SafeInverse(float x) {
    return std::fabs(x) <= std::numeric_limits<float>::epsilon()
        ? std::copysign(1.0f / std::numeric_limits<float>::epsilon(), x)
        : 1.0f / x;
}

glm::vec3 SafeInverse(const glm::vec3& Vec) {
    return glm::vec3(SafeInverse(Vec.x), SafeInverse(Vec.y), SafeInverse(Vec.z));
}

bool Scene::Intersect(const Ray& R, TriangleIntersection& IntersectionInfo) const {
    // Cached rcp ray (inverse ray)
    Ray CRR;
    CRR.Direction = SafeInverse(R.Direction);
    CRR.Origin = -R.Origin * CRR.Direction;

	bool SuccessfulHit = false;

	for (const Mesh* M : Meshes) {
		SuccessfulHit |= M->Intersect(R, CRR, IntersectionInfo);
	}

	return SuccessfulHit;
}

//void GetMaterialProperty(const aiMaterial* Mat, )

void Scene::LoadFile(const char* Path) {
    TotalLightArea = 0.0f;

    std::string CXXPath = Path;
    std::string Folder = CXXPath.substr(0, CXXPath.find_last_of('/') + 1);

    Assimp::Importer Importer;

    // Turn off smooth normals for path tracing to prevent "broken" BRDFs and energy loss. 
    const aiScene* ImportedScene = Importer.ReadFile(Path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenSmoothNormals | aiProcess_GenUVCoords | aiProcess_ImproveCacheLocality);

    for (uint32_t MeshIndex = 0; MeshIndex < ImportedScene->mNumMeshes; MeshIndex++) {
        const aiMesh* ImportedMesh = ImportedScene->mMeshes[MeshIndex];

        std::vector     <Vertex> Vertices;
        std::vector <glm::ivec3> Indices;

        Vertices.reserve(ImportedMesh->mNumVertices);
        Indices.reserve(ImportedMesh->mNumFaces);

        Vertices.reserve(Vertices.size() + ImportedMesh->mNumVertices);
        for (uint32_t VertexIndex = 0; VertexIndex < ImportedMesh->mNumVertices; VertexIndex++) {
            Vertex CurrentVertex;

            CurrentVertex.Position = ConvertToGLM(ImportedMesh->mVertices[VertexIndex]);
            CurrentVertex.Normal   = ConvertToGLM(ImportedMesh->mNormals[VertexIndex]);

            if (ImportedMesh->mTextureCoords[0]) {
                // So you can cast vec3 to vec2 implicitly? Ok...
                CurrentVertex.TextureCoordinates = ConvertToGLM(ImportedMesh->mTextureCoords[0][VertexIndex]);
            }

            Vertices.push_back(CurrentVertex);
        }

        for (uint32_t FaceIndex = 0; FaceIndex < ImportedMesh->mNumFaces; FaceIndex++) {
            const aiFace& Face = ImportedMesh->mFaces[FaceIndex];

            glm::ivec3 CurrentIndexData;
            for (uint32_t ElementIndex = 0; ElementIndex < Face.mNumIndices; ElementIndex++) {
                CurrentIndexData[ElementIndex] = Face.mIndices[ElementIndex];
            }

            Indices.push_back(CurrentIndexData);
        }

        Mesh* CurrentMesh = new Mesh;

        std::vector<Triangle> Triangles;

        Triangles.reserve(Indices.size());

        for (const glm::ivec3& IndexTriple : Indices) {
            Triangle T;

            T.Vertices[0] = Vertices[IndexTriple[0]];
            T.Vertices[1] = Vertices[IndexTriple[1]];
            T.Vertices[2] = Vertices[IndexTriple[2]];

            Triangles.push_back(T);
        }

        CurrentMesh->SetTriangles(Triangles);

        const aiMaterial* ImportedMaterial = ImportedScene->mMaterials[ImportedMesh->mMaterialIndex];

        MaterialProperties Material;
        aiColor4D Color;
        
        aiGetMaterialColor(ImportedMaterial, AI_MATKEY_COLOR_DIFFUSE, &Color);
        Material.Diffuse = ConvertToGLM(Color);
        //std::cout << glm::dot(Material.Diffuse, glm::vec3(1.0f)) << '\n';

        aiGetMaterialColor(ImportedMaterial, AI_MATKEY_COLOR_EMISSIVE, &Color);
        glm::vec3 GLMColor = ConvertToGLM(Color);
        if (glm::dot(glm::vec3(1.0f), GLMColor) > std::numeric_limits<float>::epsilon()) {
            Material.Type = MaterialProperties::SurfaceType::Emissive;
            Material.Emission = GLMColor;
            //std::cout << "Emissive object\n";

            PolygonalLight CurrentLight;
            CurrentLight.SetPolygons(CurrentMesh->GetTriangles());
            CurrentLight.Material = CurrentMesh->GetMaterial();

            Lights.push_back(CurrentLight);

            TotalLightArea += CurrentLight.GetArea();
        }

        aiGetMaterialFloat(ImportedMaterial, AI_MATKEY_REFRACTI, &Material.RefractiveIndex);
        if (fabsf(Material.RefractiveIndex - 1.0f) > 1e-1f) { // Just to be sure no prasing errors occured
            std::cout << "Refract:\t" << Material.RefractiveIndex << '\n';
            Material.Diffuse = glm::vec3(1.0f);
            Material.Type = MaterialProperties::SurfaceType::Refractive;
        }

        CurrentMesh->SetMaterial(Material);

        Meshes.push_back(CurrentMesh);

    }

    Importer.FreeScene();
}

// Basic sky I always use. Copied from builderboy's code
glm::vec3 Scene::GetSkyIrradiance(const Ray& R) const {
    constexpr glm::vec3 SkyCoefficients = glm::vec3(0.0625, 0.125, 0.25);
    return SkyCoefficients / (R.Direction.y * R.Direction.y + SkyCoefficients);
}

bool Scene::ComputeVisibility(const Ray& R, const Triangle* T, const float L) const {
    // Cached rcp ray (inverse ray)
    Ray CRR;
    CRR.Direction = SafeInverse(R.Direction);
    CRR.Origin = -R.Origin * CRR.Direction;

    bool SuccessfulHit = false;

    TriangleIntersection EarlyExitInfo;
    for (const Mesh* M : Meshes) {
        SuccessfulHit |= M->Intersect(R, CRR, EarlyExitInfo);
        constexpr float kEpsilon = 1e-1f; // Generous epsilon to prevent errors at even large distances
        if (EarlyExitInfo.Depth + kEpsilon < L) {
            //break;
        }
    }

    //std::cout << EarlyExitInfo.Surface << '\t' << T << '\n';

    return (!SuccessfulHit) || (EarlyExitInfo.Surface == T);
}

const PolygonalLight* Scene::GetRandomLight(const float X) const {
    float AreaIndex = X * TotalLightArea;
    float AreaCounter = 0.0f;

    for (const PolygonalLight& Light : Lights) {
        AreaCounter += Light.GetArea();

        if (AreaIndex < AreaCounter) {
            return &Light;
        }
    }

    return &Lights.back();
    //abort();
}

float Scene::GetTotalLightArea(void) const {
    return TotalLightArea;
}

void PolygonalLight::SetPolygons(const std::vector<Triangle>& Geometry) {
    Area = 0.0f;

    SelectionTriangles.clear();
    SelectionTriangles.reserve(Geometry.size());

    for (const Triangle& T : Geometry) {
        // Formula I learned from high school math textbook:
        // Area of triangle = 1/2 * length of side length 1 * length of side length 2 * sine of included angle
        glm::vec3 V01 = T.Vertices[1].Position - T.Vertices[0].Position;
        glm::vec3 V02 = T.Vertices[2].Position - T.Vertices[0].Position;

        float Length1 = glm::length(V01);
        float Length2 = glm::length(V02);

        float CosineTheta = glm::dot(V01, V02) / (Length1 * Length2);
        float SineTheta = std::sqrt(1.0f - CosineTheta * CosineTheta);

        AreaTriangle NewT;

        NewT.T = &T;
        NewT.A = 0.5f * Length1 * Length2 * SineTheta;

        Area += NewT.A;

        SelectionTriangles.push_back(NewT);
    }
}

/*
X = triangle to choose, by area
R0 and R1 - to choose point on chosen triangle
*/
std::pair<Vertex, const Triangle*> PolygonalLight::ChooseRandomPoint(float X, float R0, float R1) const {
    float AreaIndex = X * Area;
    float AreaCounter = 0.0f;

    for (const AreaTriangle& AT : SelectionTriangles) {
        AreaCounter += AT.A;

        if (AreaIndex < AreaCounter) {


            return GetRandomPoint(AT, R0, R1);
        }
    }

    return GetRandomPoint(SelectionTriangles.back(), R0, R1);
    //abort();
}

std::pair<Vertex, const Triangle*> PolygonalLight::GetRandomPoint(const AreaTriangle& AT, float R0, float R1) const {
    // Choose random point
    float R0_0_5 = std::sqrtf(R0);

    // https://chrischoy.github.io/research/barycentric-coordinate-for-mesh-sampling/ 
    Vertex LightVertex =
        AT.T->Vertices[0] * (1.0f - R0_0_5) +
        AT.T->Vertices[1] * (1.0f - R1) * R0_0_5 +
        AT.T->Vertices[2] * R0_0_5 * R1;

    return std::make_pair(LightVertex, AT.T);
}

float PolygonalLight::GetArea(void) const {
    return Area;
}

/*
hardcode cornell 

        glm::vec3 Colors[] = {
            glm::vec3(0.725f, 0.71f, 0.68f),
            glm::vec3(0.725f, 0.71f, 0.68f),
            glm::vec3(0.725f, 0.71f, 0.68f),
            glm::vec3(0.14f, 0.45f, 0.091f), // Green
            glm::vec3(0.63f, 0.065f, 0.05f), // Red
            glm::vec3(0.725f, 0.71f, 0.68f),
        };


        Material.Diffuse = Colors[MeshIndex];
        if (MeshIndex == 5) {
            Material.isEmissive = true;
            Material.Emission = glm::vec3(17.0f, 12.0f, 4.0f);
        }
*/