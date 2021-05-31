#pragma once

#include "Triangle.h"
#include "AABB.h"

#include <vector>
#include <cstdint>

#include <glm/glm.hpp>

struct LeafContents {
	std::vector<uint32_t> Indices;
};

struct LeafPointer {
	int32_t Offset;
	int32_t Size;
};

enum class NodeType {
	LEAF,
	NODE
};

struct NodeSerialized {

	void MakeLeaf(void);
	NodeType GetType(void);

	AABB BoundingBox;

	union {
		int32_t ChildrenNodes[2];
		LeafPointer Leaf;
	};
};

// BVH triangle
struct TriangleCentroid {
	glm::vec3 Position;
	uint32_t Index;
};

struct NodeUnserialized {
	NodeUnserialized(void);

	AABB BoundingBox;

	std::vector<TriangleCentroid> Centroids;

	//union {
		NodeUnserialized* Children[2];
		LeafPointer Leaf;
	//};

	NodeType Type;
	int32_t Index;

	uint32_t SplitAxis;

	float ComputeSAH(void);
};

struct Split {
	Split(void);
	// Axis of the split. 0-X, 1-Y, 2-Z
	uint32_t Axis;
	// List of centroids. 0 is behind of the split, 1 is infornt of it
	std::vector<TriangleCentroid> Centroids[2];
	// Boxes formed by the split
	AABB Box[2];
	// SAH score of the split
	float SAH;

	float ComputeSAH(void);
};

class BoundingVolumeHierarchy {
public:
	void ConstructAccelerationStructure(const std::vector<Triangle>& Triangles);

	std::vector<NodeSerialized> Nodes;
	std::vector<int32_t> LeafContents;
};