#include "Mesh.h"
#include <stack>
#include <iostream>

struct TraversalStack {
public:
	TraversalStack(void) : StackIndex(-1) {}

	bool Nonempty(void) {
		return StackIndex > -1;
	}

	void operator <<(int32_t Index) {
		Stack[++StackIndex] = Index;
	}

	void operator >>(int32_t& Index) {
		Index = Stack[StackIndex--];
	}
private:
	int32_t StackIndex;
	int32_t Stack[256];
	// TODO: support for extra stack space using heap array or vector
};

void Mesh::SetTriangles(const std::vector<Triangle>& Triangles) {
	this->Triangles = Triangles;
	BVH.ConstructAccelerationStructure(Triangles);
}

const std::vector<Triangle>& Mesh::GetTriangles(void) const {
	return this->Triangles;
}

bool Mesh::IntersectLeaf(const NodeSerialized& N, const Ray& R, TriangleIntersection& IntersectionInfo) const {
	//std::cout << "Intersection leaf\n";
	auto Begin = BVH.LeafContents.begin() + N.Leaf.Offset;
	auto End   = Begin                    - N.Leaf.Size  ; // Negate to remove leaf flag

	bool Hit = false;

	for (auto Iter = Begin; Iter != End; Iter++) {
		//std::cout << std::distance(BVH.LeafContents.begin(), Iter) << ' ' << std::distance(BVH.LeafContents.begin(), BVH.LeafContents.end()) << ' ' << std::distance(BVH.LeafContents.begin(), End) << '\n';
		Hit |= Triangles[*Iter].Intersect(R, IntersectionInfo, GetMaterial());
	}

	return Hit;
}

bool Mesh::Intersect(const Ray& R, const Ray& CRR, TriangleIntersection& IntersectionInfo) const {
	bool SuccessfulIntersection = false;

	AABBIntersection RootNodeIntersection;
	RootNodeIntersection.Depth = IntersectionInfo.Depth;
	if (!BVH.Nodes[0].BoundingBox.Intersect(CRR, RootNodeIntersection)) {
		return false;
	}

	TraversalStack Stack;
	int32_t CurrentIndex = BVH.Nodes[0].ChildrenNodes[0];

	while (true) {
		NodeSerialized Children[2] = {
			BVH.Nodes[CurrentIndex    ],
			BVH.Nodes[CurrentIndex + 1],
		};

		AABBIntersection IntersectionBounds[2];
		IntersectionBounds[0].Depth = IntersectionInfo.Depth;
		IntersectionBounds[1].Depth = IntersectionInfo.Depth;

		bool AABBHit[2] = {
			Children[0].BoundingBox.Intersect(CRR, IntersectionBounds[0]),
			Children[1].BoundingBox.Intersect(CRR, IntersectionBounds[1]),
		};

		if (AABBHit[0] && Children[0].GetType() == NodeType::LEAF) {
			SuccessfulIntersection |= IntersectLeaf(Children[0], R, IntersectionInfo);
			AABBHit[0] = false;
		}

		if (AABBHit[1] && Children[1].GetType() == NodeType::LEAF) {
			SuccessfulIntersection |= IntersectLeaf(Children[1], R, IntersectionInfo);
			AABBHit[1] = false;
		}

		if (AABBHit[0] && AABBHit[1]) {
			// Should be faster than swapping
			if (IntersectionBounds[0].Min < IntersectionBounds[1].Min) {
				Stack << Children[0].ChildrenNodes[0];
				CurrentIndex = Children[1].ChildrenNodes[0];
			} else {
				Stack << Children[1].ChildrenNodes[0];
				CurrentIndex = Children[0].ChildrenNodes[0];
			}
			continue;
		} else if (AABBHit[0]) {
			CurrentIndex = Children[0].ChildrenNodes[0];
			continue;
		} else if (AABBHit[1]) {
			CurrentIndex = Children[1].ChildrenNodes[0];
			continue;
		} else {
			if (Stack.Nonempty()) {
				Stack >> CurrentIndex;
			} else {
				break;
			}
			continue;
		}
	}

	return SuccessfulIntersection;
}

void Mesh::SetMaterial(const MaterialProperties& Mtl) {
	Material = Mtl;
}

const MaterialProperties* Mesh::GetMaterial(void) const {
	return &Material;
}
