#include "BVH.h"
#include "TimeUtil.h"
#include <stack>
#include <algorithm>
#include <list>
#include <queue>
#include <thread>
#include <stdio.h>
#include <iostream>
#include <future>
#include <condition_variable>

void DebugPrintBVH(const std::vector<NodeSerialized>& Nodes, const std::vector<int32_t>& LeafContents);

using BVH = BoundingVolumeHierarchy;

// 4T optimal on FX 8350
constexpr size_t WorkerThreadCount = 4;

struct ConstructionNode {
	NodeUnserialized* DataPtr;
	int32_t Depth;
};

// Allocates nodes in chunks
struct NodeAllocator {
	constexpr static size_t PoolSize = 16384;

	std::vector<std::vector<NodeUnserialized>> MemoryPools;
	size_t PoolIndex;

	std::vector<NodeUnserialized>::iterator NextFreeNode;

	void AllocatePool(size_t PIdx) {
		MemoryPools.push_back(std::vector<NodeUnserialized>(PoolSize));
		PoolIndex = PIdx;

		NextFreeNode = MemoryPools[PIdx].begin();
	}

	void AllocNewPool(void) {
		PoolIndex++;
		AllocatePool(PoolIndex);
	}

	NodeAllocator(void) : PoolIndex(SIZE_MAX) {
		// Allocate initial pool
		AllocNewPool();
	}

	NodeUnserialized* AllocateNode(void) {
		// Inc iter, alloc pool if current pool has no free nodes remaining
		NextFreeNode++;
		if (NextFreeNode == MemoryPools[PoolIndex].end()) {
			AllocNewPool();
		}
		// Return address of iter
		return &*NextFreeNode;
	}
};

inline std::vector<TriangleCentroid> SortAxis(const std::vector<TriangleCentroid>& Centroids, uint32_t Axis) {

	std::vector<TriangleCentroid> Sorted(Centroids);

	std::sort(
		Sorted.begin(), Sorted.end(),
		[Axis](const TriangleCentroid& Arg0, const TriangleCentroid& Arg1) -> bool {
			return Arg0.Position[Axis] < Arg1.Position[Axis];
		}
	);

	return Sorted;
}

inline AABB CreateBoundingBox(const std::vector<TriangleCentroid>& Centroids, const std::vector<AABB>& TriangleBoundingBoxes) {
	// Loop through all centroids and get their triangle to create box
	AABB Box;

	for (const TriangleCentroid& Centroid : Centroids) {
		Box.Extend(TriangleBoundingBoxes[Centroid.Index]);
	}

	return Box;
}

// Precompute bounding boxes with another transfer split like algorithm
inline std::vector<AABB> PrecomputeBoundingBoxes(const std::vector<TriangleCentroid>& Centroids, const std::vector<AABB>& TriangleBoundingBoxes, size_t Count) {
	std::vector<AABB> BoundingBoxes;
	BoundingBoxes.reserve(Count);

	AABB CurrentExtent;

	for (size_t CentroidIndex = 0; CentroidIndex < Count; CentroidIndex++) {
		CurrentExtent.Extend(TriangleBoundingBoxes[Centroids[CentroidIndex].Index]);

		BoundingBoxes.push_back(CurrentExtent);
	}

	// Reverse since transfer algorithm iteratively adds, not subtractes like the loop
	std::reverse(BoundingBoxes.begin(), BoundingBoxes.end());

	return BoundingBoxes;
}

inline Split FindBestSplit(const std::vector<TriangleCentroid>& Centroids, const std::vector<AABB>& TriangleBoundingBoxes, uint32_t Axis, uint32_t PreviousAxis) {
	struct FastSplit {
		AABB BoundingBox[2];
		size_t CentroidCounts[2];
		float SAH;

		void Initialize(const std::vector<TriangleCentroid>& SortedCentroidArray) {
			CentroidCounts[0] = SortedCentroidArray.size();
			CentroidCounts[1] = 0;
		}

		const TriangleCentroid& Transfer(const std::vector<TriangleCentroid>& SortedCentroidArray) {
			size_t ReverseIndexIter = --CentroidCounts[0];
			                          CentroidCounts[1]++;

									  return SortedCentroidArray[ReverseIndexIter];
		}

		float ComputeSAH(void) {
			SAH = BoundingBox[0].SurfaceAreaHalf() * CentroidCounts[0] + BoundingBox[1].SurfaceAreaHalf() * CentroidCounts[1];
			return SAH;
		}
	};

	std::vector<TriangleCentroid> SortedCentroidArray = Axis == PreviousAxis ? Centroids : SortAxis(Centroids, Axis);
	size_t TransferableCentroids = SortedCentroidArray.size() - 1;
	std::vector<AABB> BoundingBoxes = PrecomputeBoundingBoxes(SortedCentroidArray, TriangleBoundingBoxes, TransferableCentroids);

	FastSplit CurrentSplit;
	CurrentSplit.Initialize(SortedCentroidArray);

	Split BestSplit;
	size_t BestSplitIndex = 0;

	for (size_t CentroidIndex = 0; CentroidIndex < TransferableCentroids; CentroidIndex++) {
		// Create bounding boxes and everything. 
		CurrentSplit.BoundingBox[0] = BoundingBoxes[CentroidIndex]; // This might benfit from a check of whether the centroid had an effect on the bounding box
		// Transfer the centroid
		CurrentSplit.BoundingBox[1].Extend(TriangleBoundingBoxes[CurrentSplit.Transfer(SortedCentroidArray).Index]);

		// Compute SAH and update if it's better than the current best split
		if (CurrentSplit.ComputeSAH() < BestSplit.SAH) {
			BestSplit.SAH = CurrentSplit.SAH;

			BestSplit.Box[0] = CurrentSplit.BoundingBox[0];
			BestSplit.Box[1] = CurrentSplit.BoundingBox[1];

			BestSplitIndex = CentroidIndex;
		}
	}
	BestSplitIndex++;

	BestSplit.Centroids[0] = std::move(std::vector<TriangleCentroid>(SortedCentroidArray.cbegin() , SortedCentroidArray.cend()    - BestSplitIndex));
	BestSplit.Centroids[1] = std::move(std::vector<TriangleCentroid>(SortedCentroidArray.crbegin(), SortedCentroidArray.crbegin() + BestSplitIndex));

	return BestSplit;
}

inline Split ChooseBestSplit(const Split& X, const Split& Y, const Split& Z, uint32_t& Axis) {
	if (Z.SAH > X.SAH&& Z.SAH > Y.SAH) {
		if (Y.SAH < X.SAH) {
			Axis = 1;
			return Y;
		}
		else {
			Axis = 0;
			return X;
		}
	}
	else {
		Axis = 3;
		return Z;
	}
}

NodeUnserialized::NodeUnserialized(void) : Children{ nullptr, nullptr }, Type(NodeType::NODE) {}

Split::Split(void) : SAH(FLT_MAX) {}

float Split::ComputeSAH(void) {
	// Fun fact: we don't need the actual surface area, just a quantity porportional to it. So we can avoid multiplying by 2 in this case
	SAH = Box[0].SurfaceAreaHalf() * Centroids[0].size() + Box[1].SurfaceAreaHalf() * Centroids[1].size();
	return SAH;
}

float NodeUnserialized::ComputeSAH(void) {
	return BoundingBox.SurfaceAreaHalf() * Centroids.size();
}

// Terminate if all threads are waiting
bool AllThreadWaitingConditions(bool* WaitConditions) {
	bool AllWait = true;

	for (size_t Counter = 0; Counter < WorkerThreadCount; Counter++) {
		AllWait &= WaitConditions[Counter];
	}

	return AllWait;
};

void MakeLeaf(ConstructionNode& CurrentNode, std::vector<int32_t>& LBuf, std::mutex& LBufMutex) {
	CurrentNode.DataPtr->Type = NodeType::LEAF;

	std::lock_guard<std::mutex> LBufLock(LBufMutex);

	CurrentNode.DataPtr->Leaf.Offset = (int32_t)LBuf.size();
	CurrentNode.DataPtr->Leaf.Size   = (int32_t)CurrentNode.DataPtr->Centroids.size();

	for (const TriangleCentroid& Tri : CurrentNode.DataPtr->Centroids) {
		LBuf.push_back(Tri.Index);
	}
}

void ParallelConstructionTask(
	std::condition_variable& WorkSignal,
	std::mutex& WorkMtx,
	bool* WaitConditions,
	bool& ThreadWait,
	const std::vector<AABB>& TriAABBs,
	std::stack<ConstructionNode>& ConstructionNodes,
	std::mutex& ConstructionMutex,
	std::vector<int32_t>& LBuf,
	std::mutex& LBufMutex,
	NodeAllocator& Alloc,
	std::mutex& AllocMutex
) {
	while (true) {
		ConstructionNode CurrentNode;

		bool FirstWait = true;
		while (true) {
			ConstructionMutex.lock();
			if (!ConstructionNodes.empty()) {
				CurrentNode = ConstructionNodes.top();
				ConstructionNodes.pop();

				ConstructionMutex.unlock();
				break;
			} else {
				ConstructionMutex.unlock();

				if (FirstWait) {
					ThreadWait = true;
					FirstWait = false;

					WorkSignal.notify_all();
				}

				if (AllThreadWaitingConditions(WaitConditions)) {
					return;
				}

				std::unique_lock<std::mutex> SignalLock(WorkMtx);
				WorkSignal.wait(SignalLock); // Wait until next update

			}
		}

		ThreadWait = false;

		// If this node has just 1 triangle, let's just turn it into a leaf immediatly without any try-spliting 
		if (CurrentNode.DataPtr->Centroids.size() < 2) {
			MakeLeaf(CurrentNode, LBuf, LBufMutex);
			continue;
		}

		// Next try to find the best split on all 3 axes
		Split TentativeSplits[3];

		TentativeSplits[0] = FindBestSplit(CurrentNode.DataPtr->Centroids, TriAABBs, 0, CurrentNode.DataPtr->SplitAxis);
		TentativeSplits[1] = FindBestSplit(CurrentNode.DataPtr->Centroids, TriAABBs, 1, CurrentNode.DataPtr->SplitAxis);
		TentativeSplits[2] = FindBestSplit(CurrentNode.DataPtr->Centroids, TriAABBs, 2, CurrentNode.DataPtr->SplitAxis);
		// Find/selelct the best split from all axes 
		uint32_t ChosenAxis;
		Split BestSplit = ChooseBestSplit (
			TentativeSplits[0],
			TentativeSplits[1],
			TentativeSplits[2],
			ChosenAxis
		);

		// Subdivision termination taken from Jacco Bikker, "The Perfect BVH", slide #12
		if (BestSplit.SAH > CurrentNode.DataPtr->ComputeSAH()) {
			MakeLeaf(CurrentNode, LBuf, LBufMutex);
		} else {
			// Construct children nodes
			ConstructionNode Children[2];

			AllocMutex.lock();
			Children[0].DataPtr = Alloc.AllocateNode();
			Children[1].DataPtr = Alloc.AllocateNode();
			AllocMutex.unlock();

			Children[0].DataPtr->BoundingBox = BestSplit.Box[0];
			Children[1].DataPtr->BoundingBox = BestSplit.Box[1];

			Children[0].DataPtr->Centroids = BestSplit.Centroids[0];
			Children[1].DataPtr->Centroids = BestSplit.Centroids[1];

			Children[0].DataPtr->SplitAxis = ChosenAxis;
			Children[1].DataPtr->SplitAxis = ChosenAxis;

			int32_t ChildDepth = CurrentNode.Depth + 1;

			Children[0].Depth = ChildDepth;
			Children[1].Depth = ChildDepth;

			CurrentNode.DataPtr->Children[0] = Children[0].DataPtr;
			CurrentNode.DataPtr->Children[1] = Children[1].DataPtr;

			// Push nodes onto the stack
			ConstructionMutex.lock();
			ConstructionNodes.push(Children[0]);
			ConstructionNodes.push(Children[1]);
			ConstructionMutex.unlock();
			
			WorkSignal.notify_all();
		}
	}
}

// Prevent construction bug where all threads sleep
void RenotifyThreads(std::condition_variable& WorkSignal, bool& WorkerThreadsRunning) {
	while (WorkerThreadsRunning) {
		WorkSignal.notify_all();
		// Renotify every 1 ms
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	//std::cout << "Done renotifying\n";
}

void BoundingVolumeHierarchy::ConstructAccelerationStructure(const std::vector<Triangle>& Triangles) {
	std::cout << "Start of BVH construction" << std::endl;

	Timer ConstructionTimer;

	// First things first. We need to build a list of triangles

	ConstructionTimer.Begin();

	std::vector<TriangleCentroid> CentroidList;
	CentroidList.reserve(Triangles.size());

	std::vector<AABB> TriangleBoundingBoxes;

	TriangleBoundingBoxes.reserve(Triangles.size());

	for (uint32_t TriIdx = 0; TriIdx < Triangles.size(); TriIdx++) {
		Triangle CurrentTriangle = Triangles[TriIdx];

		// Calculate centriod of the triangle. We need this in order to split triangles in the BVH building process
		TriangleCentroid Centroid;

		Centroid.Index = TriIdx;

		Centroid.Position =
			CurrentTriangle.Vertices[0].Position +
			CurrentTriangle.Vertices[1].Position +
			CurrentTriangle.Vertices[2].Position ;

		Centroid.Position /= 3.0f;

		CentroidList.push_back(Centroid);

		AABB Box;

		Box.Extend(CurrentTriangle.Vertices[0].Position);
		Box.Extend(CurrentTriangle.Vertices[1].Position);
		Box.Extend(CurrentTriangle.Vertices[2].Position);

		TriangleBoundingBoxes.push_back(Box);
	}

	ConstructionTimer.End();
	//ConstructionTimer.DebugTime();
	ConstructionTimer.Begin();
	
	NodeAllocator Allocator;
	std::mutex AllocatorMutex;

	std::stack<ConstructionNode> ConstructionNodeStack;
	std::mutex ConstructionStackMutex;

	std::vector<int32_t> LeafContentBuffer;
	std::mutex LeafContentMutex;

	NodeUnserialized* RootNode = Allocator.AllocateNode();

	RootNode->BoundingBox = CreateBoundingBox(CentroidList, TriangleBoundingBoxes);
	RootNode->Centroids   =                   CentroidList;
	RootNode->SplitAxis   = UINT32_MAX;

	ConstructionNode CRN; // CRN = construction root node

	CRN.DataPtr = RootNode;
	CRN.Depth = 0;

	ConstructionNodeStack.push(CRN);

	bool ThreadWaitingConditions[WorkerThreadCount]{ false };

	std::future<void> WorkerThreads[WorkerThreadCount];

	std::condition_variable WorkSignal;
	std::mutex WorkMtx;

	// First begin the renotification thread
	bool WorkerThreadsRunning = true;
	std::future<void> RenotificationThread = std::async(std::launch::async, RenotifyThreads, std::ref(WorkSignal), std::ref(WorkerThreadsRunning));

	for (size_t Index = 0; Index < WorkerThreadCount; Index++) {
		WorkerThreads[Index] = std::async(
			std::launch::async,
			ParallelConstructionTask,
			std::ref(WorkSignal),
			std::ref(WorkMtx),
			ThreadWaitingConditions, 
			std::ref(ThreadWaitingConditions[Index]),
			std::ref(TriangleBoundingBoxes),
			std::ref(ConstructionNodeStack),
			std::ref(ConstructionStackMutex),
			std::ref(LeafContentBuffer),
			std::ref(LeafContentMutex),
			std::ref(Allocator),
			std::ref(AllocatorMutex)
		);
	}

	for (size_t Index = 0; Index < WorkerThreadCount; Index++) {
		WorkerThreads[Index].wait();
	}

	WorkerThreadsRunning = false;
	// Wait just to make sure it's done
	RenotificationThread.wait();

	ConstructionTimer.End();
	//ConstructionTimer.DebugTime();
	ConstructionTimer.Begin();

	std::queue<NodeUnserialized*> IndexBuildingQueue;
	IndexBuildingQueue.push(RootNode);

	int32_t IndexCounter = 0;

	while (!IndexBuildingQueue.empty()) {
		NodeUnserialized* CurrentNode = IndexBuildingQueue.front();
		IndexBuildingQueue.pop();

		CurrentNode->Index = IndexCounter++;

		if (CurrentNode->Type == NodeType::NODE) {
			assert(CurrentNode->Children[0]);
			assert(CurrentNode->Children[1]);

			IndexBuildingQueue.push(CurrentNode->Children[0]);
			IndexBuildingQueue.push(CurrentNode->Children[1]);
		}
	}

	std::queue<NodeUnserialized*> IndexConnectionQueue;
	IndexConnectionQueue.push(RootNode);

	// This vector will contain our nodes that we have processed
	std::vector<NodeSerialized> ProcessedNodes;

	// This while loop connects indices
	while (!IndexConnectionQueue.empty()) {
		NodeUnserialized* CurrentNode = IndexConnectionQueue.front();
		IndexConnectionQueue.pop();

		NodeSerialized SerializedNode;

		SerializedNode.BoundingBox = CurrentNode->BoundingBox;

		if (CurrentNode->Type == NodeType::NODE) {
			SerializedNode.ChildrenNodes[0] = CurrentNode->Children[0]->Index;
			SerializedNode.ChildrenNodes[1] = CurrentNode->Children[1]->Index;

			IndexConnectionQueue.push(CurrentNode->Children[0]);
			IndexConnectionQueue.push(CurrentNode->Children[1]);
		} else {
			SerializedNode.Leaf = CurrentNode->Leaf;
			SerializedNode.MakeLeaf();
		}
		ProcessedNodes.push_back(SerializedNode);
	}

	ConstructionTimer.End();
	ConstructionTimer.DebugTime();
	ConstructionTimer.Begin();

	Nodes = ProcessedNodes;
	LeafContents = LeafContentBuffer;

	ConstructionTimer.End();
	ConstructionTimer.DebugTime();

	//DebugPrintBVH(ProcessedNodes, LeafContentBuffer);

	//std::cout << "End of BVH construction" << std::endl;
}


void NodeSerialized::MakeLeaf(void) {
	Leaf.Size = -Leaf.Size;
}

NodeType NodeSerialized::GetType(void) {
	return Leaf.Size < 0 ? NodeType::LEAF : NodeType::NODE;
}

void IndentDebugBVH(int32_t TabCount) {
	for (int32_t Counter = 0; Counter < TabCount; Counter++) {
		printf("\t");
	}
}

void DebugPrintBVH(const std::vector<NodeSerialized>& Nodes, const std::vector<int32_t>& LeafContents) {
	struct NodeStackPrintItem {
		NodeSerialized Node;
		int32_t Depth;
	};

	NodeStackPrintItem RootNode;

	RootNode.Node = Nodes[0];
	RootNode.Depth = 0;

	int32_t CurrentDepth;

	std::stack<NodeStackPrintItem> NodeStack;
	NodeStack.push(RootNode);

	while (!NodeStack.empty()) {
		NodeStackPrintItem CurrentItem = NodeStack.top();
		NodeStack.pop();

		CurrentDepth = CurrentItem.Depth;

		int32_t NumTabs = CurrentDepth;

		IndentDebugBVH(NumTabs);

		if (CurrentItem.Node.GetType() == NodeType::NODE) {
			printf("Node: Depth %i\n", CurrentDepth);

			CurrentDepth++;

			NodeStackPrintItem ChildItems[2];

			ChildItems[0].Node = Nodes[CurrentItem.Node.ChildrenNodes[0]];
			ChildItems[0].Depth = CurrentDepth;

			ChildItems[1].Node = Nodes[CurrentItem.Node.ChildrenNodes[1]];
			ChildItems[1].Depth = CurrentDepth;

			NodeStack.push(ChildItems[0]);
			NodeStack.push(ChildItems[1]);
		}
		else {
			printf("Leaf: Offset %i\tSize %i\n", CurrentItem.Node.Leaf.Offset + 1, -CurrentItem.Node.Leaf.Size);

			IndentDebugBVH(NumTabs);

			printf("Contents: ");

			for (int32_t TriCounter = CurrentItem.Node.Leaf.Offset; TriCounter < CurrentItem.Node.Leaf.Offset - CurrentItem.Node.Leaf.Size; TriCounter++) {
				printf("%i ", LeafContents[TriCounter]);
			}

			printf("\n");
		}

		IndentDebugBVH(NumTabs);

		printf("Bounds: (%f %f %f) -> (%f %f %f)\n",
			CurrentItem.Node.BoundingBox.Min.x, CurrentItem.Node.BoundingBox.Min.y, CurrentItem.Node.BoundingBox.Min.z,
			CurrentItem.Node.BoundingBox.Max.x, CurrentItem.Node.BoundingBox.Max.y, CurrentItem.Node.BoundingBox.Max.z
		);
	}
}