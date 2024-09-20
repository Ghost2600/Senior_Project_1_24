// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HAL/Event.h"
#include "WorldCollision.h"
#include <memory>
#include <chrono>
#include <vector>
#include <atomic>
#include <set>
#include <list>
#include "PhysicsInterfaceTypesCore.h"
#include "CPathDefines.h"
#include "CPathOctree.h"
#include "CPathNode.h"
#include "CPathAsyncVolumeGeneration.h"
#include "CPathVolume.generated.h"

class ACPathCore;

UCLASS()
class SENIOR_PROJECT_1_24_API ACPathVolume : public AActor
{
	GENERATED_BODY()

	friend class FCPathAsyncVolumeGenerator;
	friend class UCPathDynamicObstacle;
public:
	ACPathVolume();

	virtual void Tick(float DeltaTime) override;

	// This is the method to find get a path in c++, asynchronously. 
	// Example function you can provide: void OnPathFound(FCPathResult& PathResult);
	// You can get the function name via macro: GET_FUNCTION_NAME_CHECKED(YourUObjectType, OnPathFound);
	// Returns false if FindPath request wasn't made (happens if somehow called before begin play or if one of the volumes has been destroyed)
	bool FindPathAsync(UObject* CallingObject, const FName& InFunctionName,
		FVector Start, FVector End,
		uint32 SmoothingPasses = 2, int32 UserData = 0, float TimeLimit = 0.15f,
		bool RequestRawPath = false, bool RequestUserPath = true);

	// Same as above, just using the FCPathRequest structure to pass parameters
	bool FindPathAsync(FCPathRequest& Request);

	// This searches for a path on this thread, so the result is available here and now.
	// Increase TimeLimit at your own risk. 
	// Default time of 2ms means that in the worst case scenatio, this call will increse your frametime by 2ms!
	// Use this only for small graphs or very short paths (for example, <=1000 node graph)
	// Whenever possible, use FindPathAsync instead
	// IMPORTANT: When using with dynamic obstacles, this might fail with reason GraphNotGenerated while the graph is being updated!
	FCPathResult FindPathSynchronous(FVector Start, FVector End,
		uint32 SmoothingPasses = 2, int32 UserData = 0, float TimeLimit = 0.002f,
		bool RequestRawPath = false, bool RequestUserPath = true);


	// Blueprint exposed version
	// This searches for a path on this thread, so the result is available here and now.
	// Increase TimeLimit at your own risk. 
	// Default time of 2ms means that in the worst case scenatio, this call will increse your frametime by 2ms!
	// Use this only for small graphs or very short paths (for example, <=1000 node graph)
	// Whenever possible, use FindPathAsync instead
	// IMPORTANT: When using with dynamic obstacles, this might fail with reason GraphNotGenerated while the graph is being updated!
	UFUNCTION(BlueprintCallable, Category = "CPath", Meta = (ExpandEnumAsExecs = "Branches"))
		void FindPathSynchronous(BranchFailSuccessEnum Branches, TArray<FCPathNode>& Path, ECPathfindingFailReason FailReason,
			 FVector Start, FVector End, int SmoothingPasses = 2,
			int UserData = 0, float TimeLimit = 0.002f);


	// ------- EXTENDABLE ------

	// Overwrite this function to change the priority of nodes as they are selected for the path.
	// Note that this is potentially called thousands of times per FindPath call, so it shouldnt be too complex (unless your graph not very dense)
	virtual void CalcFitness(CPathAStarNode& Node, FVector TargetLocation, int32 UserData);

	// Overwrite this function to change the default conditions of a tree being free/ocupied.
	// You may also save other information in the Data field of an Octree, as only the least significant bit is used.
	// This is called during graph generation, for every subtree including leafs, so potentially millions of times. 
	virtual bool RecheckOctreeAtDepth(CPathOctree* OctreeRef, FVector TreeLocation, uint32 Depth);


	// -------- BP EXPOSED ----------

	//Box to mark the area to generate graph in. It should not be rotated, the rotation will be ignored.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Components", meta = (EditCondition = "GenerationStarted==false"))
		class UBoxComponent* VolumeBox;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false"))
		TEnumAsByte<ECollisionChannel>  TraceChannel = ECollisionChannel::ECC_Visibility;

	// Spports Capsule, sphere and box.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false"))
		EAgentShape AgentShape = EAgentShape::Capsule;

	// In case of a box, this is X and Y extent. Z and Y should be the same, since the actual agent will most likely rotate.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0", UIMin = "0"))
		float AgentRadius = 0;

	// In case of a box, this is Z extent.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false && AgentShape!=EAgentShape::Sphere", ClampMin = "0", UIMin = "0"))
		float AgentHalfHeight = 0;




	// Size of the smallest voxel edge.
	// In most cases, setting this to min(AgentRadius, AgentHalfHeight)*2 is enough.
	// For precise (dense) graph - set this to min(AgentRadius, AgentHalfHeight).
	// Small values increase memory cost, and potentially CPU load.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0.1", UIMin = "0.1"))
		float VoxelSize = 60;

	// How many times per second do parts of the volume get regenerated based on dynamic obstacles, in seconds.
	// Values higher than 5 are an overkill, but for the purpose of user freedom, I leave it unlocked.
	// If no dynamic obstacles were added, this doesnt have any performance impact
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0.01", UIMin = "0.01", ClampMax = "30", UIMax = "30"))
		float DynamicObstaclesUpdateRate = 3;

	// 2 Is optimal in most cases. If you have very large open speces with small amount of obstacles, then 3 will be better.
	// For dense labirynths with little to no open space, 1 or even 0 will be faster.
	// Check documentation for detailed performance guidance.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0", ClampMax = "3", UIMin = "0", UIMax = "3"))
		int OctreeDepth = 2;


	// If want to call Generate() later or with some condition.
	// Note that volume wont be usable before it is generated
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath")
		bool GenerateOnBeginPlay = true;

	// Set a custom generation thread limit. By default, it's system's Physical Core count - 1.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath")
		bool OverwriteMaxGenerationThreads = false;

	// How many threads can graph generation split into. 
	// If left <=0 (RECOMMENDED), it uses system's Physical Core count - 1. 
	// Generation threads are allocated dynamically, so it only uses more than 1 thread when necessary.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false && OverwriteMaxGenerationThreads==true", ClampMin = "0", ClampMax = "31", UIMin = "0", UIMax = "31"))
		int MaxGenerationThreads = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Render")
		bool DrawFree = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Render")
		bool DrawOccupied = false;

	// You can hide selected depths from rendering
	UPROPERTY(EditAnywhere, BlueprintReadOnly, EditFixedSize, Category = "CPath|Render")
		TArray<bool> DepthsToDraw;

	// How thick should the green and red debug boxes be
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CPath|Render")
		float DebugBoxesThickness = 1.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CPath|Render")
		float DebugPathThickness = 1.5f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		bool GenerationStarted = false;

	// This is a read only info about initially generated graph
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		TArray<int> OctreeCountAtDepth = { 0, 0, 0, 0 };

	// This is a read only info about initially generated graph
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		int TotalNodeCount = 0;

	// Draws FREE neighbouring leafs
	UFUNCTION(BlueprintCallable, Category = "CPath|Render")
		void DebugDrawNeighbours(FVector WorldLocation);

	// Draws the octree structure around WorldLocation, up tp VoxelLlimit
	UFUNCTION(BlueprintCallable, Category = "CPath|Render")
		void DrawDebugNodesAroundLocation(FVector WorldLocation, int VoxelLimit, float Duration);

	// Draws path with points, for visualization only
	// Duration == 0 - draw for one frame
	// Duration < 0 - persistent
	UFUNCTION(BlueprintCallable, Category = "CPath|Render")
		void DrawDebugPath(const TArray<FCPathNode>& Path, float Duration, bool DrawPoints = true, FColor Color = FColor::Magenta);

	// Before this is true, the graph is inoperable
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		bool InitialGenerationFinished = false;

	// WARNING: This will freeze the game for the benchmark duration!
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark")
		bool PerformBenchmarkAfterGeneration = false;

	// Benchmark duration in seconds
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark")
		float BenchmarkDurationSeconds = 20.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark")
		int BenchmarkFindPathUserData = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark")
		float BenchmarkFindPathTimeLimit = 0.1;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark")
		FString BenchmarkName = FString("DefaultName");

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark")
		bool SaveBenchmarkResultToFile = true;

	// if sample size of successful searches is too low, don't save the results to file
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark", meta = (EditCondition = "SaveBenchmarkResultToFile==true"))
		bool SaveBenchmarksWithUnreliableResults = false;

	// Not used for now, async benchmark does not provide reliable results
	//UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Benchmark")
	bool IsAsyncBenchmark = false;


	// Shapes to use when checking if voxel is free or not
	std::vector<std::vector<FCollisionShape>> TraceShapesByDepth;

	// Returns false if graph couldnt start generating
	bool GenerateGraph();

	// These are called by UE in this order
	virtual void EndPlay(EEndPlayReason::Type EndPlayReason) override;
	virtual void BeginDestroy() override;
	virtual bool IsReadyForFinishDestroy() override;
	virtual void FinishDestroy() override;

protected:

	virtual void BeginPlay() override;

	// The Octree data
	CPathOctree* Octrees = nullptr;

	// This is for find path requests, shouldn't be accessed directly unless you know what you're doing
	// UPROPERTY() is here so that UE's garabge collector doesn't randomly
	// decide that this is useless and destroy it -_-
	UPROPERTY()
	ACPathCore* CoreInstance = nullptr;
public:

	// Location of the first voxel, set during graph generation
	FVector StartPosition;

	// Dimension sizes of the Nodes array, XYZ 
	uint32 NodeCount[3];

	//----------- TreeID ------------------------------------------------------------------------

	// Returns the child with this tree id, or his parent at DepthReached in case the child doesnt exist
	CPathOctree* FindTreeByID(uint32 TreeID, uint32& DepthReached);

	CPathOctree* FindTreeByID(uint32 TreeID);

	// Returns a tree and its TreeID by world location, returns null if location outside of volume. Only for Outer index
	CPathOctree* FindTreeByWorldLocation(FVector WorldLocation, uint32& TreeID);

	// Returns a leaf and its TreeID by world location, returns null if location outside of volume. 
	CPathOctree* FindLeafByWorldLocation(FVector WorldLocation, uint32& TreeID, bool MustBeFree = 1);

	// Returns a free leaf and its TreeID by world location, as long as it exists in provided search range and WorldLocation is in this Volume
	// If SearchRange <= 0, it uses a default dynamic search range
	// If SearchRange is too large, you might get a free node that is inaccessible from provided WorldLocation
	CPathOctree* FindClosestFreeLeaf(FVector WorldLocation, uint32& TreeID, float SearchRange = -1);

	// Returns a neighbour of the tree with TreeID in given direction, also returns  TreeID if the neighbour if found
	CPathOctree* FindNeighbourByID(uint32 TreeID, ENeighbourDirection Direction, uint32& NeighbourID);

	// Returns a list of adjecent leafs as TreeIDs
	std::vector<uint32> FindNeighbourLeafs(uint32 TreeID, bool MustBeFree = true);

	// Returns a list of adjecent free leafs as CPathAStarNode
	std::vector<CPathAStarNode> FindFreeNeighbourLeafs(CPathAStarNode& Node);

	// Returns a parent of tree with given TreeID or null if TreeID has depth of 0
	FORCEINLINE CPathOctree* GetParentTree(uint32 TreeId)
	{
		uint32 Depth = ExtractDepth(TreeId);
		if (Depth)
		{
			ReplaceDepth(TreeId, Depth - 1);
			return FindTreeByID(TreeId, Depth);
		}
		return nullptr;
	};

	// Returns world location of a voxel at this TreeID. This returns CENTER of the voxel
	FORCEINLINE FVector WorldLocationFromTreeID(uint32 TreeID) const
	{
		uint32 OuterIndex = ExtractOuterIndex(TreeID);
		uint32 Depth = ExtractDepth(TreeID);

		FVector CurrPosition = StartPosition + GetVoxelSizeByDepth(0) * LocalCoordsInt3FromOuterIndex(OuterIndex);

		for (uint32 CurrDepth = 1; CurrDepth <= Depth; CurrDepth++)
		{
			CurrPosition += GetVoxelSizeByDepth(CurrDepth) * 0.5f * LookupTable_ChildPositionOffsetMaskByIndex[ExtractChildIndex(TreeID, CurrDepth)];
		}

		return CurrPosition;
	}

	FORCEINLINE FVector LocalCoordsInt3FromOuterIndex(uint32 OuterIndex) const
	{
		uint32 X = OuterIndex / (NodeCount[1] * NodeCount[2]);
		OuterIndex -= X * NodeCount[1] * NodeCount[2];
		return FVector(X, OuterIndex / NodeCount[2], OuterIndex % NodeCount[2]);
	};

	// Creates TreeID for AsyncOverlapByChannel
	FORCEINLINE uint32 CreateTreeID(uint32 Index, uint32 Depth) const
	{
		checkf(Depth <= MAX_DEPTH, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
		Index |= Depth << DEPTH_0_BITS;
		return Index;
	}

	// Extracts Octrees array index from TreeID
	FORCEINLINE uint32 ExtractOuterIndex(uint32 TreeID) const
	{
		return TreeID & DEPTH_0_MASK;
	}

	// Replaces Depth in the TreeID with NewDepth
	FORCEINLINE void ReplaceDepth(uint32& TreeID, uint32 NewDepth)
	{
		checkf(NewDepth <= MAX_DEPTH, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
		TreeID &= ~DEPTH_MASK;
		TreeID |= NewDepth << DEPTH_0_BITS;
	}

	// Extracts depth from TreeID
	FORCEINLINE uint32 ExtractDepth(uint32 TreeID) const
	{
		return (TreeID & DEPTH_MASK) >> DEPTH_0_BITS;
	}

	// Returns a number from  0 to 7 - a child index at requested Depth
	FORCEINLINE uint32 ExtractChildIndex(uint32 TreeID, uint32 Depth) const
	{
		checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
		uint32 DepthOffset = (Depth - 1) * 3 + DEPTH_0_BITS + 2;
		uint32 Mask = 0x00000007 << DepthOffset;
		return (TreeID & Mask) >> DepthOffset;
	}

	// This assumes that child index at Depth is 000, if its not use ReplaceChildIndex
	FORCEINLINE void AddChildIndex(uint32& TreeID, uint32 Depth, uint32 ChildIndex)
	{
		checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
		checkf(ChildIndex < 8, TEXT("CPATH - Graph Generation:::Child Index can be up to 7"));
		ChildIndex <<= (Depth - 1) * 3 + DEPTH_0_BITS + 2;
		TreeID |= ChildIndex;
	};

	// Replaces child index at given depth
	FORCEINLINE void ReplaceChildIndex(uint32& TreeID, uint32 Depth, uint32 ChildIndex)
	{
		checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
		checkf(ChildIndex < 8, TEXT("CPATH - Graph Generation:::Child Index can be up to 7"));
		uint32 DepthOffset = (Depth - 1) * 3 + DEPTH_0_BITS + 2;

		// Clearing previous child index
		TreeID &= ~(0x00000007 << DepthOffset);
		ChildIndex <<= DepthOffset;
		TreeID |= ChildIndex;
	}


	// Replaces child index at given depth and also replaces depth to the same one
	FORCEINLINE void ReplaceChildIndexAndDepth(uint32& TreeID, uint32 Depth, uint32 ChildIndex)
	{
		checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
		checkf(ChildIndex < 8, TEXT("CPATH - Graph Generation:::Child Index can be up to 7"));
		uint32 DepthOffset = (Depth - 1) * 3 + DEPTH_0_BITS + 2;

		// Clearing previous child index
		TreeID &= ~(0x00000007 << DepthOffset);
		ChildIndex <<= DepthOffset;
		TreeID |= ChildIndex;
		ReplaceDepth(TreeID, Depth);
	}

	// Traverses the tree downwards and adds every tree to the container
	void GetAllSubtrees(uint32 TreeID, std::vector<uint32>& Container);

	// Volume is not safe to access as long as this is not 0, pathfinders should wait till this is 0
	std::atomic_int GeneratorsRunning = 0;

	// Wake up call for pathfinding threads waiting for generation to finish
	FEvent* GenerationFinishedSemaphore = nullptr;

	// Volume wont start generating as long as this is not 0
	std::atomic_int PathfindersRunning = 0;
	std::atomic_int PathfindersWaiting = 0;

	// This is for other threads to check if graph is accessible
	std::atomic_bool InitialGenerationCompleteAtom = false;

	// This is filled by DynamicObstacle component
	std::set<class UCPathDynamicObstacle*> TrackedDynamicObstacles;

	// ----------- Other helper functions ---------------------

	FORCEINLINE float GetVoxelSizeByDepth(int Depth) const
	{
		checkf(Depth <= OctreeDepth, TEXT("CPATH - Graph Generation:::DEPTH was higher than OctreeDepth"));
		return LookupTable_VoxelSizeByDepth[Depth];
	}

	// Draws the voxel, this takes all the drawing options into condition. If Duraiton is below 0, it never disappears. 
	// If Color = green, free trees are green and occupied are red.
	// Returns true if drawn, false otherwise
	bool DrawDebugVoxel(uint32 TreeID, bool DrawIfNotLeaf = true, float Duration = 0, FColor Color = FColor::Green, CPathVoxelDrawData* OutDrawData = nullptr);
	void DrawDebugVoxel(const CPathVoxelDrawData& DrawData, float Duration) const;

protected:

	// Returns an index in the Octree array from world position. NO BOUNDS CHECK
	FORCEINLINE int WorldLocationToIndex(FVector WorldLocation) const
	{
		FVector XYZ = WorldLocationToLocalCoordsInt3(WorldLocation);
		return LocalCoordsInt3ToIndex(XYZ);
	}

	// Multiplies local integer coordinates into index
	FORCEINLINE float LocalCoordsInt3ToIndex(FVector V) const
	{
		return (V.X * (NodeCount[1] * NodeCount[2])) + (V.Y * NodeCount[2]) + V.Z;
	}

	// Returns the X Y and Z relative to StartPosition and divided by VoxelSize. Multiply them to get the index. NO BOUNDS CHECK
	FVector WorldLocationToLocalCoordsInt3(FVector WorldLocation) const;

	// Returns world location of a tree at depth 0. Extracts only outer index from TreeID
	FORCEINLINE FVector GetOuterTreeWorldLocation(uint32 TreeID) const
	{
		FVector LocalCoords = LocalCoordsInt3FromOuterIndex(ExtractOuterIndex(TreeID));
		LocalCoords *= GetVoxelSizeByDepth(0);
		return StartPosition + LocalCoords;
	}

	// takes in what `WorldLocationToLocalCoordsInt3` returns and performs a bounds check
	bool IsInBounds(FVector LocalCoordsInt3) const;

	// Helper function for 'FindLeafByWorldLocation'. Relative location is location relative to the middle of CurrentTree
	CPathOctree* FindLeafRecursive(FVector RelativeLocation, uint32& TreeID, uint32 CurrentDepth, CPathOctree* CurrentTree);

	// Returns IDs of all free leafs on chosen side of a tree. Sides are indexed in the same way as neighbours, and adds them to passed Vector.
	// ASSUMES THAT PASSED TREE HAS CHILDREN
	void FindLeafsOnSide(uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector, bool MustBeFree = true);

	// Same as above, but skips the part of getting a tree by TreeID so its faster
	void FindLeafsOnSide(CPathOctree* Tree, uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector, bool MustBeFree = true);

	// Same as above, but wrapped in CPathAStarNode
	void FindLeafsOnSide(CPathOctree* Tree, uint32 TreeID, ENeighbourDirection Side, std::vector<CPathAStarNode>* Vector, bool MustBeFree = true);

	// Internal function used in GetAllSubtrees
	void GetAllSubtreesRec(uint32 TreeID, CPathOctree* Tree, std::vector<uint32>& Container, uint32 Depth);


	// -------- GENERATION -----
	FTimerHandle GenerationTimerHandle;

	std::list<std::unique_ptr<FCPathAsyncVolumeGenerator>> GeneratorThreads;

	// Garbage collection
	void CleanFinishedGenerators();

	// Checking if initial generation has finished
	void InitialGenerationUpdate();

	// Checking if there are any trees to regenerate from dynamic obstacles
	void GenerationUpdate();

	std::set<int32> TreesToRegenerate;

	// This is so that when an actor moves, the previous space it was in needs to be regenerated as well
	std::set<int32> TreesToRegeneratePreviousUpdate;

	// This is set in GenerateGraph() using a formula that estimates total voxel count
	int OuterIndexesPerThread;

	bool ThreadIDs[64];

	uint32 GetFreeThreadID() const;

	void PerformRandomBenchmark(uint32 UserData = 0, float TimeLimit = 0.2);

	// ----- Lookup tables-------
	static const FVector LookupTable_ChildPositionOffsetMaskByIndex[8];
	static const FVector LookupTable_NeighbourOffsetByDirection[6];

	// Positive values = ChildIndex of the same parent, negative values = (-ChildIndex - 1) of neighbour at Direction [6]
	static const int8 LookupTable_NeighbourChildIndex[8][6];

	// First index is side as in ENeighbourDirection, and then you get indices of children on that side (in ascending order)
	static const int8 LookupTable_ChildrenOnSide[6][4];

	// Left returns right, up returns down, Front returns behind, etc
	static const int8 LookupTable_OppositeSide[6];

	// Set in begin play
	float LookupTable_VoxelSizeByDepth[MAX_DEPTH + 1];


	// -------- DEBUGGING -----
	std::vector<CPathVoxelDrawData> PreviousDrawAroundLocationData;

	std::chrono::steady_clock::time_point GenerationStart;
	bool PrintGenerationTime = false;


};
