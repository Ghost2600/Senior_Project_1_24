// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

class ACPathVolume;
class CPathOctree;




class SENIOR_PROJECT_1_24_API FCPathAsyncVolumeGenerator : public FRunnable
{


public:
	// Geneated trees in range Start(inclusive) - End(not inclusive). If Obstacles = true, it takes from Volume->TreesToRegenerate, if not, it takes from Volume->Octrees (default)
	FCPathAsyncVolumeGenerator(ACPathVolume* Volume, uint32 StartIndex, uint32 EndIndex, uint8 ThreadID, FString ThreadName, bool Obstacles = false);

	// Not used for now
	FCPathAsyncVolumeGenerator(ACPathVolume* Volume);

	~FCPathAsyncVolumeGenerator();

	virtual bool Init();

	virtual uint32 Run();

	virtual void Stop();

	virtual void Exit();

	bool HasFinishedWorking();

	// The main generating function, generated/regenerates the whole octree at given index
	void RefreshTree(uint32 OuterIndex);

	bool bObstacles = false;

	FRunnableThread* ThreadRef = nullptr;

	uint8 GenThreadID;

	static FString GetNameFromID(uint8 ID);

	FString Name = "";

	uint32 OctreeCountAtDepth[4] = { 0, 0, 0, 0 };


protected:

	ACPathVolume* VolumeRef;

	uint32 FirstIndex = 0;
	uint32 LastIndex = 0;
	std::atomic_bool RequestedKill = false;
	std::atomic_bool ThreadExited = false;

	bool bIncreasedGenRunning = false;

	// Gets called by RefreshTree. Returns true if ANY child is free
	bool RefreshTreeRec(CPathOctree* OctreeRef, uint32 Depth, FVector TreeLocation);

	bool ShouldWakeUp();

	TFunctionRef< bool()> WakeUpCondition;

public:

};
