// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Detour/DetourNavMesh.h"
#include "MassEntityTypes.h"
#include "FlowFieldNeiboursSubsystem.generated.h"

/**
 * 
 */
UCLASS()
class CROWDFLOW_API UFlowFieldNeiboursSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
	
public:
	TMap<dtPolyRef, TArray<FMassEntityHandle>> PolyNeibours;

	void RegisterPolyEntity(dtPolyRef PolyRef,FMassEntityHandle Entity);

	void UnregisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle Entity);

	const TArray<FMassEntityHandle>* GetPolyEntities(dtPolyRef PolyRef) const;

	void ResetMap()	{		PolyNeibours.Reset();	}
	
};
