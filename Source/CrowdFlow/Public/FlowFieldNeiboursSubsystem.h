// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Detour/DetourNavMesh.h"
#include "MassEntityTypes.h"
#include "FlowFieldNeiboursSubsystem.generated.h"

class AFlowFieldVoxelBuilder;
/**
 * 
 */
UCLASS()
class CROWDFLOW_API UFlowFieldNeiboursSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
	
public:
	TMap<dtPolyRef, TArray<FMassEntityHandle>> PolyNeibours;
	AFlowFieldVoxelBuilder* FlowFieldBuilder = nullptr;

	void RegisterPolyEntity(dtPolyRef PolyRef,FMassEntityHandle Entity);

	void UnregisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle Entity);

	void UpdatePolyEntity(dtPolyRef NewPolyRef,dtPolyRef OldPolyRef, FMassEntityHandle Entity);

	TArray<FMassEntityHandle> GetPolyEntities(dtPolyRef PolyRef,int32 MaxNum) const;

	UFUNCTION(BlueprintCallable, Category = "Subsystem")
	void ResetMap()	{		PolyNeibours.Reset();	}

	UFUNCTION(BlueprintCallable, Category = "Subsystem")
	void DebugMap();

	UFUNCTION(BlueprintCallable, Category = "Subsystem")
	void InitializeManual();


	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

	
	
};
