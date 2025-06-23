// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Tickable.h"
#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Detour/DetourNavMesh.h"
#include "MassEntityTypes.h"
#include "FlowFieldNeiboursSubsystem.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnEntityRegistered, float, DeltaTime);

class AEntityActor;
class AFlowFieldVoxelBuilder;


/**
 * 
 */
UCLASS()
class CROWDFLOW_API UFlowFieldNeiboursSubsystem : public UWorldSubsystem, public FTickableGameObject
{
	GENERATED_BODY()
	
public:
	TMap<dtPolyRef, TArray<FMassEntityHandle>> PolyNeibours;

	AFlowFieldVoxelBuilder* FlowFieldBuilder = nullptr;

	TMap<FMassEntityHandle, AEntityActor*> EntityToActor;

	void RegisterPolyEntity(dtPolyRef PolyRef,FMassEntityHandle Entity);

	void UnregisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle Entity);

	void UpdatePolyEntity(dtPolyRef NewPolyRef,dtPolyRef OldPolyRef, FMassEntityHandle Entity);

	TArray<FMassEntityHandle> GetPolyEntities(dtPolyRef PolyRef,int32 MaxNum) const;

	UFUNCTION(BlueprintCallable, Category = "Subsystem")
	void ResetMap()	{		PolyNeibours.Reset();	}

	UFUNCTION(BlueprintCallable, Category = "Subsystem")
	void DebugMap();

	UFUNCTION(BlueprintCallable, Category = "Subsystem")
	void InitializeManual(AFlowFieldVoxelBuilder* FlowField);

	UFUNCTION(BlueprintCallable, Category = "Subsystem")
	void RegistryMassEntity(AEntityActor* Entity);

	UPROPERTY(BlueprintAssignable, Category = "Subsystem")
	FOnEntityRegistered OnEntityRegistered;

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Tick(float DeltaTime) override;
	virtual bool IsTickable() const override { return true; }
	virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(UFlowFieldNeiboursSubsystem, STATGROUP_Tickables); }

};
