// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlowFieldVoxelBuilder.generated.h"

USTRUCT(BlueprintType)
struct FFlowVoxel
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly)
	FVector WorldPosition;

	UPROPERTY(BlueprintReadOnly)
	FVector FlowDirection;

	UPROPERTY(BlueprintReadOnly)
	bool bIsValid;
};
UCLASS()
class CROWDFLOW_API AFlowFieldVoxelBuilder : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFlowFieldVoxelBuilder();

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	FVector VolumeSize = FVector(2000, 2000, 800);

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	float VoxelSize = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	FVector TargetLocation;

	UPROPERTY(BlueprintReadOnly)
	TMap<FIntVector, FFlowVoxel> FlowField;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	int32 SamplesPerTriangle = 3;

	UFUNCTION(CallInEditor, Category = "Flow Field")
	void GenerateFlowField();

	UFUNCTION(CallInEditor, Category = "Flow Field")
	void DebugDrawFlowField();

	UFUNCTION(BlueprintPure, Category = "Flow Field")
	FIntVector GetGridIndex(const FVector& Location) const;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void OnConstruction(const FTransform& Transform) override;

	

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
