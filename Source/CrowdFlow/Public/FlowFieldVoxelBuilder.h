// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Detour/DetourNavMesh.h"
#include "MassEntityTypes.h"
#include "EntityActor.h"
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
	int32 Heat;

	UPROPERTY(BlueprintReadOnly)
	bool flag;

	UPROPERTY(BlueprintReadOnly)
	bool bIsValid;
};

USTRUCT(BlueprintType)
struct FNavPolyFlow
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly)
	FVector Center;

	TArray<dtPolyRef> EdgeNeibours;

	TArray<dtPolyRef> VertNeibours;

	UPROPERTY(BlueprintReadOnly)
	TArray<FVector> PolyVerts;

	UPROPERTY(BlueprintReadOnly)
	TArray<int32> BoundaryEdge;

	UPROPERTY(BlueprintReadOnly)
	FVector PolyNormal;

	UPROPERTY(BlueprintReadOnly)
	TArray<FVector> BoundaryEdgeNormal;

	UPROPERTY(BlueprintReadOnly)
	FVector FlowDirection;

	UPROPERTY(BlueprintReadOnly)
	bool bIsValid;

	UPROPERTY(BlueprintReadOnly)
	FString StrPolyRef;
};

UENUM(BlueprintType)
enum class EFlowFieldNeibourType : uint8
	{
		FaceNeibour,
		EdgeNeibour,
		VertNeibour
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
	TArray<FIntVector> IndexOffset;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	FVector TargetLocation;

	UPROPERTY(BlueprintReadOnly)
	TMap<FIntVector, FFlowVoxel> FlowField;


	UPROPERTY(EditAnywhere, Category = "Flow Field")
	float AgentRadius = 100.f;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	float DebugDrawTime = 10.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force")
	float DesiredForceStrength = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force")
	float GuidanceForceStrength = 50.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force")
	float RepelForceStrength = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force")
	float PlaneForceStrength = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force")
	float DistTolenrance = 300.f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force")
	float NeibourRepel = 50.f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force | PlaneForce")
	float SurfaceHeightTolerance = 10.f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force | PlaneForce")
	float SurfaceSpringK = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field | Force | PlaneForce")
	float SurfaceSpringDampingC = 300.0f;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	int32 MaxDrawCount = 100;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	int32 DebugStart = 100;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	int32 DebugEnd = 100;

	UPROPERTY(EditAnywhere, Category = "Flow Field")
	EFlowFieldNeibourType NeibourType = EFlowFieldNeibourType::FaceNeibour;

	UFUNCTION(CallInEditor, Category = "Flow Field")
	void GenerateFlowField();

	UFUNCTION(CallInEditor, Category = "Flow Field")
	void DebugDrawFlowField();

	UFUNCTION(BlueprintPure, Category = "Flow Field")
	FIntVector GetGridIndex(const FVector& Location) const;

	UFUNCTION(BlueprintPure, Category = "Flow Field")
	FVector SampleDirction(const FVector& Location) ;

	void ConstructNeibourOffsets();

	TMap<dtPolyRef, FNavPolyFlow> FlowFieldByPoly;

	TMultiMap<FVector, dtPolyRef> VertToPolyMap;

	UFUNCTION(CallInEditor, Category = "Flow Field | Poly")
	void GenerateFlowFieldPoly();

	UFUNCTION(CallInEditor, Category = "Flow Field | Poly")
	void DebugDrawFlowFieldPoly();

	UFUNCTION(CallInEditor,BlueprintCallable, Category = "Flow Field | Poly")
	void DebugDrawSinglePoly(FString PolyRef);

	UFUNCTION(BlueprintPure,Category="Flow Field | Poly")
	void  GetFlowPoly(TArray<FNavPolyFlow>& Polys) const {  FlowFieldByPoly.GenerateValueArray(Polys); } ;

	UFUNCTION(BlueprintCallable, Category = "Flow Field | Poly")
	FVector DeltaMove(AActor* Agent, UPARAM(ref) FVector& Velocity, float MaxSpeed=100.f, float Mass=1.f);

	UFUNCTION(BlueprintCallable,Category="Flow Field | Poly")
	FVector GetFlowByPoly(const FVector& Location, FVector CurVelocity,
		FVector& RepelForce, FVector& GuidanceForce, FVector& PlaneForce, 
		FVector ProjectExtent=FVector(50,50,200));

	UFUNCTION(BlueprintCallable, Category = "EntityNeibour")
	void DebugDrawNeibours(AEntityActor* Entity,  int32 MaxNum);

	UFUNCTION(BlueprintCallable, Category = "FlowField | Poly | Force")
	void GetForceFromNeibours(AEntityActor* EntityActor,FVector& NeiRepel);

	UFUNCTION(CallInEditor,BlueprintCallable, Category = "EntityNeibour")
	void RegistryNeibourSubsystem();

	FVector GetFlowCenter(dtPolyRef PolyRef) const;

	dtPolyRef GetPolyRef(const FVector& Location, FVector ProjectExtent) const;
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void OnConstruction(const FTransform& Transform) override;

	

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
