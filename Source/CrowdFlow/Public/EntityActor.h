

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Detour/DetourNavMesh.h"
#include "EntityActor.generated.h"
class AFlowFieldVoxelBuilder; // Forward declaration to avoid circular dependency
UCLASS()
class CROWDFLOW_API AEntityActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AEntityActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity Properties")
	int64 EntityID; // Unique identifier for the entity

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity Properties")
	AFlowFieldVoxelBuilder* FlowFieldVoxelBuilder; // Reference to the FlowFieldVoxelBuilder

	dtPolyRef CurrentPolyRef=0; 

	UPROPERTY(VisibleAnywhere, Category = "Entity Properties")
	FMassEntityHandle EntityHandle; 

protected:
	void OnConstruction(const FTransform& Transform) override;
	
	
};
