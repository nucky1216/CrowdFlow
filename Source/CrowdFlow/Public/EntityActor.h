

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




public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity Properties")
	int64 EntityID; // Unique identifier for the entity

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity Properties")
	AFlowFieldVoxelBuilder* FlowFieldVoxelBuilder; // Reference to the FlowFieldVoxelBuilder

	dtPolyRef CurrentPolyRef=0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity Metadata")
	FString StrPolyRef; // String representation of the polyRef

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity Properties")
	FVector Velocity = FVector(1, 0, 0); 
	UPROPERTY(VisibleAnywhere, Category = "Entity Properties")
	UFlowFieldNeiboursSubsystem* FlowFieldNeiboursSubsystem; 

	UPROPERTY(VisibleAnywhere, Category = "Entity Properties")
	FMassEntityHandle EntityHandle; 

	UFUNCTION(BlueprintCallable, Category = "Tool")
	void ConstructHandle();

	UFUNCTION(BlueprintCallable, Category = "Tool")
	void RegistryToSubsystem();

	void SetPolyRef(dtPolyRef NewPolyRef) { CurrentPolyRef = NewPolyRef; StrPolyRef = LexToString(NewPolyRef); }
protected:
	void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	
};
