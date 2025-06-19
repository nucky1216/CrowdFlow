


#include "EntityActor.h"
#include "MassEntityTypes.h"

// Sets default values
AEntityActor::AEntityActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AEntityActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AEntityActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AEntityActor::OnConstruction(const FTransform& Transform)
{
	EntityHandle = FMassEntityHandle::FromNumber(EntityID);
	UE_LOG(LogTemp, Log, TEXT("Get ID:%d to EntityHandle:%u"),EntityID,EntityHandle.AsNumber());
}


