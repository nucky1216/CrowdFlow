


#include "EntityActor.h"
#include "MassEntityTypes.h"
#include "FlowFieldVoxelBuilder.h"

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
	ConstructHandle();
}

// Called every frame
void AEntityActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AEntityActor::ConstructHandle()
{
	EntityHandle = FMassEntityHandle::FromNumber(EntityID);
	UE_LOG(LogTemp, Log, TEXT("Get ID:%lld to EntityHandle:%llu"), EntityID, EntityHandle.AsNumber());
}

void AEntityActor::RegistryToSubsystem()
{
	FlowFieldNeiboursSubsystem = GetWorld()->GetSubsystem<UFlowFieldNeiboursSubsystem>();
	if (!FlowFieldNeiboursSubsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlowFieldNeiboursSubsystem is not set in AEntityActor! "));
		return;
	}
	FlowFieldNeiboursSubsystem->RegistryMassEntity(this);
}

void AEntityActor::OnConstruction(const FTransform& Transform)
{
	ConstructHandle();
}


